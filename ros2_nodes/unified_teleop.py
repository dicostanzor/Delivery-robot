#!/usr/bin/env python3
# unified_teleop.py
# Runs on the Jetson Orin NX -- SINGLE SESSION VERSION
#
# On startup this script itself brings up can0, launches the motor
# control node, platform_receiver.py, and actuator_receiver.py as
# background subprocesses (logs go to /tmp/*.log, not this terminal),
# then drops into the keyboard teleop:
#   - Mecanum drive         -> publishes geometry_msgs/Twist to /cmd_vel
#   - Lift raise/lower      -> TCP "raise"/"lower"/"stop" to platform_receiver.py :5007
#   - Tilt/level (axis 0)   -> TCP "tilt"/"level" to platform_receiver.py :5007
#   - Actuator              -> TCP "actuator" to actuator_receiver.py :5006
#
# On exit (CTRL-C) it stops the robot, sends a platform stop, and
# terminates the subprocesses it started. can0 is left up.
#
# Run with (one SSH session, one terminal):
#   source ~/ros2_ws/install/setup.bash
#   python3 unified_teleop.py
#
# You will be prompted for your sudo password once at startup for the
# can0 bring-up -- that's expected, type it and continue.

import os
import sys
import tty
import time
import termios
import socket
import signal
import subprocess
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# --- Config ---
# These assume this script runs on the Jetson alongside the receivers.
# If you run it from another machine, change to 192.168.137.2 (ICS) or
# the Jetson's Tailscale IP (100.66.202.98), and set LAUNCH_SUBPROCESSES
# to False since there's nothing local to launch.
PLATFORM_HOST = '127.0.0.1'
PLATFORM_PORT = 5007
ACTUATOR_HOST = '127.0.0.1'
ACTUATOR_PORT = 5006

# Set to False if you'd rather keep starting these manually in their
# own terminals like before.
LAUNCH_SUBPROCESSES = True

# Verify/adjust these paths for your setup.
ROS2_WS_SETUP = os.path.expanduser('~/ros2_ws/install/setup.bash')
MOTOR_CONTROL_NODE = os.path.expanduser(
    '~/ros2_ws/install/motor_control/lib/motor_control/motor_control_node')
PHOENIX_LIB_PATH = '/usr/local/lib:/home/nvidia/Phoenix5-Linux-Example/lib'
PLATFORM_RECEIVER_PATH = os.path.expanduser('~/platform_receiver.py')
ACTUATOR_RECEIVER_PATH = os.path.expanduser('~/actuator_receiver.py')

LOG_DIR = '/tmp'

LINEAR_SPEED = 0.3    # m/s, adjust live with q/z
ANGULAR_SPEED = 1.0   # rad/s, adjust live with q/z

# --- Movement keybindings: key -> (vx, vy, wz) multiplier ---
# Lowercase = classic 3x3 grid (diagonals blend forward/back with strafe):
#   u i o
#   j k l
#   m , .
# Shift+key (capitals) = pure strafe, no rotation blended in:
#   U I O
#   J   L
#   M < >
MOVE_BINDINGS = {
    # lowercase
    'u': (1, 1, 0),      # forward-left
    'i': (1, 0, 0),      # forward
    'o': (1, -1, 0),     # forward-right
    'j': (0, 0, 1),      # rotate left
    'l': (0, 0, -1),     # rotate right
    'm': (-1, 1, 0),     # back-left
    ',': (-1, 0, 0),     # back
    '.': (-1, -1, 0),    # back-right

    # shift-held: pure strafe, no rotation
    'U': (1, 1, 0),      # strafe forward-left
    'I': (1, 0, 0),      # forward (no rotation, same as i)
    'O': (1, -1, 0),     # strafe forward-right
    'J': (0, 1, 0),      # strafe left (pure, no rotation)
    'L': (0, -1, 0),     # strafe right (pure, no rotation)
    'M': (-1, 1, 0),     # strafe back-left
    '<': (-1, 0, 0),     # back (no rotation, same as ,)
    '>': (-1, -1, 0),    # strafe back-right
}
STOP_KEYS = ('k', 'K')

# --- Platform (lift + tilt) keybindings: key -> command string ---
PLATFORM_KEYS = {
    'r': 'raise',   # lift up, all 3 axes, runs until stop
    'f': 'lower',   # lift down, all 3 axes, runs until stop
    't': 'tilt',    # axis 0 only, single-shot 200-step move
    'g': 'level',   # axis 0 only, single-shot 200-step move
    'x': 'stop',    # halts raise/lower motion (send after r or f)
}

EXTEND_KEY = 'e'
RETRACT_KEY = 'd'

SPEED_UP_KEY = 'q'
SPEED_DOWN_KEY = 'z'

HELP = """
ELARA unified teleop
---------------------
Movement (mecanum):
  u i o       forward-left / forward / forward-right
  j k l       rotate left / STOP / rotate right
  m , .       back-left / back / back-right
  (hold Shift on any of the above for pure strafe, no rotation --
   Shift+J / Shift+L give pure left/right strafe)
  q / z       increase / decrease speed

Lift (axes 1 & 2, plus axis 0 assist):
  r           raise -- keeps moving until 'x'
  f           lower -- keeps moving until 'x'
  x           stop lift motion

Tilt / level (front motor, axis 0 only):
  t           tilt
  g           level

Actuator:
  e           extend actuator
  d           retract actuator

CTRL-C to quit (sends stop everywhere first)
"""


def bring_up_can():
    print("Bringing up can0 (sudo password may be requested)...")
    subprocess.run(['sudo', 'ip', 'link', 'set', 'can0', 'down'], check=False)
    result = subprocess.run(
        ['sudo', 'ip', 'link', 'set', 'can0', 'up', 'type', 'can',
         'bitrate', '1000000', 'restart-ms', '100'],
        check=False)
    if result.returncode != 0:
        print("  [!] can0 bring-up failed -- check wiring/adapter, "
              "or bring it up manually and rerun.")
    else:
        print("  can0 up.")


def launch_logged(cmd, log_name, env=None):
    """Launch a subprocess with stdout/stderr sent to a log file in LOG_DIR."""
    log_path = os.path.join(LOG_DIR, log_name)
    log_file = open(log_path, 'w')
    proc = subprocess.Popen(
        cmd, stdout=log_file, stderr=subprocess.STDOUT,
        env=env, preexec_fn=os.setsid)
    print(f"  started {log_name.replace('.log', '')} (pid {proc.pid}), "
          f"log: {log_path}")
    return proc


def launch_subprocesses():
    procs = []

    # Motor control node -- must NOT run under sudo.
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = '0'
    env['LD_LIBRARY_PATH'] = PHOENIX_LIB_PATH + ':' + env.get('LD_LIBRARY_PATH', '')
    motor_cmd = (
        f"source {ROS2_WS_SETUP} && exec {MOTOR_CONTROL_NODE}"
    )
    procs.append(launch_logged(
        ['bash', '-c', motor_cmd], 'motor_control_node.log', env=env))

    # Platform receiver (lift/tilt via RS485).
    procs.append(launch_logged(
        ['python3', PLATFORM_RECEIVER_PATH], 'platform_receiver.log'))

    # Actuator receiver.
    procs.append(launch_logged(
        ['python3', ACTUATOR_RECEIVER_PATH], 'actuator_receiver.log'))

    print("Waiting a few seconds for everything to come up...")
    time.sleep(4)
    return procs


def shutdown_subprocesses(procs):
    for proc in procs:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except Exception:
            pass
    for proc in procs:
        try:
            proc.wait(timeout=3)
        except Exception:
            pass


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def send_platform_command(command):
    try:
        s = socket.socket()
        s.settimeout(1)
        s.connect((PLATFORM_HOST, PLATFORM_PORT))
        s.send(command.encode())
        s.close()
        print(f"  [platform] {command}")
    except Exception as e:
        print(f"  [!] platform command error: {e}")


def send_extend_command():
    def _run():
        try:
            s = socket.socket()
            s.settimeout(5)
            s.connect((ACTUATOR_HOST, ACTUATOR_PORT))
            s.send(b"EXTEND")
            s.close()
            print("  [actuator] extend sent")
        except Exception as e:
            print(f"  [!] actuator error: {e}")
    threading.Thread(target=_run, daemon=True).start()


def send_retract_command():
    def _run():
        try:
            s = socket.socket()
            s.settimeout(5)
            s.connect((ACTUATOR_HOST, ACTUATOR_PORT))
            s.send(b"RETRACT")
            s.close()
            print("  [actuator] retract sent")
        except Exception as e:
            print(f"  [!] actuator error: {e}")
    threading.Thread(target=_run, daemon=True).start()


def main():
    settings = termios.tcgetattr(sys.stdin)

    procs = []
    if LAUNCH_SUBPROCESSES:
        bring_up_can()
        procs = launch_subprocesses()

    rclpy.init()
    node = Node('unified_teleop')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    linear_speed = LINEAR_SPEED
    angular_speed = ANGULAR_SPEED

    print(HELP)

    try:
        while True:
            key = get_key(settings)

            if key in MOVE_BINDINGS:
                vx, vy, wz = MOVE_BINDINGS[key]
                twist = Twist()
                twist.linear.x = float(vx * linear_speed)
                twist.linear.y = float(vy * linear_speed)
                twist.angular.z = float(wz * angular_speed)
                pub.publish(twist)

            elif key in STOP_KEYS:
                pub.publish(Twist())  # zero everything

            elif key in PLATFORM_KEYS:
                send_platform_command(PLATFORM_KEYS[key])

            elif key == EXTEND_KEY:
                send_extend_command()

            elif key == RETRACT_KEY:
                send_retract_command()

            elif key == SPEED_UP_KEY:
                linear_speed *= 1.1
                angular_speed *= 1.1
                print(f"  speed: linear={linear_speed:.2f} angular={angular_speed:.2f}")

            elif key == SPEED_DOWN_KEY:
                linear_speed *= 0.9
                angular_speed *= 0.9
                print(f"  speed: linear={linear_speed:.2f} angular={angular_speed:.2f}")

            elif key == '\x03':  # CTRL-C
                break

    except Exception as e:
        print(e)

    finally:
        pub.publish(Twist())
        send_platform_command('stop')
        node.destroy_node()
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        if procs:
            print("Stopping motor control node / receivers...")
            shutdown_subprocesses(procs)
            print("Done. (can0 left up -- bring it down manually if needed.)")


if __name__ == '__main__':
    main()
