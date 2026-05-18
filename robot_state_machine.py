"""
robot_state_machine.py
CareGo State Machine
Assisted Living Facility Package Delivery
Hardware:
    Brain:      Jetson Orin Nano
    Drive:      Mecanum wheels via drive STM32 over serial
    Lift:       Ball lift motor via lift STM32 over serial
    Sensors:    Ultrasonic sensors (obstacle detection)
    Vision:     Limelight 4 (AprilTag localization via NetworkTables)
    Navigation: SLAM (ROS2 Nav2 / RTAB-Map)

File dependencies (place in same package):
    drive/motor_control.py   — DriveMotors class
    drive/mecanum.py         — inverse kinematics
    lift/lift_control.py     — LiftMotor class
    sensors/ultrasonic.py    — UltrasonicSensors class
    navigation/slam.py       — SLAMNavigator class
    navigation/limelight.py  — Limelight4 class
"""

from enum import Enum, auto
import time

from drive.motor_control import DriveMotors
from lift.lift_control import LiftMotor

# ---------------------------------------------------------------------------
# States
# ---------------------------------------------------------------------------

class State(Enum):
    IDLE              = auto()   # Waiting at base station
    LOCALIZING        = auto()   # SLAM init + AprilTag scan to confirm pose
    NAVIGATING        = auto()   # Following SLAM path toward destination
    OBSTACLE_DETECTED = auto()   # Stopped; waiting for path to clear or rerouting
    ARRIVED           = auto()   # At destination; opening lift, awaiting pickup
    RETURNING         = auto()   # Navigating back to base
    ERROR             = auto()   # Fault detected; alerting staff


# ---------------------------------------------------------------------------
# Obstacle detection zones
# Three zones give residents time to react before the robot stops.
# ---------------------------------------------------------------------------

ZONE_CAUTION_CM   = 150   # Slow to 20% speed
ZONE_STOP_CM      =  50   # Full stop, wait for clear
ZONE_EMERGENCY_CM =  20   # Hard stop + strafe away


# ---------------------------------------------------------------------------
# Stub classes for hardware not yet wired to real drivers
# Replace each with the real implementation as hardware arrives.
# ---------------------------------------------------------------------------

class UltrasonicSensors:
    def get_distance_front(self) -> float:
        return 200.0   # stub: always clear

    def get_distance_left(self) -> float:
        return 200.0

    def get_distance_right(self) -> float:
        return 200.0


class StaffAlert:
    def send(self, message: str):
        print(f"[ALERT] {message}")
        # TODO: wire to buzzer, display, or network push notification


class SLAMNavigator:
    """
    Stub for ROS2 Nav2 / RTAB-Map.
    In real code this will publish to /goal_pose and subscribe to /odom.
    """
    def initialize(self):
        print("[SLAM] Initializing map and pose estimation")

    def set_goal(self, destination: str):
        print(f"[SLAM] Goal set: {destination}")

    def set_base_goal(self):
        print("[SLAM] Goal set: base station")

    def is_goal_reached(self) -> bool:
        return False   # stub

    def compute_reroute(self) -> bool:
        print("[SLAM] Computing alternate route")
        return False   # stub — True if alternate route found


class Limelight4:
    """
    Stub for Limelight 4 AprilTag pipeline.
    Real implementation reads from NetworkTables:
        'tid'     -> detected tag ID
        'botpose' -> robot pose in field space
    See: https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api
    """
    def get_detected_tag_id(self) -> int | None:
        return None   # stub

    def get_pose_estimate(self) -> dict | None:
        return None   # stub: { 'x', 'y', 'z', 'yaw' }

    def pose_confirmed(self) -> bool:
        return True   # stub — replace with real confidence threshold


# ---------------------------------------------------------------------------
# Robot
# ---------------------------------------------------------------------------

class DeliveryRobot:

    OBSTACLE_TIMEOUT_SECONDS = 30.0   # Reroute or escalate to ERROR after this
    PICKUP_WAIT_SECONDS      = 60.0   # How long to wait for resident to collect

    def __init__(self):
        self.state       = State.IDLE
        self.destination = None

        # Hardware — DriveMotors and LiftMotor connect to STM32s over serial
        self.drive     = DriveMotors()
        self.lift      = LiftMotor()
        self.sensors   = UltrasonicSensors()
        self.alert     = StaffAlert()
        self.slam      = SLAMNavigator()
        self.limelight = Limelight4()

        # Internal timers
        self._obstacle_start_time = None
        self._pickup_start_time   = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def assign_delivery(self, destination: str):
        """Call this when a new delivery job is dispatched."""
        if self.state != State.IDLE:
            print(f"[ROBOT] Cannot assign — current state: {self.state.name}")
            return
        print(f"[ROBOT] Delivery assigned to: {destination}")
        self.destination = destination
        self._transition(State.LOCALIZING)

    def resolve_error(self):
        """Call this when staff have physically resolved the error condition."""
        if self.state == State.ERROR:
            print("[ROBOT] Error resolved by staff — returning to IDLE")
            self._transition(State.IDLE)

    def update(self):
        """
        Main loop tick. Call this repeatedly:
          - Simple loop:   while True: robot.update(); time.sleep(0.05)
          - ROS2 timer:    self.create_timer(0.05, robot.update)
        """
        handlers = {
            State.IDLE:              self._handle_idle,
            State.LOCALIZING:        self._handle_localizing,
            State.NAVIGATING:        self._handle_navigating,
            State.OBSTACLE_DETECTED: self._handle_obstacle,
            State.ARRIVED:           self._handle_arrived,
            State.RETURNING:         self._handle_returning,
            State.ERROR:             self._handle_error,
        }
        handler = handlers.get(self.state)
        if handler:
            handler()

    # ------------------------------------------------------------------
    # State handlers
    # ------------------------------------------------------------------

    def _handle_idle(self):
        pass   # Waiting for assign_delivery()

    def _handle_localizing(self):
        self.slam.initialize()
        self.limelight.get_detected_tag_id()

        if self.limelight.pose_confirmed():
            print("[ROBOT] Pose confirmed via AprilTag")
            self.slam.set_goal(self.destination)
            self._transition(State.NAVIGATING)
        else:
            print("[ROBOT] Waiting for AprilTag lock...")
            # TODO: slowly rotate to scan for tags
            #       self.drive.move(vx=0.0, vy=0.0, omega=0.2)
            # TODO: add a timeout to transition to ERROR if no tag found

    def _handle_navigating(self):
        front = self.sensors.get_distance_front()
        left  = self.sensors.get_distance_left()
        right = self.sensors.get_distance_right()

        # --- Emergency zone: strafe away from the closest side ---
        if front < ZONE_EMERGENCY_CM:
            print("[ROBOT] Emergency zone — strafing clear")
            if left > right:
                self.drive.move(vx=0.0, vy=-0.3, omega=0.0)   # strafe left
            else:
                self.drive.move(vx=0.0, vy=0.3, omega=0.0)    # strafe right
            self._obstacle_start_time = time.time()
            self._transition(State.OBSTACLE_DETECTED)
            return

        # --- Stop zone: full stop, wait ---
        if front < ZONE_STOP_CM:
            print("[ROBOT] Stop zone — waiting for clearance")
            self.drive.move(vx=0.0, vy=0.0, omega=0.0)
            self._obstacle_start_time = time.time()
            self._transition(State.OBSTACLE_DETECTED)
            return

        # --- Caution zone: slow down ---
        if front < ZONE_CAUTION_CM:
            speed = 0.2
        else:
            speed = 0.5

        # Gentle side bias to hug center of hallway
        vy_bias = 0.0
        if left < 60:
            vy_bias = 0.1    # nudge right if too close to left wall
        elif right < 60:
            vy_bias = -0.1   # nudge left if too close to right wall

        self.drive.move(vx=speed, vy=vy_bias, omega=0.0)

        if self.slam.is_goal_reached():
            self.drive.move(vx=0.0, vy=0.0, omega=0.0)
            self._transition(State.ARRIVED)

    def _handle_obstacle(self):
        front = self.sensors.get_distance_front()

        # Path cleared — resume
        if front >= ZONE_STOP_CM:
            print("[ROBOT] Path clear — resuming navigation")
            self._obstacle_start_time = None
            self._transition(State.NAVIGATING)
            return

        # Check how long we have been blocked
        if self._obstacle_start_time:
            blocked_for = time.time() - self._obstacle_start_time

            if blocked_for > self.OBSTACLE_TIMEOUT_SECONDS:
                print(f"[ROBOT] Blocked for {blocked_for:.0f}s — attempting reroute")
                found_route = self.slam.compute_reroute()

                if found_route:
                    self._obstacle_start_time = None
                    self._transition(State.NAVIGATING)
                else:
                    self.alert.send("Robot blocked and cannot reroute. Please assist.")
                    self._transition(State.ERROR)

    def _handle_arrived(self):
        # Open lift on first tick of this state
        if self._pickup_start_time is None:
            self.lift.open()
            print(f"[ROBOT] Arrived at {self.destination}. Waiting for pickup...")
            self._pickup_start_time = time.time()
            return

        # Wait for resident to collect package
        waited = time.time() - self._pickup_start_time
        if waited >= self.PICKUP_WAIT_SECONDS:
            print("[ROBOT] Pickup window elapsed — closing lift and returning")
            self.lift.close()
            self._pickup_start_time = None
            self.slam.set_base_goal()
            self._transition(State.RETURNING)

    def _handle_returning(self):
        front = self.sensors.get_distance_front()
        left  = self.sensors.get_distance_left()
        right = self.sensors.get_distance_right()

        # Same three-zone logic as NAVIGATING
        if front < ZONE_EMERGENCY_CM:
            if left > right:
                self.drive.move(vx=0.0, vy=-0.3, omega=0.0)
            else:
                self.drive.move(vx=0.0, vy=0.3, omega=0.0)
            self._obstacle_start_time = time.time()
            self._transition(State.OBSTACLE_DETECTED)
            return

        if front < ZONE_STOP_CM:
            self.drive.move(vx=0.0, vy=0.0, omega=0.0)
            self._obstacle_start_time = time.time()
            self._transition(State.OBSTACLE_DETECTED)
            return

        speed = 0.2 if front < ZONE_CAUTION_CM else 0.5

        vy_bias = 0.0
        if left < 60:
            vy_bias = 0.1
        elif right < 60:
            vy_bias = -0.1

        self.drive.move(vx=speed, vy=vy_bias, omega=0.0)

        if self.slam.is_goal_reached():
            self.drive.move(vx=0.0, vy=0.0, omega=0.0)
            print("[ROBOT] Returned to base station")
            self._transition(State.IDLE)

    def _handle_error(self):
        self.drive.move(vx=0.0, vy=0.0, omega=0.0)
        self.lift.close()
        print("[ROBOT] ERROR state — awaiting staff intervention")

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _transition(self, new_state: State):
        print(f"[STATE] {self.state.name} -> {new_state.name}")
        self.state = new_state

    def shutdown(self):
        """Clean shutdown — stop motors and close serial connections."""
        self.drive.move(vx=0.0, vy=0.0, omega=0.0)
        self.lift.close()
        self.drive.close()
        self.lift.close_connection()
        print("[ROBOT] Shutdown complete")


# ---------------------------------------------------------------------------
# Entry point — simple demo loop
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    robot = DeliveryRobot()

    try:
        robot.assign_delivery(destination="Room 214")

        for tick in range(10):
            print(f"\n--- Tick {tick + 1} | State: {robot.state.name} ---")
            robot.update()
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n[ROBOT] Interrupted by user")

    finally:
        robot.shutdown()
