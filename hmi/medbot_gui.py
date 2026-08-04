# medbot_gui.py
# ELARA CareGo ATR — Touchscreen HMI for Raspberry Pi Touch Display
# Runs on Raspberry Pi 4 with the official 7" DSI touch display
# Handles voice input, room confirmation, platform controls,
# and sends confirmed room number to Jetson via TCP socket

import sounddevice as sd
from scipy.io.wavfile import write
from faster_whisper import WhisperModel
import subprocess
import socket
import serial
import re
import time
import tkinter as tk
import threading

# --- Hardware Config ---

# Find ReSpeaker mic by name rather than hardcoded index
# Device index can change at boot so name lookup is more reliable
def find_mic():
    devices = sd.query_devices()
    for i, d in enumerate(devices):
        if 'ReSpeaker' in d['name']:
            return i
    return None

MIC_DEVICE = find_mic()
SAMPLE_RATE = 16000     # 16kHz required by Whisper
SECONDS = 10            # Recording duration increased to 10 seconds
PIPER_MODEL = "/home/pi/piper/models/en_US-lessac-medium.onnx"
JETSON_IP = '192.168.137.2'
JETSON_PORT = 5005      # room_receiver.py
PLATFORM_PORT = 5007    # platform_receiver.py
ACTUATOR_PORT = 5006    # actuator_receiver.py
RS485_PORT = '/dev/ttyUSB0'
RS485_BAUD = 38400

model = WhisperModel("tiny.en", device="cpu", compute_type="int8")


# --- Audio Functions ---

def record_and_transcribe(filename="command.wav"):
    """Records audio and transcribes using Whisper. Returns text string."""
    if MIC_DEVICE is None:
        print("  [!] Microphone not found")
        return ""
    audio = sd.rec(
        int(SECONDS * SAMPLE_RATE),
        samplerate=SAMPLE_RATE,
        channels=1,
        dtype="int16",
        device=MIC_DEVICE
    )
    sd.wait()
    write(filename, SAMPLE_RATE, audio)
    segments, info = model.transcribe(filename)
    return " ".join(segment.text.strip() for segment in segments)


def speak(message):
    """Converts text to speech using Piper TTS and plays through speaker."""
    subprocess.run(
        f'echo "{message}" | /home/pi/piper/piper --model {PIPER_MODEL} --output_file response.wav',
        shell=True
    )
    subprocess.run("aplay -D plughw:3,0 response.wav", shell=True)


# --- Room Number Parsing ---

def parse_room(text):
    """
    Extracts a 3-digit room number from transcribed speech.
    Handles: "302", "three oh two", "three hundred and two", "300 and 2"
    Returns room number string or None.
    """
    text = text.lower().replace("-", " ").replace(",", "").replace(".", "")

    # Case 1: "300 and 2" -> 302
    and_match = re.search(r'\b(\d{3})\s+and\s+(\d)\b', text)
    if and_match:
        return str(int(and_match.group(1)) + int(and_match.group(2)))

    # Case 2: bare 3-digit number
    match = re.search(r'\b(\d{3})\b', text)
    if match:
        return match.group(1)

    # Case 3: individual spoken digits — "oh"/"o" treated as zero
    single_digits = {
        "zero": "0", "oh": "0", "o": "0",
        "one": "1", "two": "2", "three": "3", "four": "4",
        "five": "5", "six": "6", "seven": "7", "eight": "8", "nine": "9"
    }
    words = text.split()
    digits = []
    for word in words:
        if word in single_digits:
            digits.append(single_digits[word])
        elif word.isdigit() and len(word) == 1:
            digits.append(word)
    if len(digits) == 3:
        return "".join(digits)

    # Case 4: word numbers e.g. "three hundred and two"
    hundreds = {
        "one hundred": 100, "two hundred": 200, "three hundred": 300,
        "four hundred": 400, "five hundred": 500, "six hundred": 600,
        "seven hundred": 700, "eight hundred": 800, "nine hundred": 900
    }
    ones = {
        "one": 1, "two": 2, "three": 3, "four": 4, "five": 5,
        "six": 6, "seven": 7, "eight": 8, "nine": 9, "ten": 10,
        "eleven": 11, "twelve": 12, "thirteen": 13, "fourteen": 14,
        "fifteen": 15, "sixteen": 16, "seventeen": 17, "eighteen": 18,
        "nineteen": 19, "twenty": 20, "thirty": 30, "forty": 40,
        "fifty": 50, "sixty": 60, "seventy": 70, "eighty": 80, "ninety": 90
    }
    base = None
    for phrase, val in hundreds.items():
        if phrase in text:
            base = val
            text = text.replace(phrase, "").replace("and", "").strip()
            break
    if base is not None:
        remainder = 0
        for word, val in sorted(ones.items(), key=lambda x: -x[1]):
            if word in text:
                remainder += val
                text = text.replace(word, "", 1).strip()
        return str(base + remainder)

    return None


# --- Jetson Communication ---

def send_to_jetson(room):
    """Sends confirmed room number to Jetson port 5005 via TCP socket."""
    try:
        s = socket.socket()
        s.connect((JETSON_IP, JETSON_PORT))
        s.send(f'room_{room}'.encode())
        s.close()
        print(f"  Sent room_{room} to Jetson")
    except Exception as e:
        print(f"  [!] Failed to send to Jetson: {e}")


# --- Button Style Helper ---

def make_button(parent, text, bg, command=None, width=14, height=2, font_size=24):
    """
    Creates a styled button with raised relief and padding for a rounded look.
    bd=4 and relief='raised' give a 3D appearance on the touch display.
    """
    return tk.Button(
        parent,
        text=text,
        font=("Helvetica", font_size),
        bg=bg,
        fg="white",
        activebackground=bg,
        activeforeground="white",
        width=width,
        height=height,
        relief="raised",
        bd=4,
        padx=10,
        pady=5,
        cursor="hand2",
        command=command
    )


# --- GUI Application ---

class ElaraApp:
    """
    Main Tkinter GUI for ELARA HMI.

    Screen flow:
      Sleep -> Menu -> Package Placement? -> Listening -> Thinking ->
      Understood / Not Understood -> Confirmation -> Confirmed ->
      Dropoff Confirmation -> Returning to Hub -> Sleep

      Menu -> Platform Controls -> Back -> Menu
    """

    def __init__(self, root):
        self.root = root
        self.root.title("ELARA")
        self.root.attributes('-fullscreen', True)
        self.root.configure(bg='black')
        self.root.bind('<Escape>', lambda e: self.root.attributes('-fullscreen', False))
        self.frame = None
        self.confirmed = None
        self._sleep_timer = None
        self.show_sleep_screen()

    def reset_sleep_timer(self):
        """
        Resets 3-minute inactivity sleep timer.
        Called on menu/button interactions.
        NOT called on dropoff screen — robot may take time to arrive.
        """
        if self._sleep_timer:
            self.root.after_cancel(self._sleep_timer)
        self._sleep_timer = self.root.after(180000, self.show_sleep_screen)

    def clear(self):
        """Destroys current frame and creates a fresh blank one."""
        if self.frame:
            self.frame.destroy()
        self.frame = tk.Frame(self.root, bg='black')
        self.frame.pack(expand=True, fill='both')

    # -------------------------------------------------------------------------
    # Screens
    # -------------------------------------------------------------------------

    def show_sleep_screen(self):
        """
        Sleep/idle screen — shown at startup and after inactivity.
        Tap anywhere to wake. Cancels sleep timer since already asleep.
        """
        if self._sleep_timer:
            self.root.after_cancel(self._sleep_timer)
            self._sleep_timer = None
        self.clear()
        lbl = tk.Label(self.frame, text="Touch to Start",
                       font=("Helvetica", 48), fg="white", bg="black")
        lbl.pack(expand=True)
        self.frame.bind("<Button-1>", lambda e: self.show_menu_screen())
        lbl.bind("<Button-1>", lambda e: self.show_menu_screen())

    def show_menu_screen(self):
        """Main menu — Send Delivery or Platform Controls. Resets sleep timer."""
        self.reset_sleep_timer()
        self.clear()
        tk.Label(self.frame, text="Select Mode",
                 font=("Helvetica", 40), fg="white", bg="black").pack(pady=50)
        btn_frame = tk.Frame(self.frame, bg='black')
        btn_frame.pack(pady=20)
        make_button(btn_frame, "Send\nDelivery", "#1a73e8",
                    command=self.show_package_placement_screen,
                    width=10, height=3).pack(side='left', padx=30)
        make_button(btn_frame, "Platform\nControls", "#e8871a",
                    command=self.show_platform_screen,
                    width=10, height=3).pack(side='left', padx=30)

    def show_package_placement_screen(self):
        """
        Package placement confirmation — after Send Delivery is pressed.
        YES starts listening session. NO returns to menu.
        """
        self.reset_sleep_timer()
        self.clear()
        tk.Label(self.frame, text="Have you placed your",
                 font=("Helvetica", 32), fg="white", bg="black").pack(pady=20)
        tk.Label(self.frame, text="package on the platform?",
                 font=("Helvetica", 32), fg="white", bg="black").pack()
        btn_frame = tk.Frame(self.frame, bg='black')
        btn_frame.pack(pady=40)
        make_button(btn_frame, "YES", "green",
                    command=self.start_session,
                    width=8, height=2).pack(side='left', padx=30)
        make_button(btn_frame, "NO", "#cc0000",
                    command=self.show_menu_screen,
                    width=8, height=2).pack(side='left', padx=30)

    def show_listening_screen(self):
        """Listening screen — shown while mic records for 10 seconds."""
        self.clear()
        tk.Label(self.frame, text="Where would you like",
                 font=("Helvetica", 32), fg="white", bg="black").pack(pady=40)
        tk.Label(self.frame, text="to send the package to?",
                 font=("Helvetica", 32), fg="white", bg="black").pack()
        tk.Label(self.frame, text="Listening...",
                 font=("Helvetica", 28), fg="#aaaaaa", bg="black").pack(pady=30)

    def show_thinking_screen(self):
        """
        Thinking screen — shown while Whisper processes audio.
        TODO: add loading animation in a future update.
        """
        self.clear()
        tk.Label(self.frame, text="ELARA Thinking...",
                 font=("Helvetica", 48), fg="white", bg="black").pack(expand=True)

    def show_understood_screen(self, raw, room):
        """
        Understood screen — valid room detected.
        Shows what was heard and parsed room. TTS speaks confirmation.
        """
        self.clear()
        tk.Label(self.frame, text="I heard:",
                 font=("Helvetica", 28), fg="#aaaaaa", bg="black").pack(pady=20)
        tk.Label(self.frame, text=f'"{raw}"',
                 font=("Helvetica", 26), fg="white", bg="black").pack()
        tk.Label(self.frame, text=f"Room {room}",
                 font=("Helvetica", 42), fg="green", bg="black").pack(pady=20)

    def show_not_understood_screen(self, raw):
        """
        Not understood screen — no room number detected.
        Shows what was heard and error message. TTS speaks retry prompt.
        """
        self.clear()
        tk.Label(self.frame, text="I heard:",
                 font=("Helvetica", 28), fg="#aaaaaa", bg="black").pack(pady=20)
        tk.Label(self.frame, text=f'"{raw}"' if raw else "(nothing)",
                 font=("Helvetica", 26), fg="white", bg="black").pack()
        tk.Label(self.frame, text="Could not detect a room number.",
                 font=("Helvetica", 26), fg="#cc0000", bg="black").pack(pady=15)
        tk.Label(self.frame, text="Please try again.",
                 font=("Helvetica", 24), fg="#aaaaaa", bg="black").pack()

    def show_confirmation_screen(self, room):
        """
        Confirmation screen — staff taps YES or NO.
        Updates self.confirmed which the session thread watches.
        """
        self.clear()
        tk.Label(self.frame, text=f"Deliver to Room {room}?",
                 font=("Helvetica", 38), fg="white", bg="black").pack(pady=60)
        btn_frame = tk.Frame(self.frame, bg='black')
        btn_frame.pack()
        make_button(btn_frame, "YES", "green",
                    command=lambda: self.on_yes(room),
                    width=8, height=2).pack(side='left', padx=40)
        make_button(btn_frame, "NO", "#cc0000",
                    command=self.on_no,
                    width=8, height=2).pack(side='left', padx=40)

    def show_confirmed_screen(self, room):
        """Confirmed screen — shown for 3 seconds after YES, then dropoff screen."""
        self.clear()
        tk.Label(self.frame, text="Confirmed!",
                 font=("Helvetica", 48), fg="green", bg="black").pack(pady=50)
        tk.Label(self.frame, text="Going to",
                 font=("Helvetica", 34), fg="white", bg="black").pack()
        tk.Label(self.frame, text=f"Room {room}",
                 font=("Helvetica", 48), fg="white", bg="black").pack(pady=20)

    def show_dropoff_confirmation_screen(self):
        """
        Dropoff confirmation screen — shown while robot is at destination.
        Staff presses large button to confirm package collected.
        Sleep timer NOT active — robot may take time to arrive.
        TODO: button press will trigger Nav2 return-to-hub when SLAM integrated.
        """
        self.clear()
        tk.Label(self.frame, text="Press button to confirm",
                 font=("Helvetica", 30), fg="white", bg="black").pack(pady=30)
        tk.Label(self.frame, text="package delivery",
                 font=("Helvetica", 30), fg="white", bg="black").pack()
        make_button(self.frame, "Package\nDelivered", "#1a73e8",
                    command=self.show_returning_screen,
                    width=16, height=4, font_size=30).pack(pady=40)

    def show_returning_screen(self):
        """
        Returning to hub screen — shown after dropoff confirmed.
        Displays for 3 seconds then goes to sleep screen.
        TODO: will trigger Nav2 return-to-hub navigation when SLAM integrated.
        """
        self.clear()
        tk.Label(self.frame, text="Returning to Hub...",
                 font=("Helvetica", 44), fg="white", bg="black").pack(expand=True)
        self.root.after(3000, self.show_sleep_screen)

    def show_platform_screen(self):
        """
        Platform controls — lift raise/lower (hold), tilt level/tilt (single press),
        actuator trigger (single press). Resets sleep timer.
        """
        self.reset_sleep_timer()
        self.clear()
        tk.Label(self.frame, text="Platform Controls",
                 font=("Helvetica", 28), fg="white", bg="black").pack(pady=10)
        btn_frame = tk.Frame(self.frame, bg='black')
        btn_frame.pack(pady=5)

        # Lift row — press and hold
        lift_row = tk.Frame(btn_frame, bg='black')
        lift_row.pack(pady=4)
        raise_btn = make_button(lift_row, "▲ Raise", "green",
                                width=9, height=2, font_size=22)
        raise_btn.pack(side='left', padx=8)
        raise_btn.bind("<ButtonPress-1>", lambda e: self.platform_move("raise"))
        raise_btn.bind("<ButtonRelease-1>", lambda e: self.platform_move("stop"))
        lower_btn = make_button(lift_row, "▼ Lower", "#cc0000",
                                width=9, height=2, font_size=22)
        lower_btn.pack(side='left', padx=8)
        lower_btn.bind("<ButtonPress-1>", lambda e: self.platform_move("lower"))
        lower_btn.bind("<ButtonRelease-1>", lambda e: self.platform_move("stop"))

        # Tilt row — single press, fixed step count
        tilt_row = tk.Frame(btn_frame, bg='black')
        tilt_row.pack(pady=4)
        make_button(tilt_row, "Level", "#1a73e8",
                    command=lambda: self.platform_move("level"),
                    width=9, height=2, font_size=22).pack(side='left', padx=8)
        make_button(tilt_row, "Tilt", "#e8871a",
                    command=lambda: self.platform_move("tilt"),
                    width=9, height=2, font_size=22).pack(side='left', padx=8)

        # Actuator — extend and retract buttons
        actuator_status_var = tk.StringVar(value="Actuator: Unknown")

        # Actuator — extend and retract buttons
        self.actuator_status_var = tk.StringVar(value="Actuator: Unknown")
        actuator_row = tk.Frame(btn_frame, bg='black')
        actuator_row.pack(pady=4)
        make_button(actuator_row, "Extend", "#1a73e8",
                    command=self.extend_actuator,
                    width=9, height=2, font_size=22).pack(side='left', padx=8)
        make_button(actuator_row, "Retract", "#cc0000",
                    command=self.retract_actuator,
                    width=9, height=2, font_size=22).pack(side='left', padx=8)
        tk.Label(btn_frame, textvariable=self.actuator_status_var,
                 font=("Helvetica", 16), fg="white", bg="black").pack(pady=4)

        # Back button
        make_button(btn_frame, "← Back", "#555555",
                    command=self.show_menu_screen,
                    width=20, height=2, font_size=20).pack(pady=4)

    # -------------------------------------------------------------------------
    # Platform Motor Control
    # -------------------------------------------------------------------------

    def platform_move(self, command):
        """
        Sends platform command to Jetson port 5007.
        Commands: raise, lower, stop, level, tilt
        Jetson's platform_receiver.py handles RS485 to stepper motors.
        """
        try:
            s = socket.socket()
            s.settimeout(1)
            s.connect((JETSON_IP, PLATFORM_PORT))
            s.send(command.encode())
            s.close()
            print(f"  Platform: {command}")
        except Exception as e:
            print(f"  [!] Platform command error: {e}")

    def extend_actuator(self):
        """Triggers actuator extend in background thread to prevent GUI freeze."""
        threading.Thread(target=self._actuator_extend_thread, daemon=True).start()

    def retract_actuator(self):
        """Triggers actuator retract in background thread to prevent GUI freeze."""
        threading.Thread(target=self._actuator_retract_thread, daemon=True).start()

    def _actuator_extend_thread(self):
        """Sends EXTEND command to Jetson port 5006."""
        try:
            s = socket.socket()
            s.settimeout(10)
            s.connect((JETSON_IP, ACTUATOR_PORT))
            s.send(b"EXTEND")
            s.close()
            print("  Actuator: extend sent")
            self.actuator_status_var.set("Actuator: Extended")
        except Exception as e:
            print(f"  [!] Actuator error: {e}")

    def _actuator_retract_thread(self):
        """Sends RETRACT command to Jetson port 5006."""
        try:
            s = socket.socket()
            s.settimeout(10)
            s.connect((JETSON_IP, ACTUATOR_PORT))
            s.send(b"RETRACT")
            s.close()
            print("  Actuator: retract sent")
            self.actuator_status_var.set("Actuator: Retracted")
        except Exception as e:
            print(f"  [!] Actuator error: {e}")

    # -------------------------------------------------------------------------
    # Delivery Session
    # -------------------------------------------------------------------------

    def start_session(self):
        """Starts delivery session in background thread."""
        threading.Thread(target=self.run_session, daemon=True).start()

    def run_session(self):
        """
        Main delivery loop — runs in background thread.

        1. Listen (10 seconds)
        2. Think (Whisper transcription)
        3a. Understood: show heard text + room, speak confirmation
        3b. Not understood: show error, speak retry, loop
        4. Confirmation screen — wait for YES/NO
        5a. YES: confirmed screen, send to Jetson, dropoff screen
        5b. NO: speak retry, loop
        """
        while True:
            # Step 1: Listen
            self.root.after(0, self.show_listening_screen)
            raw = record_and_transcribe("command.wav")

            # Step 2: Think
            self.root.after(0, self.show_thinking_screen)
            room = parse_room(raw)
            print(f"  Heard: \"{raw}\"")

            if not room:
                # Step 3b: Not understood
                print("  [!] No room number detected.")
                self.root.after(0, lambda r=raw: self.show_not_understood_screen(r))
                speak("I didn't catch a room number. Please try again.")
                time.sleep(2)
                continue

            # Step 3a: Understood
            self.root.after(0, lambda r=raw, rm=room: self.show_understood_screen(r, rm))
            speak(f"Deliver to room {room}. Is that correct?")

            # Step 4: Confirmation
            self.confirmed = None
            self.root.after(0, lambda rm=room: self.show_confirmation_screen(rm))
            while self.confirmed is None:
                time.sleep(0.1)

            if self.confirmed:
                # Step 5a: Confirmed
                self.root.after(0, lambda rm=room: self.show_confirmed_screen(rm))
                speak(f"Confirmed. Delivering to room {room}.")
                send_to_jetson(room)
                time.sleep(3)
                self.root.after(0, self.show_dropoff_confirmation_screen)
                break
            else:
                # Step 5b: Rejected
                speak("Okay. Please repeat the room number.")
                time.sleep(1)
                continue

    # -------------------------------------------------------------------------
    # Button Callbacks
    # -------------------------------------------------------------------------

    def on_yes(self, room):
        """YES pressed on confirmation screen."""
        self.reset_sleep_timer()
        self.confirmed = True

    def on_no(self):
        """NO pressed on confirmation screen."""
        self.reset_sleep_timer()
        self.confirmed = False


# --- Entry Point ---
# export DISPLAY=:0
# source ~/stt_env/bin/activate
# python3 ~/medbot_gui.py

root = tk.Tk()
app = ElaraApp(root)
root.mainloop()
