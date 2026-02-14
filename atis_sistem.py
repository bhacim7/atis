import cv2
import numpy as np
import time
import sys
import math
import os

# Hardware Imports
try:
    import Jetson.GPIO as GPIO
    import board
    import busio
    from adafruit_servokit import ServoKit
except ImportError as e:
    print(f"Warning: Hardware libraries not found ({e}). Running in simulation mode or on non-Jetson device.")
    GPIO = None
    ServoKit = None
    board = None
    busio = None

# Camera/AI Imports
try:
    import pyzed.sl as sl
    from ultralytics import YOLO
except ImportError as e:
    print(f"Warning: Camera/AI libraries not found ({e}).")
    sl = None
    YOLO = None

# --- Configuration Constants ---

# GPIO Pins (BOARD Mode)
STEP_PIN = 33
DIR_PIN = 31
ENA_PIN = 37
RELAY_PIN = 12  # TODO: Configure this pin! currently placeholder.

# I2C / PCA9685
SERVO_CHANNELS = [1, 2, 3] # Servo 1, 2, 3
BLDC_CHANNEL = 4           # ESC for Brushless Motor

# Stepper Settings
STEPS_PER_REV = 1600  # 1/8 microstepping assumed from olcak.py
STEPS_PER_DEGREE = STEPS_PER_REV / 360.0
MIN_STEP_MOVE = 10    # Minimum packet of steps to overcome friction/cable tension
MAX_STEPS_PER_LOOP = 100 # Safety cap
STEP_DELAY = 0.0015   # Speed control

# PID Constants (from denemePro.py)
KP_YAW = 0.8
KI_YAW = 0.005
KD_YAW = 0.02
PID_LIMIT = 20.0 # Integral limit

# Image Settings
IMG_WIDTH = 1280
IMG_HEIGHT = 720
FOV_X = 110.0 # ZED 2i approximate horizontal FOV in degrees

# Operational Settings
AIM_TOLERANCE_PIXELS = 40 # Pixel error tolerance for aiming
WATER_SPRAY_DURATION = 10.0 # Seconds
SCAN_STEP_SIZE = 5 # Steps per loop iteration during scan

# Model Path (TensorRT Engine)
MODEL_PATH = "/home/yarkin/roboboatIDA/roboboat/weights/small640.engine"

class HardwareController:
    def __init__(self):
        self.mock_mode = (GPIO is None or ServoKit is None)

        if not self.mock_mode:
            # GPIO Setup
            GPIO.cleanup()
            GPIO.setmode(GPIO.BOARD)
            GPIO.setwarnings(False)
            GPIO.setup([STEP_PIN, DIR_PIN, ENA_PIN, RELAY_PIN], GPIO.OUT, initial=GPIO.LOW)
            GPIO.output(ENA_PIN, GPIO.LOW) # Enable Stepper
            # Relay acts as armed when HIGH? Prompt says "Relay Pin must be set to HIGH continuously"
            # UPDATE: User requested Relay LOW initially, HIGH during ID11 maneuver.
            GPIO.output(RELAY_PIN, GPIO.LOW)

            # I2C / ServoKit Setup
            try:
                # Bus 0 - Pin 27 & 28 on Jetson Orin Nano
                try:
                    i2c_bus0 = busio.I2C(board.SCL_1, board.SDA_1)
                except:
                    import adafruit_blinka.microcontroller.tegra.t194.pin as pin
                    i2c_bus0 = busio.I2C(pin.I2C0_SCL, pin.I2C0_SDA)

                self.kit = ServoKit(channels=16, i2c=i2c_bus0, address=0x41)

                # Setup Pulse Widths (from olcak.py)
                for i in range(1, 5): # 1, 2, 3, 4
                    self.kit.servo[i].set_pulse_width_range(1000, 2000)

                print("Hardware initialized.")
                self.arm_bldc()

            except Exception as e:
                print(f"I2C/ServoKit Error: {e}")
                self.kit = None

        self.ball_index = 0 # 0, 1, 2 corresponds to Servo 1, 2, 3

    def arm_bldc(self):
        """Arm the ESC and start spinning."""
        if self.mock_mode or not self.kit: return
        print("Arming BLDC...")
        self.kit.servo[BLDC_CHANNEL].angle = 0
        time.sleep(2) # Wait for beep-beep

        # Start Spinning (Continuous)
        # olcak.py uses 1850us pwm.
        # pwm_to_angle(1850) -> (1850-1000)*180/1000 = 0.85 * 180 = 153 degrees
        target_pwm = 1850
        angle = (target_pwm - 1000) * 180 / 1000
        self.kit.servo[BLDC_CHANNEL].angle = angle
        print(f"BLDC Spinning at {target_pwm}us ({angle} deg)")

    def rotate_stepper(self, steps, direction):
        """
        Rotate stepper motor.
        direction: 1 (CW/Right) or 0 (CCW/Left) - Assumption, verify wiring.
        """
        if self.mock_mode: return

        GPIO.output(DIR_PIN, direction)
        for _ in range(int(steps)):
            GPIO.output(STEP_PIN, 1)
            time.sleep(STEP_DELAY)
            GPIO.output(STEP_PIN, 0)
            time.sleep(STEP_DELAY)

    def drop_ball(self):
        """
        Trigger the next available servo to drop a ball.
        """
        if self.mock_mode or not self.kit:
            print("Mock: Drop Ball")
            self.ball_index += 1
            return

        if self.ball_index >= len(SERVO_CHANNELS):
            print("No more balls!")
            return

        servo_ch = SERVO_CHANNELS[self.ball_index]
        print(f"Firing Ball {self.ball_index + 1} using Servo {servo_ch}")

        # Rotate 90 degrees to drop
        self.kit.servo[servo_ch].angle = 90
        time.sleep(0.5)
        # Return to 0
        self.kit.servo[servo_ch].angle = 0
        time.sleep(0.5)

        self.ball_index += 1

    def set_relay(self, state):
        if self.mock_mode: return
        # Ensure state is boolean or 0/1
        gpio_state = GPIO.HIGH if state else GPIO.LOW
        GPIO.output(RELAY_PIN, gpio_state)

    def cleanup(self):
        if self.mock_mode: return
        # Stop BLDC
        if self.kit:
            self.kit.servo[BLDC_CHANNEL].angle = 0
            for i in SERVO_CHANNELS:
                self.kit.servo[i].angle = 0

        GPIO.output(ENA_PIN, GPIO.HIGH) # Disable motor
        GPIO.cleanup()
        print("Hardware cleaned up.")

class CameraHandler:
    def __init__(self):
        self.zed = None
        self.model = None

        if sl:
            print("Initializing ZED Camera...")
            self.zed = sl.Camera()
            init_params = sl.InitParameters()
            init_params.camera_resolution = sl.RESOLUTION.HD720
            init_params.camera_fps = 30
            init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
            init_params.coordinate_units = sl.UNIT.METER

            err = self.zed.open(init_params)
            if err != sl.ERROR_CODE.SUCCESS:
                print(f"ZED Open Failed: {err}")
                self.zed = None
            else:
                self.runtime_params = sl.RuntimeParameters()
                self.image_mat = sl.Mat()

        if YOLO:
            print(f"Loading YOLO model (TensorRT Engine)...")

            # Logic from IDA1.py: Use .engine file if available, else fallback
            if not os.path.exists(MODEL_PATH):
                print(f"[WARNING] {MODEL_PATH} not found. Using yolov11n.pt as fallback (simulation mode).")
                self.model = YOLO("yolov11n.pt")
            else:
                self.model = YOLO(MODEL_PATH)

            try:
                # Enable FP16 and CUDA for Jetson Orin Nano
                self.model.to('cuda').half()
                print("[INFO] Model loaded in FP16 mode (TensorRT optimized).")
            except Exception as e:
                print(f"Model GPU Error: {e}")

    def get_frame(self):
        if not self.zed: return None
        if self.zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(self.image_mat, sl.VIEW.LEFT)
            return self.image_mat.get_data() # BGRA
        return None

    def detect_objects(self, frame):
        """
        Returns a list of detections: {'id': class_id, 'bbox': (x, y, w, h), 'center': (cx, cy)}
        Priority IDs: 2 (Car/Ball?), 11 (Stop Sign/Water?) - Using standard COCO IDs for now as per plan.
        """
        if self.model is None or frame is None: return []

        # Convert BGRA to BGR for YOLO
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        results = self.model(frame_bgr, verbose=False)

        detections = []
        for result in results:
            for box in result.boxes:
                cls_id = int(box.cls[0])
                if cls_id in [2, 11]: # Target IDs
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)
                    w = int(x2 - x1)
                    h = int(y2 - y1)
                    detections.append({
                        'id': cls_id,
                        'bbox': (x1, y1, x2, y2),
                        'center': (cx, cy)
                    })
        return detections

class PIDController:
    def __init__(self):
        self.integral_yaw = 0.0
        self.last_error_yaw = 0.0

    def update(self, error_pixels, delta_time):
        """
        Calculate yaw correction in degrees.
        """
        # Convert pixel error to approximate degrees
        # FOV_X = 110 degrees, IMG_WIDTH = 1280
        degrees_per_pixel = FOV_X / IMG_WIDTH
        error_degrees = error_pixels * degrees_per_pixel

        if delta_time <= 0: return 0.0

        self.integral_yaw += error_degrees * delta_time
        self.integral_yaw = max(min(self.integral_yaw, PID_LIMIT), -PID_LIMIT)

        derivative_yaw = (error_degrees - self.last_error_yaw) / delta_time

        output_yaw = (KP_YAW * error_degrees +
                      KI_YAW * self.integral_yaw +
                      KD_YAW * derivative_yaw)

        self.last_error_yaw = error_degrees
        return output_yaw

class System:
    def __init__(self):
        self.hw = HardwareController()
        self.cam = CameraHandler()
        self.pid = PIDController()
        self.running = True
        self.state = "SCANNING"
        self.last_time = time.time()
        self.target_lost_timer = 0
        self.spray_start_time = 0 # Deprecated in favor of relay_start_time for ID11, used for compatibility? No, remove.
        self.relay_start_time = 0
        self.relay_active = False
        self.scan_direction = 1

    def run(self):
        print("System Started. Loop running...")
        try:
            while self.running:
                current_time = time.time()
                delta_time = current_time - self.last_time
                self.last_time = current_time

                frame = self.cam.get_frame()
                detections = self.cam.detect_objects(frame)

                # Sort detections by priority: ID 2 > ID 11
                # ID 2 (Car) -> Scenario A (Ball)
                # ID 11 (Stop Sign) -> Scenario B (Water)
                target = None

                # Target Selection logic
                if self.state == "ACTION_ID11" or (self.state == "TRACKING" and self.relay_active):
                    # In Spraying State or Tracking for Spray, lock onto ID 11 and ignore ID 2 to prevent swapping
                    targets_11 = [d for d in detections if d['id'] == 11]
                    if targets_11:
                        target = targets_11[0]
                    else:
                        target = None # Lost ID 11, do not swap to ID 2
                else:
                    # General Priority: ID 2 > ID 11
                    targets_2 = [d for d in detections if d['id'] == 2]
                    if targets_2:
                        target = targets_2[0] # Pick first
                    else:
                        targets_11 = [d for d in detections if d['id'] == 11]
                        if targets_11:
                            target = targets_11[0]

                # State Machine Logic
                if self.state == "SCANNING":
                    if target:
                        print(f"Target Found (ID {target['id']}). Switching to TRACKING.")
                        self.state = "TRACKING"

                        if target['id'] == 11:
                            print("ID 11 Detected. Starting Relay and Timer.")
                            self.hw.set_relay(True)
                            self.relay_active = True
                            self.relay_start_time = current_time

                    else:
                        # Scan behavior: Rotate stepper
                        self.hw.rotate_stepper(SCAN_STEP_SIZE, self.scan_direction)
                        # Optional: Reverse direction if limit switch or count reached?
                        # For now, continuous rotation as per prompt "Stepper Motor should rotate continuously".

                elif self.state == "TRACKING":
                    # Check Relay Timer (if active)
                    if self.relay_active:
                        if current_time - self.relay_start_time > WATER_SPRAY_DURATION:
                            print("Relay Timer Expired (10s). Relay LOW. Resuming Scan.")
                            self.hw.set_relay(False)
                            self.relay_active = False
                            self.state = "SCANNING"
                            # Reset PID errors
                            self.pid.integral_yaw = 0
                            self.pid.last_error_yaw = 0
                            continue

                    if not target:
                        print("Target Lost. Switching to SCANNING.")
                        self.state = "SCANNING"

                        # Ensure Relay is OFF if we lost tracking mid-maneuver
                        if self.relay_active:
                            print("Target Lost during maneuver. Relay LOW.")
                            self.hw.set_relay(False)
                            self.relay_active = False

                        # Reset PID errors
                        self.pid.integral_yaw = 0
                        self.pid.last_error_yaw = 0
                        continue

                    # PID Control
                    center_x = IMG_WIDTH // 2
                    error_pixels = target['center'][0] - center_x

                    # Check alignment FIRST
                    if abs(error_pixels) < AIM_TOLERANCE_PIXELS:
                        if target['id'] == 2:
                            print("Aligned with ID 2. Engaging Scenario A (Fire).")
                            self.state = "ACTION_ID2"
                        elif target['id'] == 11:
                            print("Aligned with ID 11. Engaging Scenario B (Water).")
                            # Relay is already HIGH from SCANNING -> TRACKING transition
                            self.state = "ACTION_ID11"
                        continue # Don't move if aligned

                    # Calculate correction
                    yaw_correction_deg = self.pid.update(error_pixels, delta_time)

                    # Convert degrees to steps
                    steps = abs(yaw_correction_deg * STEPS_PER_DEGREE)
                    direction = 1 if yaw_correction_deg > 0 else 0 # Assuming 1 is Right/CW

                    # Apply movement logic with Min/Max clamping
                    if steps < MIN_STEP_MOVE:
                        steps = MIN_STEP_MOVE
                    if steps > MAX_STEPS_PER_LOOP:
                        steps = MAX_STEPS_PER_LOOP

                    self.hw.rotate_stepper(steps, direction)

                elif self.state == "ACTION_ID2":
                    # Fire Ball
                    self.hw.drop_ball()
                    # Wait/Cooldown or Immediate Resume?
                    # "After the servo triggers ... resume the Default State"
                    print("Ball Fired. Resuming Scan.")
                    self.state = "SCANNING"
                    # Reset PID
                    self.pid.integral_yaw = 0

                elif self.state == "ACTION_ID11":
                    # Water Spraying
                    # "Relay Pin must be set to HIGH continuously" -> It is already HIGH.
                    # "Lock onto the target and hold the aim for approximately 10 seconds."

                    # Check Timer (Global timer for this maneuver)
                    if current_time - self.relay_start_time > WATER_SPRAY_DURATION:
                        print("Water Spray Complete (10s). Resuming Scan.")
                        self.hw.set_relay(False)
                        self.relay_active = False
                        self.state = "SCANNING"
                        continue

                    # We need to Keep Tracking (PID) while in this state
                    if not target:
                        print("Target lost during spray. Resuming Scan.")
                        self.hw.set_relay(False)
                        self.relay_active = False
                        self.state = "SCANNING"
                        continue

                    # PID Control (Keep Aiming)
                    center_x = IMG_WIDTH // 2
                    error_pixels = target['center'][0] - center_x

                    if abs(error_pixels) < AIM_TOLERANCE_PIXELS:
                        continue

                    yaw_correction_deg = self.pid.update(error_pixels, delta_time)
                    steps = abs(yaw_correction_deg * STEPS_PER_DEGREE)
                    direction = 1 if yaw_correction_deg > 0 else 0

                    if steps < MIN_STEP_MOVE:
                        steps = MIN_STEP_MOVE
                    if steps > MAX_STEPS_PER_LOOP:
                        steps = MAX_STEPS_PER_LOOP

                    self.hw.rotate_stepper(steps, direction)

        except KeyboardInterrupt:
            print("Stopping...")
        finally:
            self.hw.cleanup()

if __name__ == "__main__":
    sys_inst = System()
    sys_inst.run()
