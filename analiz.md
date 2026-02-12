# Code Analysis and System Report

## Code Analysis

The system `atis_sistem.py` was created by integrating logic from the provided reference files:

1.  **Hardware Control (`olcak.py`)**:
    *   The `HardwareController` class manages low-level hardware interactions.
    *   **Stepper Motor**: Implemented using `Jetson.GPIO` on pins 33 (STEP), 31 (DIR), and 37 (ENA), matching `olcak.py`.
    *   **Servos & BLDC**: Implemented using `adafruit_servokit` on I2C address 0x41. Channels 1, 2, 3 are used for ball dropping, and Channel 4 is used for the Brushless Motor (ESC), matching `olcak.py`.
    *   **Relay**: A placeholder pin (12) is assigned for the water spray relay, as the original file did not specify a pin number.

2.  **Camera & AI (`IDA1.py` & `kamera.py`)**:
    *   The `CameraHandler` class handles the ZED 2i camera and Object Detection.
    *   **ZED 2i**: Initialized using `pyzed.sl` with parameters extracted from `kamera.py` (HD720, 30FPS, PERFORMANCE depth mode).
    *   **YOLO (TensorRT)**: Utilizes the `ultralytics` library but strictly enforces the use of the **TensorRT Engine** format (`.engine`) as seen in `IDA1.py`.
        *   It looks for the engine file at `/home/yarkin/roboboatIDA/roboboat/weights/small640.engine`.
        *   If the engine file is found, it loads it and enables **CUDA** and **FP16** (Half Precision) optimization for the Jetson Orin Nano.
        *   A fallback to `yolov11n.pt` is included only for simulation/testing purposes if the engine file is missing.

3.  **PID Control (`denemePro.py`)**:
    *   The `PIDController` class implements the Yaw PID logic extracted from `denemePro.py`.
    *   It calculates the error in degrees based on pixel offset and applies P, I, and D terms to determine the required stepper movement.

## Scenario Analysis

The system operates in a continuous loop with the following states:

1.  **SCANNING (Default State)**:
    *   The Stepper Motor rotates continuously in small increments to scan the area.
    *   The BLDC motor spins continuously to be ready for firing.
    *   The Relay is set to HIGH (armed).

2.  **Target Detection**:
    *   If **Target ID 2** is detected (Priority High): The system switches to `TRACKING` mode. Once aligned (error < tolerance), it triggers `ACTION_ID2`.
    *   If **Target ID 11** is detected (Priority Low): The system switches to `TRACKING` mode. Once aligned, it triggers `ACTION_ID11`.
    *   **Target Locking**: During actions (especially water spraying), the system locks onto the specific target ID to prevent swapping to a higher priority target mid-action.

3.  **Action Scenarios**:
    *   **Scenario A (ID 2 - Ball Firing)**: The system fires one ball by rotating the next available servo (1, 2, or 3) to 90 degrees and back. It then resumes scanning.
    *   **Scenario B (ID 11 - Water Spraying)**: The system maintains aim (using PID) for 10 seconds to spray the target. After the timer expires, it resumes scanning.

## Adjustable Parameters

The following parameters in `atis_sistem.py` can be tuned for field performance:

### PID Control
*   `KP_YAW` (Default: 0.8): Proportional gain. Increase for faster response, decrease if oscillating.
*   `KI_YAW` (Default: 0.005): Integral gain. Fixes steady-state error.
*   `KD_YAW` (Default: 0.02): Derivative gain. Dampens overshoot.
*   `PID_LIMIT` (Default: 20.0): Limits the integral term accumulation.

### Hardware
*   `STEP_DELAY` (Default: 0.0004): Controls the speed of the Stepper Motor. Lower is faster.
*   `STEPS_PER_REV` (Default: 1600): Steps per full revolution (depends on microstepping settings).
*   `RELAY_PIN`: **MUST BE CONFIGURED**. Currently set to 12.

### Operation
*   `AIM_TOLERANCE_PIXELS` (Default: 20): How close to the center the target must be to trigger an action.
*   `WATER_SPRAY_DURATION` (Default: 10.0): Duration in seconds to hold aim for Scenario B.
*   `SCAN_STEP_SIZE` (Default: 5): How many steps to move per loop iteration during scanning.
*   `MODEL_PATH`: Path to the TensorRT `.engine` file.
