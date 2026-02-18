import sys
import os
from unittest.mock import MagicMock

# Mock modules BEFORE importing atis_sistem
sys.modules["cv2"] = MagicMock()
sys.modules["numpy"] = MagicMock()
sys.modules["Jetson"] = MagicMock()
sys.modules["Jetson.GPIO"] = MagicMock()
sys.modules["board"] = MagicMock()
sys.modules["busio"] = MagicMock()
sys.modules["adafruit_servokit"] = MagicMock()
sys.modules["pyzed"] = MagicMock()
sys.modules["pyzed.sl"] = MagicMock()
sys.modules["ultralytics"] = MagicMock()

# Add parent directory to path to import atis_sistem
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

try:
    import atis_sistem
except ImportError as e:
    print(f"ImportError during test setup: {e}")
    sys.exit(1)

def test_hardware_controller():
    print("\n--- Testing HardwareController ---")
    try:
        hw = atis_sistem.HardwareController()
    except Exception as e:
        print(f"Failed to instantiate HardwareController: {e}")
        return

    # Check initialization
    if not hasattr(hw, 'current_steps'):
        print("FAIL: HardwareController missing 'current_steps'")
        return

    if hw.current_steps != 0:
        print(f"FAIL: current_steps initialized to {hw.current_steps}, expected 0")
    else:
        print("PASS: current_steps initialized to 0")

    # Test rotate_stepper
    print("Testing rotate_stepper logic...")
    hw.rotate_stepper(100, 1) # CW
    if hw.current_steps != 100:
        print(f"FAIL: current_steps is {hw.current_steps} after +100, expected 100")
    else:
        print("PASS: current_steps updated correctly (+100)")

    hw.rotate_stepper(50, 0) # CCW
    if hw.current_steps != 50:
        print(f"FAIL: current_steps is {hw.current_steps} after -50, expected 50")
    else:
        print("PASS: current_steps updated correctly (-50)")

def test_bldc_methods():
    print("\n--- Testing BLDC Methods ---")
    try:
        hw = atis_sistem.HardwareController()
    except:
        return

    if not hasattr(hw, 'activate_bldc'):
        print("FAIL: HardwareController missing 'activate_bldc'")
    else:
        print("PASS: activate_bldc exists")

    if not hasattr(hw, 'deactivate_bldc'):
        print("FAIL: HardwareController missing 'deactivate_bldc'")
    else:
        print("PASS: deactivate_bldc exists")

def test_system_constants():
    print("\n--- Testing System Constants ---")
    if not hasattr(atis_sistem, 'SWEEP_ANGLE'):
        print("FAIL: SWEEP_ANGLE constant missing")
    else:
        print(f"PASS: SWEEP_ANGLE is {atis_sistem.SWEEP_ANGLE}")

    if not hasattr(atis_sistem, 'SWEEP_STEPS'):
         print("FAIL: SWEEP_STEPS constant missing")
    else:
         print(f"PASS: SWEEP_STEPS is {atis_sistem.SWEEP_STEPS}")

if __name__ == "__main__":
    print("Running Logic Tests...")
    test_hardware_controller()
    test_bldc_methods()
    test_system_constants()
