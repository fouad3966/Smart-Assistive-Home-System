"""
test_gripper.py - Standalone test for the gripper mimicking the Arduino test.
Connects to arm_servo_listener.py on the Pi.
"""
import time
from pickup_sequence import ArmController

def test_gripper():
    print("=== Gripper Test (via ZMQ) ===")
    print("70° = CLOSED  |  0° = OPEN")
    
    # Initialize the arm controller
    arm = ArmController()
    
    # Lock base, shoulder, elbow to safe positions
    safe_base = 90
    safe_shoulder = 140
    safe_elbow = 100

    print("Starting CLOSED...")
    arm.send_manual(safe_base, safe_shoulder, safe_elbow, gripper=70)
    time.sleep(1.0)
    
    print("\nHolding CLOSED (70°)...")
    arm.send_manual(safe_base, safe_shoulder, safe_elbow, gripper=70)
    time.sleep(1.0)
    
    print("\nOpening gripper...")
    print("  -> OPEN (0°)")
    arm.send_manual(safe_base, safe_shoulder, safe_elbow, gripper=0)
    time.sleep(1.0)
    
    print("\nClosing gripper...")
    print("  -> CLOSED (70°)")
    arm.send_manual(safe_base, safe_shoulder, safe_elbow, gripper=70)
    time.sleep(1.0)

    print("\nDone.")
    arm.close()

if __name__ == "__main__":
    test_gripper()
