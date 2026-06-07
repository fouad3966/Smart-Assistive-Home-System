# Robotic Arm Movement Logic

This document explains how the 4-DOF robotic arm is controlled during the autonomous pickup sequence. 

## 1. The Hardware & Calibration

The robotic arm uses 4 servo motors connected to Raspberry Pi GPIO pins. Each servo expects a specific angle value. However, servos have physical limits, and their "0 degrees" rarely matches what we intuitively think of as "straight". 

*   **Base:** Swivels left and right.
*   **Shoulder:** Moves the entire arm up and down.
    *   `140°` = Arm lifted high up.
    *   `20°` = Arm tucked low down.
*   **Elbow:** Extends or retracts the forearm.
    *   `170°` = Arm fully extended straight out.
    *   `100°` = Arm pulled back (retracted).
*   **Gripper:** Opens and closes the claw.
    *   `0°` = Claw wide open.
    *   `70°` = Claw fully closed tight.

Because these angles are specific to how the servos were physically assembled on your robot, we don't calculate complex kinematics (like "move hand to X=10, Y=5"). Instead, we use **pre-defined "Poses"**.

## 2. The Pose System

Instead of guessing angles on the fly, `pickup_sequence.py` contains a dictionary of safe, tested poses named `ARM_POSES`. Every time we want the arm to do something, we send a single command: *"Go to the 'open' pose."* 

The script sends this as a JSON dictionary to the Pi:
```json
{"base": 90, "shoulder": 120, "elbow": 160, "gripper": 170}
```

### The Sequence

The entire pickup process is built as a step-by-step animation using these poses. 

1.  **`default`** (Phase 0 - Driving)
    *   **Angles:** `shoulder: 20, elbow: 100`
    *   **Why:** Before the car starts driving and using the camera to search, the arm needs to get out of the way. We drop the shoulder all the way down and pull the elbow back so the camera has a clear, unobstructed view of the floor.

2.  **`extend`** (Phase 4 - Start of Pickup)
    *   **Angles:** `shoulder: 120, elbow: 160`
    *   **Why:** The car has stopped in front of the object. We push the elbow forward (`160`) to reach out, but we keep the shoulder HIGH (`120`). **We do not dip down** because sweeping low across the ground can hit the object and knock it away before we can grab it.

3.  **`open`**
    *   **Angles:** `gripper: 170` (shoulder and elbow stay exactly the same)
    *   **Why:** Now that the arm is hovering over/near the object, we open the claw wide so it's ready to clamp down.

4.  **`grab`**
    *   **Angles:** `gripper: 70`
    *   **Why:** We snap the claw shut around the object. We give this step extra time (`GRIP_SETTLE_TIME = 2.0s`) to ensure the servos have fully tightened their grip before moving on.

5.  **`lift`**
    *   **Angles:** `shoulder: 140, elbow: 120`
    *   **Why:** With the object secured, we immediately lift the shoulder up (`140`) to get the object off the ground. We also pull the elbow back slightly (`120`) to bring the weight closer to the car's center of gravity so it doesn't tip forward.

6.  **`secure`**
    *   **Angles:** `elbow: 100` (gripper remains at `70`)
    *   **Why:** We pull the elbow all the way back to its stowed position. We send the `gripper: 70` command one more time just to ensure the servo is actively applying pressure and holding the object tight while driving.

## 3. Smooth Movement (ZMQ Listener)

If we instantly changed a servo from 20° to 140°, the arm would violently snap upward, potentially throwing the object or flipping the car. 

To solve this, the logic is split across the network:
1.  **PC side (`pickup_sequence.py`)**: Says *"Go to pose X"*.
2.  **Pi side (`arm_servo_listener.py`)**: Receives the target angles, compares them to the *current* angles, and uses a `smooth_move()` function. It chops the big jump into tiny 2° steps, pausing for 0.02 seconds between each step. This creates a smooth, sweeping robotic motion instead of a violent jerk.
