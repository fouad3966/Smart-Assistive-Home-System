# Autonomous Object Retrieval Pipeline: Navigation to Manipulation

## 1. System Architecture Overview

The autonomous retrieval system is distributed across two primary computational nodes: an edge device (Raspberry Pi) handling hardware interfacing, and a central processing unit (PC) executing high-level computer vision and orchestrating state machines.

Communication between these nodes relies on two distinct protocols tailored to their specific latency and bandwidth requirements:
*   **Video Streaming & Arm Control:** ZeroMQ (ZMQ) PUB/SUB sockets. ZMQ provides high-throughput, low-latency streaming. The camera stream utilizes `CONFLATE` to drop stale frames, ensuring the vision processing pipeline only operates on the most recent temporal data.
*   **Motor Control:** RESTful HTTP via Flask. The mobile platform's L298N motor drivers are controlled via simple stateless HTTP requests, ensuring robust command execution and built-in failure recovery.

## 2. Computer Vision and Object Detection

The system utilizes OpenCV to process the live camera feed and identify target objects. To ensure computational efficiency and robustness against varying lighting conditions, the detection pipeline operates in the HSV (Hue, Saturation, Value) color space.

### 2.1. Image Processing Pipeline
1.  **Gaussian Blurring:** A 7x7 Gaussian blur is applied to the raw frame to reduce high-frequency noise and prevent false contour detections.
2.  **HSV Thresholding:** The image is converted from BGR to HSV. A pre-calibrated bounding range is applied to isolate the target color. For example, black objects are detected by thresholding the 'Value' (brightness) channel, ignoring hue variations.
3.  **Morphological Operations:** To ensure a clean binary mask, morphological closing (dilation followed by erosion) fills small holes within the detected object, and morphological opening (erosion followed by dilation) removes isolated background noise pixels.
4.  **Contour Extraction & Spatial Moments:** The system extracts the largest contour from the mask. Using image moments ($m_{10}$, $m_{01}$, $m_{00}$), the exact geometric centroid $(c_x, c_y)$ and the pixel area of the object are calculated.

## 3. Automated Navigation and Alignment

Once an object is detected, the autonomous platform must orient itself to face the object dead-on. This is achieved using a proportional closed-loop control system.

### 3.1. Error Calculation and Dead Zones
The system calculates the spatial error, $E_x$, defined as the difference between the object's centroid X-coordinate ($c_x$) and the center axis of the camera frame. 

To prevent the robot from oscillating infinitely around the exact center point (hunting), a **Dead Zone** of $\pm 40$ pixels is established. If $|E_x| < 40$, the robot is considered aligned. 

### 3.2. Proportional Pulse Control
If the object is outside the dead zone, the car issues short rotational pulses to the motors. The duration of the pulse ($T_{pulse}$) is proportional to the magnitude of the error:
$$ T_{pulse} = k_p \times |E_x| $$
This ensures rapid turning when the object is at the edge of the frame, and gentle, precise micro-adjustments as the object nears the center. The system requires the object to remain within the dead zone for three consecutive frames to confirm a successful alignment state.

## 4. Distance Estimation and Approach

With the robot perfectly aligned, it must drive forward until it reaches the optimal distance for the robotic arm to perform a grasp. 

### 4.1. Area-Based Distance Estimation
Rather than using external ultrasonic or LiDAR sensors, the system relies on monocular depth cues. As the robot approaches the object, the bounding box area (in pixels) of the object increases exponentially. The optimal grabbing distance was experimentally mapped to a target area threshold ($Area \ge 15000$ px²). The car drives forward iteratively until this threshold is breached.

### 4.2. ZUPT-Style Stall Detection and Auto-Recovery
Mobile platforms operating on high-friction surfaces (e.g., carpets) frequently encounter stalls where motor torque is insufficient to overcome static friction, especially at the lower PWM speeds used for precise approaching.

To combat this, the pipeline incorporates a rolling-window stall detection algorithm inspired by Zero Velocity Update (ZUPT) principles in inertial navigation.
*   **Monitoring:** The system tracks the object's pixel area over a rolling window. 
*   **Detection:** If the motors are active but the pixel area fails to grow by a minimum delta threshold, the system flags a physical stall.
*   **Recovery:** The system automatically triggers an escalating recovery protocol. First, it attempts a "Soft Boost" by briefly spiking motor PWM to 60%. If the stall persists, it executes a "Hard Unstick"—a rapid reverse jog followed by a high-torque forward lunge to break static friction, ensuring the robot reliably reaches the target.

## 5. Robotic Arm Kinematics and Manipulation

Upon reaching the target zone, the mobile base halts, and control is handed over to the 4-DOF robotic arm. To maximize reliability and prevent collisions with the object or the floor, the arm relies on a highly optimized, state-based sequence of pre-calibrated joint poses.

### 5.1. The Pre-Calibrated Pose System
Calculating dynamic inverse kinematics on the fly introduces significant margin for error, particularly with budget servo motors lacking absolute encoders. Instead, the manipulation sequence utilizes a hardened, state-machine approach where critical manipulation points are mapped to explicit joint angles (Base, Shoulder, Elbow, Gripper).

### 5.2. The Manipulation Sequence
1.  **Stow/Default Phase:** During navigation, the arm is tucked low (Shoulder: 20°, Elbow: 100°) to ensure the camera's field of view remains entirely unobstructed.
2.  **Extend Phase:** Once in position, the elbow extends outward (170°). Crucially, the shoulder remains elevated (120°). This elevated approach trajectory guarantees the arm does not sweep across the ground and accidentally knock the target object away.
3.  **Open & Grab Phase:** The gripper opens (0°), and then snaps shut (70°). The system introduces a dedicated 2.0-second delay to ensure the servo has reached maximum holding torque.
4.  **Lift & Secure Phase:** The shoulder elevates (140°) to lift the object, and the elbow retracts slightly to shift the payload's center of mass closer to the robot's wheelbase, maintaining physical stability. A final gripper close command is issued to secure the hold.

## 6. Hardware Abstraction and Smooth Control

Commanding a servo to immediately jump from 20° to 140° results in a violent mechanical snap, which can damage the servos, drop the payload, or flip the robot.

To achieve fluid, life-like robotic motion, the edge device runs a dedicated hardware abstraction layer (`arm_servo_listener.py`) utilizing the `pigpio` daemon for precise, hardware-timed PWM signals. 

When the listener receives a target pose via ZMQ, it does not apply the angles instantaneously. Instead, a `smooth_move()` algorithm calculates the delta between the current hardware state and the target state. It linearly interpolates the movement, stepping the servos by a maximum of 2° every 20 milliseconds. This software-defined acceleration profile ensures smooth, sweeping kinematics without requiring specialized robotic motor drivers.
