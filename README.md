# 🤖 Smart Home & Assistive Robot System

**Autonomous Object Retrieval Pipeline — Navigation · Alignment · Grasping**

> ESI-SBA — 2CS ISI | Smart Home & Assistive Robotics Module  
> Academic Year 2024–2025 | Supervised by **Pr. Rahmoune**

---

## 📋 Overview

This project implements a **fully autonomous pick-and-place robotic system** built on severely resource-constrained edge hardware (Raspberry Pi 3B). The robot can:

1. **Navigate** autonomously to predefined stations using SLAM-based positioning and IMU heading
2. **Detect** target objects using classical HSV computer vision (no GPU/deep learning required)
3. **Align** precisely to the target via a closed-loop proportional controller
4. **Approach** the object using monocular pixel-area distance estimation
5. **Grasp** the object with a 4-DOF robotic arm using pre-calibrated pose sequences

The entire pipeline runs with **sub-5ms detection latency** and requires zero human intervention once started.

---

## 🏗️ System Architecture

The system uses a **distributed two-node architecture**:

```
┌──────────────────────────────┐         ┌──────────────────────────────┐
│     RASPBERRY PI 3B          │         │      PC / WORKSTATION        │
│     (Edge Device)            │         │    (Central Processor)       │
│                              │         │                              │
│  ▸ Camera Driver (libcamera) │◄───────►│  ▸ Vision Pipeline (OpenCV)  │
│  ▸ Motor Controller (Flask)  │  Wi-Fi  │  ▸ State Machine Controller  │
│  ▸ Arm Servo Listener        │         │  ▸ Arm Pose Sequencer        │
│    (pigpio HAL)              │         │  ▸ Navigation Engine         │
└──────────────────────────────┘         └──────────────────────────────┘
         │                                         │
         │  ZMQ PUB/SUB  ─── Camera frames ───►    │
         │  ZMQ PUB/SUB  ◄── Arm commands ────     │
         │  HTTP REST    ◄── Motor commands ───     │
         └─────────────────────────────────────────┘
```

### Communication Protocols

| Protocol | Direction | Purpose |
|----------|-----------|---------|
| **ZMQ PUB/SUB** | Pi → PC | High-throughput, low-latency video streaming. `CONFLATE` ensures only latest frame is processed. |
| **ZMQ PUB/SUB** | PC → Pi | Arm servo commands with smooth interpolation. |
| **HTTP REST** | PC → Pi | Stateless motor control. Missed commands self-recover. |

---

## 📁 Repository Structure

```
Smart-Assistive-Home-System/
│
├── raspberry_pi/                    # Code that runs ON the Raspberry Pi
│   ├── motor_control/
│   │   └── car.py                   # Flask HTTP server for 4WD differential drive
│   ├── camera/
│   │   └── camera_stream.py         # MJPEG HTTP stream server (Pi Camera V2)
│   └── arm_servo/
│       └── arm_servo_listener.py    # ZMQ listener → pigpio servo driver (HAL)
│
├── pc/                              # Code that runs on the PC/workstation
│   ├── vision/
│   │   ├── color_detect.py          # HSV color detection pipeline
│   │   └── detect.py                # Basic detection utilities
│   ├── alignment/
│   │   └── align_to_object.py       # Proportional alignment controller
│   ├── pickup/
│   │   └── pickup_sequence.py       # End-to-end pickup state machine
│   ├── navigation/
│   │   └── navigator.py             # SLAM+IMU L-shape station routing
│   └── calibration/
│       ├── arm_calibrate.py         # Interactive arm calibration CLI
│       ├── arm_web_control.py       # Web-based arm joint sliders
│       ├── arm_test.py              # Arm movement tests
│       └── test_gripper.py          # Gripper open/close test
│
├── dashboard/                       # Web-based monitoring & control
│   ├── web_control/
│   │   ├── server.py                # FastAPI navigation dashboard backend
│   │   └── index.html               # Real-time map + station control UI
│   └── station_mapper/
│       └── mapping_tool.py          # FastAPI tool to capture station poses
│
├── config/
│   └── stations.json                # Named station coordinates & orientations
│
├── tests/
│   ├── test_navigator.py            # Full 25-test suite (hardware → integration)
│   ├── test_generic.py              # Generic utility tests
│   ├── navigator_mock.py            # Mock navigator for testing without hardware
│   └── navigator_auto_mock.py       # Auto-mode mock navigator
│
├── docs/
│   ├── Autonomous_Object_Retrieval_Report.pdf   # Full academic report (LaTeX)
│   ├── arm_movement_logic.md        # Arm kinematics documentation
│   ├── autonomous_pickup_pipeline.md # Pipeline design document
│   └── website_integration_guide.md  # Dashboard integration guide
│
├── .gitignore
└── README.md                        # ← You are here
```

---

## ⚙️ How It Works

### The Master State Machine

The entire retrieval pipeline is orchestrated by a finite state machine:

```
IDLE ──► SCANNING ──► ALIGNING ──► APPROACHING ──► GRASPING ──► CARRYING ──► DONE
              │            │             │              │
              └── Timeout/lost ──────────┘──────────────┘
```

### Phase 1: Computer Vision (HSV Detection)

Instead of deep learning, we use a **classical 4-stage image processing pipeline**:

1. **Gaussian Blur** — Removes high-frequency noise (7×7 kernel)
2. **HSV Thresholding** — Color detection robust to lighting changes (hue separated from brightness)
3. **Morphological Ops** — Close → Open sequence to clean binary mask
4. **Contour Analysis** — Largest contour → image moments → centroid (cx, cy) + apparent area

### Phase 2: Alignment (Closed-Loop Controller)

A **proportional pulse controller** in image space:

- Error: `Ex = cx - W/2` (offset from frame center)
- Dead zone: ±40 pixels (prevents oscillation/hunting)
- Pulse duration: `T = kp × |Ex|` (large error → long pulse, small → micro-pulse)
- Confirmation: Must be centered for **3 consecutive frames** before proceeding

### Phase 3: Approach (Monocular Distance Estimation)

Using the **inverse-square law** of perspective projection: `A ∝ 1/d²`

- Robot drives forward in short bursts, checking pixel area after each
- Stops when area ≥ 15,000 px² (calibrated optimal grasp distance)
- **ZUPT-inspired stall detection**: if area doesn't grow over N readings → escalating recovery:
  - **Soft boost**: 60% PWM for 300ms
  - **Hard unstick**: Reverse jog + 80% PWM forward lunge

### Phase 4: Robotic Arm Manipulation

The 4-DOF arm uses **pre-calibrated pose sequences** (not runtime IK) to eliminate accumulated error from encoder-less servos:

| State | Shoulder | Elbow | Gripper | Purpose |
|-------|----------|-------|---------|---------|
| Stow | 20° | 100° | 70° (closed) | Tucked during navigation |
| Extend | 120° | 170° | 70° (closed) | Reach forward |
| Open | 120° | 170° | 0° (open) | Ready to receive |
| Grab | 120° | 170° | 70° (closed) | Clamp object |
| Lift | 140° | 120° | 70° (closed) | Raise payload |

All transitions use **linear interpolation** (max 2°/joint per 20ms) to prevent mechanical shock.

### Navigation (SLAM + IMU)

L-shaped station routing with 3 phases:
1. **Drive to elbow** (waypoint) with SLAM position tracking and IMU heading correction
2. **Spin 90°** (direction computed via 2D cross product)
3. **Dock straight** into standoff point (0.3m clearance from station)

---

## 🔧 Hardware

| Component | Model | Role |
|-----------|-------|------|
| Edge Processor | Raspberry Pi 3B | GPIO, motor/servo control, camera |
| Camera | IMX219 (Pi Camera V2) | 8MP, 1080p30 — primary sensor |
| Motor Drivers | Dual L298N H-Bridge | 4WD differential drive (PWM) |
| Drive Motors | DC Gear Motors ×4 | 6V, 200 RPM |
| Shoulder/Elbow | MG995 Servo ×2 | High-torque (9.4 kg·cm) |
| Base/Gripper | SG90 Micro Servo ×2 | Lightweight positioning |
| Network | Wi-Fi 802.11n | Pi hotspot mode |
| Power (Logic) | 5V / 3A USB-C | Isolated from motors |
| Power (Motors) | 7.4V LiPo 2S | Clean motor voltage |

---

## 🚀 Quick Start

### Prerequisites

```bash
# On your PC
pip install opencv-python numpy pyzmq requests matplotlib scipy flask fastapi uvicorn

# On the Raspberry Pi
sudo apt install pigpio python3-pigpio python3-flask python3-zmq
sudo pigpiod   # start GPIO daemon
```

### 1. Start services on the Raspberry Pi

```bash
# Terminal 1: Motor control server
python3 raspberry_pi/motor_control/car.py

# Terminal 2: Camera stream
python3 raspberry_pi/camera/camera_stream.py

# Terminal 3: Arm servo listener
python3 raspberry_pi/arm_servo/arm_servo_listener.py
```

### 2. Run from your PC

```bash
# Full autonomous pickup pipeline
python3 pc/pickup/pickup_sequence.py --color blue

# Navigation only (to a named station)
python3 pc/navigation/navigator.py station1 --stations config/stations.json

# Vision detection test
python3 pc/vision/color_detect.py --color red

# Arm calibration
python3 pc/calibration/arm_calibrate.py
```

### 3. Web Dashboard

```bash
# Navigation dashboard with live map
python3 dashboard/web_control/server.py --mode test --port 8080
# Open http://localhost:8080

# Station mapping tool
uvicorn dashboard.station_mapper.mapping_tool:app --host 0.0.0.0 --port 8000
# Open http://localhost:8000
```

### 4. Run Tests

```bash
# All tests
python3 tests/test_navigator.py

# By section
python3 tests/test_navigator.py geometry    # Pure math (no hardware)
python3 tests/test_navigator.py hardware    # Motor direction checks
python3 tests/test_navigator.py readers     # IMU + SLAM connectivity
python3 tests/test_navigator.py navigator   # Full integration

# Single test
python3 tests/test_navigator.py T12         # Run test T12 only
```

---

## 🎯 Key Engineering Decisions

| Decision | Rationale |
|----------|-----------|
| HSV over RGB | Illumination-robust color separation |
| Area-based depth | No extra sensors; monocular camera only |
| Pre-calibrated poses | Eliminates accumulated IK error from encoder-less servos |
| ZUPT stall detection | Detects zero-progress without wheel odometry or IMU |
| Proportional pulse control | Simple P-controller; no integral windup risk |
| pigpio DMA PWM | Microsecond-accurate servo signals; no OS jitter |
| Linear interpolation profile | Prevents mechanical shock; protects servos and payload |
| 3-frame confirmation | Filters transient centroid jitter; prevents premature approach |
| ZMQ CONFLATE socket | Vision pipeline always processes current frames only |
| HTTP for motor control | Stateless; missed commands isolated; built-in failure recovery |

---

## 📄 Documentation

The full academic report is available at [`docs/Autonomous_Object_Retrieval_Report.pdf`](docs/Autonomous_Object_Retrieval_Report.pdf). It covers:

- System architecture and distributed node design
- Computer vision pipeline (Gaussian blur → HSV → morphology → moments)
- Proportional alignment controller with dead-zone anti-hunting
- Monocular distance estimation and ZUPT-inspired stall recovery
- Robotic arm kinematics and pre-calibrated pose sequences
- Hardware abstraction layer (pigpio DMA PWM + smooth interpolation)
- Full system integration and state machine design
- Hardware specifications

---

## 👥 Team

- **Rabahi Mohamed Fouad**
- **Miloudi A. Aboubaker Esseddik**
- **Lakhdari Anis Charef Eddine**
- **Ameur Mohammed Menouer**

**Supervisor:** Pr. Rahmoune — ESI-SBA

---

## 📝 License

This project was developed as part of the Smart Home & Assistive Robotics module at ESI-SBA (École Supérieure en Informatique de Sidi Bel Abbès).
