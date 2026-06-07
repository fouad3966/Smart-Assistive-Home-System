# Web Integration Guide: Linking the Autonomous Car to a Frontend

This document outlines the architecture, APIs, and network protocols required to achieve real-time, bi-directional synchronization between the robotic car and a web application.

## 1. Architectural Overview: The "Bridge" Backend

Web browsers rely on standard HTTP and WebSockets, while the robotic car utilizes ZeroMQ (ZMQ) for low-latency arm control and internal LAN IP routing. Because a web frontend cannot securely or natively speak ZMQ, **you must build a Backend Bridge (e.g., Node.js/Express, Python/FastAPI, or Python/Flask with Socket.IO).**

**The Flow:**
1.  **Web to Car:** User clicks a button on the website → Website sends WebSocket/HTTP request to your Bridge Backend → Bridge Backend sends the actual ZMQ/HTTP request to the Raspberry Pi.
2.  **Car to Web:** Python automation scripts running on the PC/Pi send HTTP POST requests to your Bridge Backend when states change → Bridge Backend emits a WebSocket event to the Website → Website UI updates.

---

## 2. Web -> Car: Controlling the Hardware

### 2.1. Controlling the Motors (Driving)
The car's mobile base is controlled via a simple REST API hosted on the Raspberry Pi.

*   **Endpoint:** `http://10.213.37.191:5000/drive`
*   **Method:** `POST`
*   **Content-Type:** `application/json`
*   **Payload Schema:**
    ```json
    {
      "direction": "forward", 
      "speed": 50,           
      "duration": 0           
    }
    ```
    *   `direction`: string (`"forward"`, `"backward"`, `"left"`, `"right"`, `"stop"`).
    *   `speed`: integer (0 to 100). Determines PWM duty cycle.
    *   `duration`: float. Seconds to run before auto-stopping. Use `0` for continuous movement.

### 2.2. Controlling the Robotic Arm
The arm is controlled via a **ZeroMQ (ZMQ) PUB/SUB socket**. Your backend bridge must implement a ZMQ Publisher to send these commands.

*   **ZMQ Protocol:** `PUB` (Publisher)
*   **Address:** `tcp://10.213.37.191:5559`
*   **Payload Schema:** A stringified JSON object containing target angles.
    ```json
    {
      "base": 90,
      "shoulder": 120,
      "elbow": 170,
      "gripper": 70,
      "smooth": true
    }
    ```
    *   **Ranges:** Base (0-180), Shoulder (20-140), Elbow (100-170), Gripper (0=Open, 70=Closed).
    *   `smooth`: boolean. If `true`, the Pi interpolates the movement to prevent violent snapping.

---

## 3. Car -> Web: Real-Time State Synchronization

To make the website reflect what the car is doing autonomously (e.g., "Aligning...", "Approaching...", "Grabbing..."), the existing Python scripts (`pickup_sequence.py`, `align_to_object.py`) need to report their status to your web backend.

### The Recommended Setup: Webhooks
You should expose an endpoint on your Bridge Backend, for example: `POST https://your-backend.com/api/car-status`

Inside the Python automation scripts, the robotics engineer will inject a simple `requests.post()` whenever a major event occurs:

**Example Python Injection:**
```python
import requests

def update_website_state(state, details=""):
    try:
        requests.post("https://your-backend.com/api/car-status", json={
            "state": state,         # e.g., "SEARCHING", "ALIGNING", "PICKUP"
            "details": details      # e.g., "Offset: -38px", "Target reached"
        }, timeout=1)
    except:
        pass # Ignore network errors to keep the robot moving
```

When your Bridge Backend receives this webhook, it immediately forwards the payload to the frontend via a **WebSocket (Socket.IO)**, allowing the React/Vue/HTML UI to update instantly without refreshing.

---

## 4. Video Streaming to the Web

The Raspberry Pi is currently broadcasting the camera feed over a ZMQ socket (`tcp://10.213.37.191:5555`). Browsers cannot read ZMQ streams natively. 

**How to show the video on the website:**
1.  **Option A (Backend Transcoding - Recommended):** Your Bridge Backend subscribes to the ZMQ stream, receives the JPEG frames, and serves them to the frontend via an HTTP MJPEG stream (Motion JPEG) or WebSocket. The frontend simply uses `<img src="http://your-backend.com/video_feed">`.
2.  **Option B (Pi Direct Server):** Run a dedicated streaming server (like `mjpg-streamer` or a basic Flask script) directly on the Raspberry Pi that exposes an HTTP endpoint specifically for the browser.

---

## 5. Quick-Start Checklist for the Web Developer
1. [ ] Create a Node.js or Python backend.
2. [ ] Install the `zeromq` library in your backend to talk to the Arm.
3. [ ] Set up HTTP endpoints on your backend to act as proxies for the Car's motor API.
4. [ ] Set up a Webhook endpoint (`/api/car-status`) on your backend to receive updates from the autonomous python scripts.
5. [ ] Integrate WebSockets (e.g., Socket.io) between your backend and frontend to push live status updates to the UI in real-time.
