"""
arm_web_control.py - Web Interface for manual arm control.

Runs a lightweight Flask web server on your PC.
Open http://127.0.0.1:5050 in your browser to see the sliders.

Requirements:
    pip install flask
"""

import json
import logging
from flask import Flask, request, jsonify, render_template_string
from pickup_sequence import ArmController, PI_IP, ARM_ZMQ_ADDR

# Suppress flask logging for cleaner terminal output
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

app = Flask(__name__)
arm = None

# We lock base and gripper to safe default values
SAFE_BASE = 90
SAFE_GRIPPER = 70

HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>Arm Web Control</title>
    <style>
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background-color: #121212;
            color: #ffffff;
            display: flex;
            justify-content: center;
            align-items: center;
            height: 100vh;
            margin: 0;
        }
        .container {
            background: #1e1e1e;
            padding: 30px;
            border-radius: 12px;
            box-shadow: 0 8px 16px rgba(0,0,0,0.5);
            width: 350px;
            text-align: center;
        }
        h2 {
            margin-top: 0;
            color: #4CAF50;
        }
        .slider-group {
            margin: 25px 0;
            text-align: left;
        }
        .label {
            display: flex;
            justify-content: space-between;
            font-weight: bold;
            margin-bottom: 10px;
        }
        input[type=range] {
            width: 100%;
            cursor: pointer;
        }
        .status {
            margin-top: 20px;
            font-size: 0.9em;
            color: #aaa;
        }
    </style>
</head>
<body>
    <div class="container">
        <h2>Arm Control Panel</h2>
        
        <div class="slider-group">
            <div class="label">
                <span>Shoulder (20-140)</span>
                <span id="shoulder-val">80&deg;</span>
            </div>
            <input type="range" id="shoulder" min="20" max="140" value="80" oninput="updateVal('shoulder')" onchange="sendData()">
        </div>

        <div class="slider-group">
            <div class="label">
                <span>Elbow (100-170)</span>
                <span id="elbow-val">100&deg;</span>
            </div>
            <input type="range" id="elbow" min="100" max="170" value="100" oninput="updateVal('elbow')" onchange="sendData()">
        </div>
        
        <div class="status" id="status">Ready.</div>
    </div>

    <script>
        function updateVal(joint) {
            document.getElementById(joint + '-val').innerHTML = document.getElementById(joint).value + '&deg;';
        }

        function sendData() {
            const shoulder = document.getElementById('shoulder').value;
            const elbow = document.getElementById('elbow').value;
            const status = document.getElementById('status');
            
            status.innerHTML = "Sending...";
            status.style.color = "#ffa500";

            fetch('/set_angles', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    shoulder: parseInt(shoulder),
                    elbow: parseInt(elbow)
                })
            })
            .then(response => response.json())
            .then(data => {
                if(data.success) {
                    status.innerHTML = "Sent OK";
                    status.style.color = "#4CAF50";
                } else {
                    status.innerHTML = "Error!";
                    status.style.color = "#ff4444";
                }
                setTimeout(() => { status.innerHTML = "Ready."; status.style.color = "#aaa"; }, 1000);
            })
            .catch(err => {
                status.innerHTML = "Network Error!";
                status.style.color = "#ff4444";
            });
        }
    </script>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route('/set_angles', methods=['POST'])
def set_angles():
    data = request.json
    shoulder = data.get('shoulder', 80)
    elbow = data.get('elbow', 100)
    
    print(f"[web] -> shoulder={shoulder}°  elbow={elbow}°")
    # We send the manual pose using the ArmController
    arm.send_manual(SAFE_BASE, shoulder, elbow, SAFE_GRIPPER)
    
    return jsonify({"success": True})

if __name__ == "__main__":
    print("="*50)
    print("  Starting Web Arm Controller")
    print(f"  Target Pi IP: {PI_IP}")
    print("  Go to: http://127.0.0.1:5050 in your browser")
    print("="*50)
    
    # Init ZMQ connection
    arm = ArmController()
    
    # Run the web server
    app.run(host='127.0.0.1', port=5050, threaded=True)
