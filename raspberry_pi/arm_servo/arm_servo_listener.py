"""
arm_servo_listener.py — ZMQ listener that drives arm servos via GPIO.

Run this ON THE RASPBERRY PI. It listens for JSON commands from the PC
and drives the base, shoulder, elbow, and gripper servos accordingly.

Usage (on Pi):
    sudo pigpiod               # start the GPIO daemon first
    python3 arm_servo_listener.py

The PC sends commands via ZMQ PUB to tcp://<PI_IP>:5559
This script binds a SUB socket on port 5559 to receive them.

Command format (JSON):
    {"base": 90, "shoulder": 140, "elbow": 100, "gripper": 170}
"""

import json
import time

import zmq

# ── GPIO pin mapping (BCM) ───────────────────────────────────────────────────
GPIO_BASE     = 4     # Pin 7
GPIO_SHOULDER = 8     # Pin 24
GPIO_ELBOW    = 7     # Pin 26
GPIO_GRIPPER  = 9     # Pin 21  (NOT 25 — that conflicts with L298N motor driver!)

# ── Servo limits ──────────────────────────────────────────────────────────────
LIMITS = {
    "base":     (0, 180),
    "shoulder": (20, 140),
    "elbow":    (100, 170),
    "gripper":  (0, 70),
}

GPIO_MAP = {
    "base":     GPIO_BASE,
    "shoulder": GPIO_SHOULDER,
    "elbow":    GPIO_ELBOW,
    "gripper":  GPIO_GRIPPER,
}

# ── ZMQ config ────────────────────────────────────────────────────────────────
ZMQ_PORT = 5559

# ── Smooth movement ──────────────────────────────────────────────────────────
STEP_DELAY = 0.02   # seconds between angle increments
STEP_SIZE  = 2      # degrees per step


class ServoController:
    """Drives servos via pigpio (falls back to RPi.GPIO)."""

    def __init__(self):
        self._current = {}   # {gpio_pin: current_angle}
        self._backend = None
        self._pi = None

        try:
            import pigpio
            self._pi = pigpio.pi()
            if self._pi.connected:
                self._backend = "pigpio"
                print("[servo] using pigpio (hardware PWM)")
            else:
                raise RuntimeError("pigpio daemon not running — run: sudo pigpiod")
        except Exception as e:
            print(f"[servo] pigpio failed ({e}), trying RPi.GPIO")
            try:
                import RPi.GPIO as GPIO
                GPIO.setmode(GPIO.BCM)
                GPIO.setwarnings(False)
                self._backend = "rpigpio"
                self._pwm = {}
                print("[servo] using RPi.GPIO (software PWM — may jitter)")
            except ImportError:
                print("[servo] ERROR: no GPIO library available!")
                raise

    def set_angle(self, gpio_pin: int, angle: int):
        angle = max(0, min(180, angle))

        if self._backend == "pigpio":
            pw = int(500 + (angle / 180.0) * 2000)
            self._pi.set_servo_pulsewidth(gpio_pin, pw)
        elif self._backend == "rpigpio":
            import RPi.GPIO as GPIO
            if gpio_pin not in self._pwm:
                GPIO.setup(gpio_pin, GPIO.OUT)
                self._pwm[gpio_pin] = GPIO.PWM(gpio_pin, 50)
                self._pwm[gpio_pin].start(0)
            duty = 2.5 + (angle / 180.0) * 10.0
            self._pwm[gpio_pin].ChangeDutyCycle(duty)

        self._current[gpio_pin] = angle

    def smooth_move(self, gpio_pin: int, target: int):
        current = self._current.get(gpio_pin, 90)
        target = max(0, min(180, target))
        if current == target:
            return

        direction = 1 if target > current else -1
        pos = current
        while abs(target - pos) > STEP_SIZE:
            pos += direction * STEP_SIZE
            self.set_angle(gpio_pin, pos)
            time.sleep(STEP_DELAY)
        self.set_angle(gpio_pin, target)

    def cleanup(self):
        if self._backend == "pigpio" and self._pi:
            for pin in GPIO_MAP.values():
                self._pi.set_servo_pulsewidth(pin, 0)
            self._pi.stop()
        elif self._backend == "rpigpio":
            import RPi.GPIO as GPIO
            for p in self._pwm.values():
                p.stop()
            GPIO.cleanup()
        print("[servo] GPIO cleaned up")


def main():
    servo = ServoController()

    # Initialize to safe position
    print("[servo] moving to stow position...")
    for name, gpio in GPIO_MAP.items():
        lo, hi = LIMITS[name]
        safe = hi  # max = retracted/up/open
        servo.smooth_move(gpio, safe)
        time.sleep(0.3)
    print("[servo] ✓ stow position reached\n")

    # ZMQ subscriber
    ctx = zmq.Context()
    sock = ctx.socket(zmq.SUB)
    sock.bind(f"tcp://*:{ZMQ_PORT}")
    sock.setsockopt_string(zmq.SUBSCRIBE, "")
    print(f"[servo] listening on tcp://*:{ZMQ_PORT}")
    print("[servo] waiting for commands...\n")

    try:
        while True:
            raw = sock.recv_string()
            try:
                cmd = json.loads(raw)
            except json.JSONDecodeError:
                print(f"  [servo] bad JSON: {raw}")
                continue

            smooth = cmd.pop("smooth", True)

            print(f"  [servo] ← {cmd}  (smooth={smooth})")

            for name in ("base", "shoulder", "elbow", "gripper"):
                if name not in cmd:
                    continue
                angle = int(cmd[name])
                lo, hi = LIMITS[name]
                angle = max(lo, min(hi, angle))
                gpio = GPIO_MAP[name]

                if smooth:
                    servo.smooth_move(gpio, angle)
                else:
                    servo.set_angle(gpio, angle)

            print(f"  [servo] ✓ done")

    except KeyboardInterrupt:
        print("\n[servo] shutting down")
    finally:
        servo.cleanup()
        sock.close()
        ctx.term()


if __name__ == "__main__":
    main()
