"""
color_detect.py — HSV color detection for the arm-nav pipeline.

Runs on YOUR PC. Reads the camera feed from the Pi's existing ZMQ stream
(camera_zmq.py on port 5555) and shows a live window with detection overlay.

Requires camera_zmq.py already running on the Pi (port 5555).

Usage:
    python3 color_detect.py                          # black, Pi stream
    python3 color_detect.py --color red               # detect red
    python3 color_detect.py --camera 0                # local webcam (testing)
"""

import argparse
import time
import cv2
import numpy as np
import zmq

# ── Pi network config ─────────────────────────────────────────────────────────
PI_IP      = "10.213.37.191"
ZMQ_PORT   = 5555
STREAM_ADDR = f"tcp://{PI_IP}:{ZMQ_PORT}"

# ── HSV colour presets ────────────────────────────────────────────────────────
# BLACK: no hue — threshold on low Value channel only.
# Tweak V_MAX up if camera is dark, down if shadows trigger false positives.

COLOR_PRESETS = {
    "black": [
        (np.array([0,   0,   0]), np.array([180,  255,  50])),
    ],
    "red": [
        (np.array([0,   100, 80]), np.array([10,  255, 255])),
        (np.array([170, 100, 80]), np.array([180, 255, 255])),
    ],
    "blue": [
        (np.array([100, 120, 60]), np.array([130, 255, 255])),
    ],
    "green": [
        (np.array([35,  80,  60]), np.array([85,  255, 255])),
    ],
    "yellow": [
        (np.array([20,  100, 100]), np.array([35,  255, 255])),
    ],
}

# ── Detection tuning ─────────────────────────────────────────────────────────
MIN_CONTOUR_AREA   = 500      # px² — ignore tiny blobs / noise
BLUR_KERNEL        = (7, 7)   # Gaussian blur before threshold
MORPH_KERNEL_SIZE  = 5        # morphology kernel to clean mask


class ColorDetector:
    """
    Detect a coloured object from the Pi's ZMQ camera stream.

    Parameters
    ----------
    stream_addr : str | None
        ZMQ PUB address (e.g. tcp://10.213.37.191:5555).
        If None, uses a local camera instead.
    camera_index : int
        Local camera index (only used if stream_addr is None).
    target_color : str
        Key into COLOR_PRESETS.
    """

    def __init__(
        self,
        stream_addr: str | None = STREAM_ADDR,
        camera_index: int = 0,
        target_color: str = "black",
        custom_ranges=None,
    ):
        self._stream_addr = stream_addr
        self._cam_idx     = camera_index
        self._color       = target_color

        if custom_ranges is not None:
            self._ranges = custom_ranges
        elif target_color in COLOR_PRESETS:
            self._ranges = COLOR_PRESETS[target_color]
        else:
            raise ValueError(
                f"Unknown colour '{target_color}'. "
                f"Available: {list(COLOR_PRESETS.keys())}"
            )

        self._zmq_ctx  = None
        self._zmq_sock = None
        self._cap      = None   # only used for local camera fallback
        self._opened   = False

    # ── Stream / camera management ────────────────────────────────────────

    def open(self):
        """Open the video source (idempotent)."""
        if self._opened:
            return

        if self._stream_addr:
            # ZMQ subscriber to Pi's camera stream
            self._zmq_ctx = zmq.Context()
            self._zmq_sock = self._zmq_ctx.socket(zmq.SUB)
            self._zmq_sock.setsockopt(zmq.CONFLATE, 1)  # only keep latest frame
            self._zmq_sock.setsockopt_string(zmq.SUBSCRIBE, "")
            self._zmq_sock.setsockopt(zmq.RCVTIMEO, 300)   # 300ms timeout — fast retry
            self._zmq_sock.connect(self._stream_addr)
            print(f"[detect] connected to ZMQ stream: {self._stream_addr}")
            # Let connection establish
            time.sleep(0.5)
        else:
            # Local camera fallback
            self._cap = cv2.VideoCapture(self._cam_idx)
            if not self._cap.isOpened():
                raise RuntimeError(f"Cannot open local camera {self._cam_idx}")
            print(f"[detect] opened local camera {self._cam_idx}")

        self._opened = True

    def _grab_frame(self) -> np.ndarray | None:
        """Grab a single frame from ZMQ or local camera."""
        if self._stream_addr and self._zmq_sock:
            try:
                jpg_bytes = self._zmq_sock.recv()
                frame = cv2.imdecode(
                    np.frombuffer(jpg_bytes, dtype=np.uint8),
                    cv2.IMREAD_COLOR
                )
                if frame is not None:
                    # picamera2 captures in RGB order despite "BGR888" config,
                    # so the JPEG has swapped R/B channels. Fix it here.
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                return frame
            except zmq.Again:
                return None  # frame missed — caller retries immediately
        elif self._cap:
            ret, frame = self._cap.read()
            return frame if ret else None
        return None

    def release(self):
        """Release all resources."""
        if self._zmq_sock:
            self._zmq_sock.close()
            self._zmq_sock = None
        if self._zmq_ctx:
            self._zmq_ctx.term()
            self._zmq_ctx = None
        if self._cap:
            self._cap.release()
            self._cap = None
        self._opened = False
        print("[detect] released")

    # ── Core detection ────────────────────────────────────────────────────

    def _build_mask(self, hsv_frame: np.ndarray) -> np.ndarray:
        mask = np.zeros(hsv_frame.shape[:2], dtype=np.uint8)
        for lower, upper in self._ranges:
            mask |= cv2.inRange(hsv_frame, lower, upper)

        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE)
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=1)
        return mask

    def detect_once(self) -> dict:
        """
        Grab one frame, run detection, return results.

        Returns dict: {found, cx, cy, area, frame_w, frame_h, contour, frame, mask}
        """
        self.open()

        frame = self._grab_frame()
        if frame is None:
            return self._empty_result()

        h, w = frame.shape[:2]
        blurred = cv2.GaussianBlur(frame, BLUR_KERNEL, 0)
        hsv     = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask    = self._build_mask(hsv)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            return {
                "found": False, "cx": 0, "cy": 0, "area": 0,
                "frame_w": w, "frame_h": h,
                "contour": None, "frame": frame, "mask": mask,
            }

        largest = max(contours, key=cv2.contourArea)
        area = int(cv2.contourArea(largest))

        if area < MIN_CONTOUR_AREA:
            return {
                "found": False, "cx": 0, "cy": 0, "area": area,
                "frame_w": w, "frame_h": h,
                "contour": None, "frame": frame, "mask": mask,
            }

        M = cv2.moments(largest)
        cx = int(M["m10"] / M["m00"]) if M["m00"] > 0 else w // 2
        cy = int(M["m01"] / M["m00"]) if M["m00"] > 0 else h // 2

        return {
            "found": True,
            "cx": cx, "cy": cy, "area": area,
            "frame_w": w, "frame_h": h,
            "contour": largest, "frame": frame, "mask": mask,
        }

    def _empty_result(self) -> dict:
        return {
            "found": False, "cx": 0, "cy": 0, "area": 0,
            "frame_w": 0, "frame_h": 0,
            "contour": None, "frame": None, "mask": None,
        }

    # ── Live detection with display ───────────────────────────────────────

    def run_live(self, show_window: bool = True, duration: float = 0):
        """
        Continuously detect and display results.

        Shows an OpenCV window with:
          - Green contour outline around the detected object
          - Yellow dot at the object centre
          - White centre line for alignment reference
          - Offset arrow from centre to object
        """
        self.open()
        start = time.time()
        frame_count = 0

        print(f"[detect] live mode — colour={self._color}  "
              f"window={'ON' if show_window else 'OFF'}  "
              f"duration={'∞' if duration == 0 else f'{duration}s'}")
        print("  Press 'q' in window or Ctrl-C to stop\n")

        try:
            while True:
                det = self.detect_once()
                frame_count += 1

                # Console output
                if det["found"]:
                    off = det["cx"] - det["frame_w"] // 2
                    side = "LEFT" if off < 0 else "RIGHT" if off > 0 else "CENTER"
                    print(
                        f"  FOUND  cx={det['cx']:4d}  cy={det['cy']:4d}  "
                        f"area={det['area']:6d}  offset={off:+4d}px  ({side})"
                    )
                else:
                    if frame_count % 10 == 0:
                        print(f"  --  nothing  (area={det['area']})")

                # Display window
                if show_window and det["frame"] is not None:
                    vis = det["frame"].copy()
                    h, w = vis.shape[:2]
                    cw = w // 2

                    # Centre reference line
                    cv2.line(vis, (cw, 0), (cw, h), (255, 255, 255), 1)

                    # Dead zone lines
                    dz = 40
                    cv2.line(vis, (cw - dz, 0), (cw - dz, h), (100, 100, 100), 1)
                    cv2.line(vis, (cw + dz, 0), (cw + dz, h), (100, 100, 100), 1)

                    if det["contour"] is not None:
                        # Contour + bounding rect
                        cv2.drawContours(vis, [det["contour"]], -1, (0, 255, 0), 2)
                        x, y, bw, bh = cv2.boundingRect(det["contour"])
                        cv2.rectangle(vis, (x, y), (x + bw, y + bh), (0, 255, 0), 1)

                        # Centre dot
                        cv2.circle(vis, (det["cx"], det["cy"]), 8, (0, 255, 255), -1)
                        cv2.circle(vis, (det["cx"], det["cy"]), 8, (0, 200, 200), 2)

                        # Offset arrow
                        cv2.arrowedLine(vis, (cw, det["cy"]),
                                        (det["cx"], det["cy"]),
                                        (0, 200, 255), 2, tipLength=0.3)

                        # Text info
                        off = det["cx"] - cw
                        label = f"offset: {off:+d}px  area: {det['area']}"
                        cv2.putText(vis, label, (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                                    (0, 255, 0), 2)
                    else:
                        cv2.putText(vis, "NO DETECTION", (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                                    (0, 0, 255), 2)

                    # Color label
                    cv2.putText(vis, f"color: {self._color}", (10, h - 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (200, 200, 200), 1)

                    cv2.imshow("Color Detection", vis)

                    # Also show the mask
                    if det["mask"] is not None:
                        cv2.imshow("Mask", det["mask"])

                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break

                if duration > 0 and (time.time() - start) > duration:
                    break

                time.sleep(0.02)

        except KeyboardInterrupt:
            print("\n[detect] stopped")
        finally:
            elapsed = time.time() - start
            fps = frame_count / elapsed if elapsed > 0 else 0
            print(f"[detect] {frame_count} frames in {elapsed:.1f}s ({fps:.1f} fps)")
            if show_window:
                cv2.destroyAllWindows()


# ── CLI ───────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Test colour detection from Pi camera stream."
    )
    parser.add_argument(
        "--color", default="blue",
        help=f"Target colour ({', '.join(COLOR_PRESETS.keys())})"
    )
    parser.add_argument(
        "--camera", type=int, default=None,
        help="Use local webcam instead of Pi stream (for testing)"
    )
    parser.add_argument(
        "--no-window", action="store_true",
        help="Disable display window (headless mode)"
    )
    parser.add_argument(
        "--duration", type=float, default=0,
        help="Run for N seconds then stop (0 = forever)"
    )
    args = parser.parse_args()

    # Local camera overrides ZMQ stream
    stream = None if args.camera is not None else STREAM_ADDR

    detector = ColorDetector(
        stream_addr=stream,
        camera_index=args.camera or 0,
        target_color=args.color,
    )
    try:
        detector.run_live(
            show_window=not args.no_window,
            duration=args.duration,
        )
    finally:
        detector.release()


if __name__ == "__main__":
    main()
