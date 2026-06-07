"""
camera_stream.py — MJPEG HTTP stream server for the Pi camera.

Run this ON THE RASPBERRY PI. It opens the camera and serves a live
MJPEG stream that the PC can read with OpenCV or view in a browser.

Usage (on Pi):
    python3 camera_stream.py                    # default: camera 0, port 8081
    python3 camera_stream.py --port 8081        # custom port
    python3 camera_stream.py --camera 0         # camera index

Then on your PC, open in browser:
    http://10.213.37.191:8081/stream

Or read in OpenCV:
    cap = cv2.VideoCapture("http://10.213.37.191:8081/stream")
"""

import argparse
import threading
import time
from http.server import HTTPServer, BaseHTTPRequestHandler

import cv2

# ── Config ────────────────────────────────────────────────────────────────────
DEFAULT_PORT   = 8081
DEFAULT_CAMERA = 0
RESOLUTION     = (640, 480)
JPEG_QUALITY   = 70       # lower = smaller frames = less lag
FPS_TARGET     = 20       # target frames per second


class CameraCapture:
    """Thread-safe camera capture with latest-frame buffer."""

    def __init__(self, camera_index: int = 0):
        self._cap = cv2.VideoCapture(camera_index)
        if not self._cap.isOpened():
            raise RuntimeError(f"Cannot open camera {camera_index}")
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  RESOLUTION[0])
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, RESOLUTION[1])

        self._frame = None
        self._lock  = threading.Lock()
        self._running = True

        w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"[stream] camera {camera_index} opened ({w}x{h})")

        # Background capture thread — always grabs the latest frame
        threading.Thread(target=self._capture_loop, daemon=True).start()

    def _capture_loop(self):
        while self._running:
            ret, frame = self._cap.read()
            if ret:
                with self._lock:
                    self._frame = frame
            time.sleep(1.0 / FPS_TARGET)

    def get_jpeg(self) -> bytes | None:
        with self._lock:
            if self._frame is None:
                return None
            _, jpeg = cv2.imencode(
                '.jpg', self._frame,
                [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]
            )
            return jpeg.tobytes()

    def stop(self):
        self._running = False
        self._cap.release()


# Global camera instance (set in main)
camera: CameraCapture | None = None


class StreamHandler(BaseHTTPRequestHandler):
    """Serves MJPEG stream on /stream and a simple status page on /."""

    def do_GET(self):
        if self.path == '/stream':
            self._serve_mjpeg()
        elif self.path == '/':
            self._serve_index()
        else:
            self.send_error(404)

    def _serve_mjpeg(self):
        self.send_response(200)
        self.send_header('Content-Type',
                         'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()

        while True:
            jpeg = camera.get_jpeg()
            if jpeg is None:
                time.sleep(0.05)
                continue
            try:
                self.wfile.write(b'--frame\r\n')
                self.wfile.write(b'Content-Type: image/jpeg\r\n')
                self.wfile.write(f'Content-Length: {len(jpeg)}\r\n'.encode())
                self.wfile.write(b'\r\n')
                self.wfile.write(jpeg)
                self.wfile.write(b'\r\n')
            except (BrokenPipeError, ConnectionResetError):
                break
            time.sleep(1.0 / FPS_TARGET)

    def _serve_index(self):
        html = (
            '<html><body style="background:#111;color:#eee;font-family:monospace">'
            '<h2>Pi Camera Stream</h2>'
            '<img src="/stream" style="max-width:100%">'
            '</body></html>'
        )
        self.send_response(200)
        self.send_header('Content-Type', 'text/html')
        self.end_headers()
        self.wfile.write(html.encode())

    def log_message(self, format, *args):
        # Suppress per-request logs to keep terminal clean
        pass


def main():
    global camera

    parser = argparse.ArgumentParser(description="Pi camera MJPEG stream server")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--camera", type=int, default=DEFAULT_CAMERA)
    args = parser.parse_args()

    camera = CameraCapture(args.camera)

    server = HTTPServer(('0.0.0.0', args.port), StreamHandler)
    print(f"[stream] serving on http://0.0.0.0:{args.port}/stream")
    print(f"[stream] open in browser: http://<pi-ip>:{args.port}/")
    print("[stream] Ctrl-C to stop\n")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[stream] shutting down")
    finally:
        camera.stop()
        server.server_close()


if __name__ == "__main__":
    main()
