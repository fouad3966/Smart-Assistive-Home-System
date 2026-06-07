import numpy as np
import subprocess
import cv2

COLOR_RANGES = {
    "red": [
        (np.array([0,   100, 50]),  np.array([10,  255, 255])),
        (np.array([160, 100, 50]),  np.array([179, 255, 255])),
    ],
    "black": [
        (np.array([0, 0, 0]),       np.array([179, 100, 50])),
    ],
    "white": [
        (np.array([0, 0, 168]),     np.array([179, 60,  255])),
    ],
}

MIN_CONTOUR_AREA = 800

def detect_color(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hsv = cv2.GaussianBlur(hsv, (7, 7), 0)

    best_color = None
    best_area = 0
    best_box = None

    for color_name, ranges in COLOR_RANGES.items():
        combined_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for (lower, upper) in ranges:
            mask = cv2.inRange(hsv, lower, upper)
            combined_mask = cv2.bitwise_or(combined_mask, mask)

        kernel = np.ones((5, 5), np.uint8)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < MIN_CONTOUR_AREA:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            aspect_ratio = w / float(h)
            if not (0.4 < aspect_ratio < 2.5):
                continue

            if area > best_area:
                best_area = area
                best_color = color_name
                best_box = (x, y, w, h)

    return best_color, best_box


def main():
    cmd = [
        "libcamera-vid",
        "-t", "0",
        "--codec", "mjpeg",
        "-o", "-",
        "--width", "320",
        "--height", "240",
        "--framerate", "15",
        "--inline",
        "--nopreview"
    ]

    process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)

    print("Detection running. Press Ctrl+C to stop.")

    buf = b""
    frame_count = 0

    try:
        while True:
            buf += process.stdout.read(65536)  # read larger chunks

            # Process all complete frames in buffer
            while True:
                start = buf.find(b'\xff\xd8')
                end   = buf.find(b'\xff\xd9', start+2) if start != -1 else -1

                if start == -1 or end == -1:
                    break

                jpg = buf[start:end+2]
                buf = buf[end+2:]

                frame_count += 1
                if frame_count % 3 != 0:  # process every 3rd frame
                    continue

                frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is None:
                    continue

                color, box = detect_color(frame)

                if color and box:
                    x, y, w, h = box
                    print(f"DETECTED: {color.upper()} at x={x} y={y} w={w} h={h}")
                else:
                    print("No object detected")

    except KeyboardInterrupt:
        print("\nStopping.")
        process.terminate()

if __name__ == "__main__":
    main()