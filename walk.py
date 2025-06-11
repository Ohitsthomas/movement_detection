import cv2
import time
import hmac
import hashlib
import json
import requests
from collections import deque
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from edge_impulse_linux.runner import ImpulseRunner  # For non-image models

# ======== Edge Impulse Model Setup ========
MODEL_PATH = './walk.eim'  # <-- Change this to your actual model file
runner = ImpulseRunner(MODEL_PATH)
model_info = runner.init()
labels = model_info['model_parameters']['labels']
print(f"Model loaded: {model_info['project']['owner']} / {model_info['project']['name']}")

# ======== MediaPipe Pose Setup ========
base_opts = python.BaseOptions(model_asset_path='pose_landmarker_heavy.task')
options = vision.PoseLandmarkerOptions(
    base_options=base_opts,
    running_mode=vision.RunningMode.VIDEO
)
detector = vision.PoseLandmarker.create_from_options(options)

# ======== Buffering Setup ========
RECORD_DURATION = 2.0  # seconds
buffer = deque()
recording_start = None
frame_times = deque(maxlen=30)  # For FPS estimation

def main():
    global recording_start, buffer
    cap = cv2.VideoCapture(0)

    while True:
        success, frame = cap.read()
        if not success:
            break

        now = time.time()
        ts = int(now * 1000)
        frame_times.append(now)

        # Estimate FPS
        if len(frame_times) > 1:
            duration = frame_times[-1] - frame_times[0]
            fps = len(frame_times) / duration if duration > 0 else 30
        else:
            fps = 30
        interval_ms = int(1000 / fps)

        # Pose Detection
        mp_img = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame)
        result = detector.detect_for_video(mp_img, ts)

        if result.pose_landmarks:
            lm = result.pose_landmarks[0]
            h, w, _ = frame.shape
            l_x, l_y = int(lm[31].x * w), int(lm[31].y * h)
            r_x, r_y = int(lm[32].x * w), int(lm[32].y * h)

            if all(0 < v < max(w, h) for v in (l_x, l_y, r_x, r_y)):
                if recording_start is None:
                    recording_start = now
                    buffer.clear()

                if now - recording_start <= RECORD_DURATION:
                    buffer.append([l_x, l_y, r_x, r_y])
                else:
                    # ======= Classification ========
                    features = sum(buffer, [])  # Flatten list of lists

                    try:
                        result = runner.classify(features)
                        if "classification" in result["result"]:
                            print("\n=== Movement Detected ===")
                            for label in labels:
                                score = result["result"]["classification"].get(label, 0.0)
                                print(f"{label}: {score:.2f}")
                            print("=========================\n")
                        else:
                            print("No classification result found.")
                    except Exception as e:
                        print(f"Classification failed: {e}")

                    recording_start = None

        # Show live feed
        cv2.imshow("Live Foot Tracker", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    runner.stop()

if __name__ == "__main__":
    main()
