# testOmniv2_clean.py
#ver2 by Daada python3.6
import os
os.environ["OPENBLAS_CORETYPE"] = "ARMV8"
os.environ["OMP_NUM_THREADS"] = "1"

import queue
import threading
import time

import cv2
import numpy as np
import torch
from ultralytics import YOLO

from kinematics import Kinematics
from serial_ import ArduinoSerial
from controller import PID

# ==================== CONFIG ====================
DRY_RUN         = False
CAM_W, CAM_H    = 640, 480
IMGZ            = 736
ORIGINAL_WIDTH  = 640          # width K/dist were calibrated on
CONF_THRESHOLD  = 0.01
MODEL_PATH      = "yolov8n_736.engine"
VIDEO_PATH      = "0"

CAMERA_HEIGHT   = 0.20
CAMERA_PITCH    = 0.0
R               = 0.135
r_w             = 0.04

DESIRED_X       = 0.0
DESIRED_Z       = 0.8

Kp_x, Kp_z = 0.8, 0.8
DT              = 0.05

PRINT_EVERY_N_FRAMES = 30      # FPS / frame counter console print interval
SHOW_WINDOW     = True         # side-by-side: raw | detection

# Hardcoded camera calibration
K = np.array([[793.80733303,   0.0,         369.37028769],
              [  0.0,        808.39903635, 133.02983411],
              [  0.0,          0.0,          1.0        ]])
dist = np.array([[-4.50528078e-01, 1.40605383e+00, -1.58616084e-02,
                   1.18104384e-03, -1.97623221e+00]])

# Latest ball pixel; maxsize=1 means control always consumes the newest frame.
coordinate_queue = queue.Queue(maxsize=1)


# ==================== DRY-RUN STUB ====================
class FakeArduinoSerial:
    def __init__(self, *args, **kwargs):
        self._connected = False
        self._count = 0

    def open(self):
        self._connected = True
        print("[DRY-RUN] serial opened")
        return True

    def close(self):
        self._connected = False
        print("[DRY-RUN] serial closed, {} messages sent".format(self._count))

    def send(self, data, append_newline=True):
        if isinstance(data, bytes):
            data = data.decode()
        self._count += 1
        if self._count % 20 == 0:
            print("[DRY-RUN #{:05d}] {}".format(self._count, data.strip()))
        return True

    @property
    def is_connected(self):
        return self._connected


# ==================== ROTATION / PROJECTION ====================
def rotate_frame(frame):
    """Match the rotation the model was trained on."""
    return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)


def unrotate_pixel(u_rot, v_rot, W=ORIGINAL_WIDTH):
    """Inverse of ROTATE_90_COUNTERCLOCKWISE: (u,v) -> (v, W-1-u)."""
    return W - 1 - v_rot, u_rot


class CameraProjection:
    def __init__(self, K, dist, height, pitch=0.0):
        self.K, self.dist, self.H = K, dist, height
        R_y_flip = np.array([[1, 0, 0],
                             [0, -1, 0],
                             [0,  0, 1]])
        R_pitch = np.array([[1, 0, 0],
                            [0, np.cos(pitch), -np.sin(pitch)],
                            [0, np.sin(pitch),  np.cos(pitch)]])
        self.R = R_pitch @ R_y_flip

    def pixel_to_ground(self, u, v):
        pts = np.array([[[u, v]]], dtype=np.float32)
        xn, yn = cv2.undistortPoints(pts, self.K, self.dist,
                                     P=np.eye(3, dtype=np.float32))[0, 0]
        ray = self.R @ np.array([xn, yn, 1.0])
        if ray[1] >= 0 or abs(ray[1]) < 1e-9:
            return None
        t = -self.H / ray[1]
        if t < 0:
            return None
        return t * ray[0], t * ray[2]   # (X = lateral, Z = forward)


# ==================== DETECTOR ====================
def detect_ball(model, frame, device):
    """Returns (cx, cy) or None, plus an annotated copy of frame."""
    annotated = frame.copy()
    res = model(frame, conf=CONF_THRESHOLD, device=device, imgsz=IMGZ,
                max_det=1, verbose=False, classes=[32])[0]
    boxes = res.boxes
    if boxes is None or len(boxes) == 0:
        return None, annotated

    x1, y1, x2, y2 = map(int, boxes[0].xyxy[0].cpu().numpy())
    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
    conf = float(boxes[0].conf[0])
    cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 255), 2)
    cv2.drawMarker(annotated, (cx, cy), (0, 255, 255), cv2.MARKER_CROSS, 20, 2)
    cv2.putText(annotated, "conf {:.2f}".format(conf),
                (x1, max(y1 - 8, 15)), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 255, 255), 1)
    return (cx, cy), annotated


# ==================== CONTROL ====================
def control_loop(stop_event, arduino, kin, proj):
    pid_x = PID(Kp_x, 0, 0, setpoint=DESIRED_X, output_limits=(-0.5, 0.5))
    pid_z = PID(Kp_z, 0, 0, setpoint=DESIRED_Z, output_limits=(-0.5, 0.5))

    last_zero = 0.0
    while not stop_event.is_set():
        try:
            coord = coordinate_queue.get(timeout=0.1)
        except queue.Empty:
            now = time.time()
            if now - last_zero > 1.0:
                arduino.send("0.00,0.00,0.00")
                last_zero = now
            continue

        u_orig, v_orig = unrotate_pixel(*coord)
        ground = proj.pixel_to_ground(u_orig, v_orig)
        if ground is None:
            continue

        ball_x, ball_z = ground
        err_x = DESIRED_X - ball_x
        err_z = DESIRED_Z - ball_z

        lat_cmd = pid_x.compute(error=err_x, dt=DT)
        fwd_cmd = pid_z.compute(error=err_z, dt=DT)
        omega   = 0.0

        # kin.compute_wheel_rpm(vx=forward, vy=left, omega)
        # Our frame: +X = lateral right, +Z = forward.
        #   fwd_cmd > 0 -> back up        -> vx_kin = -fwd_cmd
        #   lat_cmd > 0 -> move toward +X -> vy_kin =  lat_cmd
        rpm = kin.compute_wheel_rpm(-fwd_cmd, lat_cmd, omega)
        arduino.send("{:.2f},{:.2f},{:.2f}".format(*rpm))


# ==================== MAIN ====================
def main():
    # --- Hardware ---
    arduino = FakeArduinoSerial() if DRY_RUN else ArduinoSerial(baudrate=115200)
    if not arduino.open():
        print("Failed to open serial port.")
        return
    time.sleep(2.0)

    kin  = Kinematics(R, r_w)
    proj = CameraProjection(K, dist, CAMERA_HEIGHT, CAMERA_PITCH)

    # --- Model ---
    device = "0" if torch.cuda.is_available() else "cpu"
    print("Device:", device.upper(), "| model:", MODEL_PATH)
    model = YOLO(MODEL_PATH, task="detect")

    # --- Camera ---
    src = int(VIDEO_PATH) if VIDEO_PATH.isdigit() else VIDEO_PATH
    cap = cv2.VideoCapture(src)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAM_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        print("Cannot open camera.")
        arduino.close()
        return

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print("Camera resolution: {}x{}".format(actual_w, actual_h))

    # --- Control thread ---
    stop_event = threading.Event()
    ctrl_thread = threading.Thread(
        target=control_loop,
        args=(stop_event, arduino, kin, proj),
        daemon=True,
    )
    ctrl_thread.start()

    # --- Detection loop (main thread) ---
    if SHOW_WINDOW:
        cv2.namedWindow("raw | detection", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("raw | detection", 1280, 480)

    frame_count = 0
    fps_t0 = time.time()
    read_fails = 0

    try:
        while not stop_event.is_set():
            ret, raw = cap.read()
            if not ret:
                read_fails += 1
                if read_fails >= 60:
                    print("[cam] camera lost after {} failed reads, exiting".format(read_fails))
                    stop_event.set()
                    break
                time.sleep(0.05)
                continue
            read_fails = 0

            frame_count += 1

            # What the model actually sees (rotated).
            rotated = rotate_frame(raw)
            coords, annotated = detect_ball(model, rotated, device)

            if coords and not coordinate_queue.full():
                coordinate_queue.put_nowait(coords)

            # FPS / counter print
            if frame_count % PRINT_EVERY_N_FRAMES == 0:
                dt = time.time() - fps_t0
                fps = PRINT_EVERY_N_FRAMES / dt if dt > 0 else 0.0
                print("frame={:6d}  fps={:.2f}  ball={}".format(
                    frame_count, fps, coords))
                fps_t0 = time.time()

            # Overlay frame counter + FPS onto both panes
            elapsed = time.time() - fps_t0
            live_fps = (frame_count % PRINT_EVERY_N_FRAMES) / elapsed if elapsed > 0 else 0.0
            tag = "frame {}  ({:.1f} fps)".format(frame_count, live_fps)
            cv2.putText(raw,       "RAW   " + tag, (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(annotated, "YOLO  " + tag, (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            if SHOW_WINDOW:
                # Match heights so hstack works regardless of rotation.
                h = min(raw.shape[0], annotated.shape[0])
                left  = cv2.resize(raw,       (int(raw.shape[1]       * h / raw.shape[0]),       h))
                right = cv2.resize(annotated, (int(annotated.shape[1] * h / annotated.shape[0]), h))
                combo = np.hstack([left, right])
                cv2.imshow("raw | detection", combo)
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    stop_event.set()

    except KeyboardInterrupt:
        stop_event.set()
    finally:
        # 1) signal, 2) wait for control thread, 3) release resources
        stop_event.set()
        ctrl_thread.join(timeout=1.0)
        cap.release()
        if SHOW_WINDOW:
            cv2.destroyAllWindows()
        arduino.close()
        print("Shut down. Total frames processed: {}".format(frame_count))


if __name__ == "__main__":
    main()
