#!/usr/bin/env python3

import serial
import time
import math
import requests
import threading
import queue
from flask import Flask, request, jsonify

app = Flask(__name__)

PORT = "/dev/ttyUSB0"
BAUD = 115200

SERVER_URL = "http://127.0.0.1:5000/update"

WHEEL_BASE_M = 0.35
WHEEL_DIAMETER_MM = 90.0

MOTOR_STEPS_PER_REV = 200
MICROSTEP = 16
GEAR_RATIO = 1.0
AXIS_STEPS_PER_MM = 80.0

X_SIGN = -1.0
Y_SIGN = 1.0
Z_SIGN = 1.0
E_SIGN = 1.0

MAX_V = 0.30
MAX_OMEGA = 1.50
MAX_TIME = 1.0

ROT_EPS_MM = 0.20

ENABLE_SERVER = True
REQUEST_TIMEOUT = 0.05

x_pos = 0.0
y_pos = 0.0
theta = 0.0

motion_lock = threading.Lock()

TOTAL_STEPS_PER_REV = MOTOR_STEPS_PER_REV * MICROSTEP * GEAR_RATIO
WHEEL_CIRCUMFERENCE_MM = math.pi * WHEEL_DIAMETER_MM

CMD_MM_PER_METER = (
    1000.0 * TOTAL_STEPS_PER_REV / (AXIS_STEPS_PER_MM * WHEEL_CIRCUMFERENCE_MM)
)

print("CMD_MM_PER_METER =", round(CMD_MM_PER_METER, 3))

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2.0)


def send(cmd, timeout=3.0):
    print(">>", cmd)
    ser.write((cmd + "\n").encode())
    ser.flush()

    start = time.time()

    while time.time() - start < timeout:
        line = ser.readline().decode(errors="ignore").strip()

        if line:
            print("<<", line)
            if line.lower().startswith("ok"):
                return True

    print(f"[WARN] timeout dla komendy: {cmd}")
    return False


def wait_finish():
    send("M400", timeout=20.0)


server_queue = queue.Queue(maxsize=1)
server_stop = threading.Event()


def server_worker():
    session = requests.Session()

    while not server_stop.is_set():
        try:
            payload = server_queue.get(timeout=0.2)
        except queue.Empty:
            continue

        while True:
            try:
                payload = server_queue.get_nowait()
            except queue.Empty:
                break

        try:
            session.post(SERVER_URL, json=payload, timeout=REQUEST_TIMEOUT)
        except Exception:
            pass


def send_position_to_server(status_text="ok", v=0.0, omega=0.0):
    if not ENABLE_SERVER:
        return

    payload = {
        "x": round(x_pos, 4),
        "y": round(y_pos, 4),
        "theta": round(theta, 4),
        "status": status_text,
        "v": round(v, 4),
        "omega": round(omega, 4),
    }

    try:
        server_queue.put_nowait(payload)
    except queue.Full:
        try:
            server_queue.get_nowait()
        except queue.Empty:
            pass
        try:
            server_queue.put_nowait(payload)
        except queue.Full:
            pass


def init_marlin():
    send("M115", timeout=3.0)
    send("G91", timeout=3.0)
    send("M83", timeout=3.0)
    send("M17", timeout=3.0)
    send("M121", timeout=3.0)
    send("M211 S0", timeout=3.0)
    send("M302 S0", timeout=3.0)

    send(
        f"M92 X{AXIS_STEPS_PER_MM} Y{AXIS_STEPS_PER_MM} "
        f"Z{AXIS_STEPS_PER_MM} E{AXIS_STEPS_PER_MM}",
        timeout=3.0,
    )

    send("M203 X50 Y50 Z50 E50", timeout=3.0)
    send("M201 X80 Y80 Z80 E80", timeout=3.0)
    send("M204 P120 T120", timeout=3.0)
    send("M205 X5 Y5 Z5 E5", timeout=3.0)

    send("G92 X0 Y0 Z0 E0", timeout=3.0)
    send("M114", timeout=3.0)
    send("M119", timeout=3.0)
    send("G4 P100", timeout=3.0)


def normalize_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def update_odometry(v, omega, duration):
    global x_pos, y_pos, theta

    if abs(omega) < 1e-9:
        ds = v * duration
        x_pos += ds * math.cos(theta)
        y_pos += ds * math.sin(theta)
        theta = normalize_angle(theta)
        return

    theta_new = theta + omega * duration

    x_pos += (v / omega) * (math.sin(theta_new) - math.sin(theta))
    y_pos += -(v / omega) * (math.cos(theta_new) - math.cos(theta))
    theta = normalize_angle(theta_new)


def move_wheels_cmd_mm(left_cmd_mm, right_cmd_mm, duration):
    if duration <= 0:
        return

    max_axis_speed_mm_s = max(
        abs(left_cmd_mm / duration),
        abs(right_cmd_mm / duration),
        1e-6,
    )

    feedrate = max_axis_speed_mm_s * 60.0

    x_cmd = X_SIGN * left_cmd_mm
    z_cmd = Z_SIGN * left_cmd_mm
    y_cmd = Y_SIGN * right_cmd_mm
    e_cmd = E_SIGN * right_cmd_mm

    cmd = (
        f"G1 "
        f"X{x_cmd:.3f} Z{z_cmd:.3f} "
        f"Y{y_cmd:.3f} E{e_cmd:.3f} "
        f"F{feedrate:.1f}"
    )

    send(cmd, timeout=5.0)
    wait_finish()


def rotate_only(omega, duration):
    global theta

    omega = max(min(omega, MAX_OMEGA), -MAX_OMEGA)
    duration = min(duration, MAX_TIME)

    side_distance_m = (omega * WHEEL_BASE_M / 2.0) * duration

    left_cmd_mm = -side_distance_m * CMD_MM_PER_METER
    right_cmd_mm = side_distance_m * CMD_MM_PER_METER

    x_base = left_cmd_mm
    z_base = left_cmd_mm + (ROT_EPS_MM if left_cmd_mm >= 0 else -ROT_EPS_MM)

    y_base = right_cmd_mm
    e_base = right_cmd_mm + (ROT_EPS_MM if right_cmd_mm >= 0 else -ROT_EPS_MM)

    max_axis_speed_mm_s = max(
        abs(left_cmd_mm / duration),
        abs(right_cmd_mm / duration),
        1e-6,
    )

    feedrate = max_axis_speed_mm_s * 60.0

    cmd = (
        f"G1 "
        f"X{X_SIGN * x_base:.3f} Z{Z_SIGN * z_base:.3f} "
        f"Y{Y_SIGN * y_base:.3f} E{E_SIGN * e_base:.3f} "
        f"F{feedrate:.1f}"
    )

    send(cmd, timeout=5.0)
    wait_finish()

    theta = normalize_angle(theta + omega * duration)
    send_position_to_server(status_text="obrót", v=0.0, omega=omega)


def move_cmd_vel(v, omega, duration):
    v = max(min(v, MAX_V), -MAX_V)
    omega = max(min(omega, MAX_OMEGA), -MAX_OMEGA)
    duration = min(max(duration, 0.01), MAX_TIME)

    if abs(v) < 1e-6 and abs(omega) < 1e-6:
        send_position_to_server(status_text="stop", v=0.0, omega=0.0)
        return

    if abs(v) < 1e-6 and abs(omega) > 1e-6:
        rotate_only(omega, duration)
        return

    v_left = v - (omega * WHEEL_BASE_M / 2.0)
    v_right = v + (omega * WHEEL_BASE_M / 2.0)

    s_left_m = v_left * duration
    s_right_m = v_right * duration

    left_cmd_mm = s_left_m * CMD_MM_PER_METER
    right_cmd_mm = s_right_m * CMD_MM_PER_METER

    move_wheels_cmd_mm(left_cmd_mm, right_cmd_mm, duration)
    update_odometry(v, omega, duration)
    send_position_to_server(status_text="ruch", v=v, omega=omega)


@app.route("/")
def home():
    return jsonify(
        {
            "status": "ok",
            "message": "RPi Flask Marlin driver działa",
            "x": x_pos,
            "y": y_pos,
            "theta": theta,
        }
    )


@app.route("/cmd_vel", methods=["POST"])
def cmd_vel_endpoint():
    data = request.get_json(silent=True)

    if data is None:
        return jsonify({"status": "error", "message": "Brak JSON"}), 400

    v = float(data.get("v", 0.0))
    omega = float(data.get("omega", 0.0))
    duration = float(data.get("duration", 0.25))

    if not motion_lock.acquire(blocking=False):
        return (
            jsonify(
                {
                    "status": "busy",
                    "message": "Robot wykonuje poprzedni ruch",
                }
            ),
            429,
        )

    try:
        move_cmd_vel(v, omega, duration)
    finally:
        motion_lock.release()

    return jsonify(
        {
            "status": "ok",
            "v": v,
            "omega": omega,
            "duration": duration,
            "x": x_pos,
            "y": y_pos,
            "theta": theta,
        }
    )


@app.route("/stop", methods=["POST"])
def stop_endpoint():
    send_position_to_server(status_text="stop", v=0.0, omega=0.0)
    return jsonify({"status": "ok", "message": "stop"})


@app.route("/state")
def state_endpoint():
    return jsonify(
        {
            "x": x_pos,
            "y": y_pos,
            "theta": theta,
        }
    )


if __name__ == "__main__":
    try:
        server_thread = threading.Thread(target=server_worker, daemon=True)

        if ENABLE_SERVER:
            server_thread.start()

        init_marlin()
        send_position_to_server(status_text="start", v=0.0, omega=0.0)

        print("Flask driver startuje na 0.0.0.0:5001")
        app.run(host="0.0.0.0", port=5001, threaded=True)

    finally:
        server_stop.set()
        try:
            ser.close()
        except Exception:
            pass
