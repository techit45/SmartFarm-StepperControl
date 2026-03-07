#!/usr/bin/env python3
"""
🌱 Smart Farm Web Server
Flask server + Camera Stream + Arduino Control + Leaf Analysis
"""

import cv2
import numpy as np
import serial
import serial.tools.list_ports
import argparse
import time
import os
import json
from datetime import datetime
from threading import Thread, Lock, Event
from flask import Flask, render_template, Response, jsonify, request

# ========================================
# Config
# ========================================
HSV_LOW = np.array([25, 40, 40])
HSV_HIGH = np.array([95, 255, 255])
MIN_CONTOUR_AREA = 500
NUM_POTS = 2

# Flask — ใช้ path ของไฟล์นี้เป็นฐาน (ไม่ขึ้นกับ CWD)
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
app = Flask(__name__,
            static_folder=os.path.join(BASE_DIR, 'static'),
            template_folder=os.path.join(BASE_DIR, 'templates'))

# ========================================
# Global State
# ========================================
class AppState:
    def __init__(self):
        self.lock = Lock()
        self.cap = None
        self.arduino = None
        self.serial_connected = False
        self.motor_busy = False
        self.motor_status = ""
        self.is_calibrated = False
        self.total_steps = 0
        self.current_position = 0
        self.pot_positions = []
        self.current_pot = None
        self.is_scanning = False
        self.scan_history = []
        self.last_analysis = None
        self.latest_frame = None
        self.hsv_low = HSV_LOW.copy()
        self.hsv_high = HSV_HIGH.copy()
        self.save_dir = "scan_results"

state = AppState()


# ========================================
# Arduino Controller
# ========================================
class ArduinoController:
    def __init__(self, port=None, baudrate=115200):
        self.ser = None
        self.port = port
        self.baudrate = baudrate
        self.lock = Lock()

    def find_port(self):
        ports = serial.tools.list_ports.comports()
        for p in ports:
            desc = p.description.upper()
            if any(k in desc for k in ['USB', 'UART', 'CP210', 'CH340', 'SERIAL']):
                return p.device
        for p in ports:
            if 'cu.usb' in p.device or 'ttyUSB' in p.device:
                return p.device
        return None

    def connect(self):
        port = self.port or self.find_port()
        if not port:
            print("❌ No Arduino port found")
            return False
        try:
            self.ser = serial.Serial(port, self.baudrate, timeout=2)
            time.sleep(2)
            self._read_all()
            print(f"✅ Serial: {port}")
            return True
        except Exception as e:
            print(f"❌ Serial error: {e}")
            return False

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def _read_all(self, timeout=0.5):
        lines = []
        end_time = time.time() + timeout
        while time.time() < end_time:
            if self.ser and self.ser.in_waiting > 0:
                try:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        lines.append(line)
                        end_time = time.time() + 0.3
                except:
                    pass
            time.sleep(0.01)
        return lines

    def send_command(self, cmd, wait_for="OK:", timeout=30):
        if not self.ser or not self.ser.is_open:
            return False, []
        with self.lock:
            try:
                self.ser.reset_input_buffer()
                self.ser.write((cmd.strip() + '\n').encode())
                self.ser.flush()
                print(f"  → {cmd.strip()}")
                lines = []
                start = time.time()
                found = False
                while time.time() - start < timeout:
                    if self.ser.in_waiting > 0:
                        line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                        if line:
                            lines.append(line)
                            print(f"  ← {line}")
                            if wait_for and wait_for in line:
                                found = True
                                time.sleep(0.1)
                                lines.extend(self._read_all(0.2))
                                break
                            if line.startswith("ERR:") or line.startswith("LIMIT:") or "STOPPED" in line:
                                found = True
                                break
                    time.sleep(0.01)
                return found, lines
            except Exception as e:
                print(f"  ❌ {e}")
                return False, [str(e)]


# ========================================
# Leaf Analysis
# ========================================
def segment_leaves(frame, hsv_low, hsv_high):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, hsv_low, hsv_high)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    kernel_big = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_big, iterations=1)
    return mask

def analyze_frame(frame):
    mask = segment_leaves(frame, state.hsv_low, state.hsv_high)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    leaves = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < MIN_CONTOUR_AREA:
            continue
        rect = cv2.minAreaRect(contour)
        w, h = min(rect[1]), max(rect[1])
        hull = cv2.convexHull(contour)
        hull_area = cv2.contourArea(hull)
        leaves.append({
            "area_px": int(area),
            "width_px": int(w),
            "height_px": int(h),
            "solidity": round(area / hull_area, 3) if hull_area > 0 else 0,
        })
    total_px = mask.shape[0] * mask.shape[1]
    green_px = cv2.countNonZero(mask)
    coverage = round(green_px / total_px * 100, 2)
    total_area = sum(l["area_px"] for l in leaves)
    return {
        "leaf_count": len(leaves),
        "total_area_px": total_area,
        "green_coverage": coverage,
        "leaves": leaves,
    }


# ========================================
# Camera Thread
# ========================================
def camera_thread_fn():
    while True:
        if state.cap and state.cap.isOpened():
            ret, frame = state.cap.read()
            if ret:
                state.latest_frame = frame
        time.sleep(0.03)  # ~30fps


def generate_mjpeg():
    while True:
        if state.latest_frame is not None:
            frame = state.latest_frame.copy()
            # Draw overlay
            mask = segment_leaves(frame, state.hsv_low, state.hsv_high)
            green_overlay = np.zeros_like(frame)
            green_overlay[:] = (0, 200, 100)
            mask_3ch = cv2.merge([mask, mask, mask])
            green_overlay = cv2.bitwise_and(green_overlay, mask_3ch)
            frame = cv2.addWeighted(frame, 1.0, green_overlay, 0.3, 0)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            valid = [c for c in contours if cv2.contourArea(c) >= MIN_CONTOUR_AREA]
            cv2.drawContours(frame, valid, -1, (0, 255, 100), 2)
            for c in valid:
                rect = cv2.minAreaRect(c)
                box = np.int32(cv2.boxPoints(rect))
                cv2.drawContours(frame, [box], 0, (255, 200, 0), 2)

            _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
        time.sleep(0.05)


# ========================================
# Motor Actions (background)
# ========================================
def _calculate_pot_positions():
    if state.total_steps <= 0:
        return
    spacing = state.total_steps / (NUM_POTS + 1)
    state.pot_positions = [int(spacing * (i + 1)) for i in range(NUM_POTS)]

def do_calibrate():
    state.motor_busy = True
    state.motor_status = "Calibrating..."
    try:
        ok, lines = state.arduino.send_command("calibrate", "--- Calibration Complete ---", timeout=120)
        if ok:
            for line in lines:
                if "Total travel:" in line:
                    try:
                        state.total_steps = int(line.split(":")[1].strip().split()[0])
                    except:
                        pass
            if state.total_steps > 0:
                state.is_calibrated = True
                _calculate_pot_positions()
                state.motor_status = f"Calibrated ({state.total_steps} steps)"
            else:
                state.motor_status = "Calibration: couldn't read steps"
        else:
            state.motor_status = "Calibration failed"
    finally:
        state.motor_busy = False

def do_goto_pot(pot_num):
    state.motor_busy = True
    state.motor_status = f"Moving to Pot {pot_num}..."
    try:
        if not state.is_calibrated:
            state.motor_status = "Not calibrated"
            return False
        idx = pot_num - 1
        if idx < 0 or idx >= len(state.pot_positions):
            state.motor_status = "Invalid pot"
            return False
        target = state.pot_positions[idx]
        ok, _ = state.arduino.send_command(f"goto {target}", "OK:")
        if ok:
            state.current_position = target
            state.current_pot = pot_num
            state.motor_status = f"At Pot {pot_num}"
            return True
        else:
            state.motor_status = "Move failed"
            return False
    finally:
        state.motor_busy = False

def do_go_home():
    state.motor_busy = True
    state.motor_status = "Going Home..."
    try:
        ok, _ = state.arduino.send_command("home", "OK:")
        if ok:
            state.current_position = 0
            state.current_pot = None
            state.motor_status = "Home"
        else:
            state.motor_status = "Home failed"
    finally:
        state.motor_busy = False

def do_scan_pot(pot_num):
    state.motor_busy = True
    state.motor_status = f"Scanning Pot {pot_num}..."
    try:
        if state.serial_connected:
            if not state.is_calibrated:
                state.motor_status = "Not calibrated"
                return None
            target = state.pot_positions[pot_num - 1]
            ok, _ = state.arduino.send_command(f"goto {target}", "OK:")
            if not ok:
                state.motor_status = "Move failed"
                return None
            state.current_position = target
            state.current_pot = pot_num

        state.motor_status = f"Pot {pot_num}: settling..."
        time.sleep(2.0)

        # Flush camera buffer
        if state.cap:
            for _ in range(8):
                state.cap.read()

        frame = state.latest_frame
        if frame is None:
            state.motor_status = "No camera"
            return None

        analysis = analyze_frame(frame.copy())
        analysis["pot"] = pot_num
        analysis["timestamp"] = datetime.now().isoformat()

        # Save image
        os.makedirs(state.save_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        img_path = os.path.join(state.save_dir, f"pot{pot_num}_{ts}.jpg")
        cv2.imwrite(img_path, frame)
        analysis["image"] = img_path

        state.scan_history.append(analysis)
        state.last_analysis = analysis
        state.motor_status = f"Pot {pot_num}: done"
        return analysis
    finally:
        state.motor_busy = False

def do_auto_scan():
    state.motor_busy = True
    state.is_scanning = True
    state.motor_status = "Auto Scan..."
    try:
        results = []
        for pot in range(1, NUM_POTS + 1):
            state.motor_status = f"Auto Scan: Pot {pot}/{NUM_POTS}"
            state.motor_busy = True  # keep busy between pots

            if state.serial_connected:
                if not state.is_calibrated:
                    break
                target = state.pot_positions[pot - 1]
                ok, _ = state.arduino.send_command(f"goto {target}", "OK:")
                if not ok:
                    continue
                state.current_position = target
                state.current_pot = pot

            time.sleep(2.0)
            if state.cap:
                for _ in range(8):
                    state.cap.read()

            frame = state.latest_frame
            if frame is None:
                continue

            analysis = analyze_frame(frame.copy())
            analysis["pot"] = pot
            analysis["timestamp"] = datetime.now().isoformat()
            os.makedirs(state.save_dir, exist_ok=True)
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            img_path = os.path.join(state.save_dir, f"pot{pot}_{ts}.jpg")
            cv2.imwrite(img_path, frame)
            analysis["image"] = img_path
            state.scan_history.append(analysis)
            state.last_analysis = analysis
            results.append(analysis)

        # Go home
        if state.serial_connected:
            state.motor_status = "Returning home..."
            state.arduino.send_command("home", "OK:")
            state.current_position = 0
            state.current_pot = None

        state.motor_status = "Auto Scan Complete"
        return results
    finally:
        state.motor_busy = False
        state.is_scanning = False


# ========================================
# Flask Routes
# ========================================
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(generate_mjpeg(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/status')
def api_status():
    return jsonify({
        "serial_connected": state.serial_connected,
        "is_calibrated": state.is_calibrated,
        "total_steps": state.total_steps,
        "current_position": state.current_position,
        "current_pot": state.current_pot,
        "pot_positions": state.pot_positions,
        "motor_busy": state.motor_busy,
        "motor_status": state.motor_status,
        "is_scanning": state.is_scanning,
        "last_analysis": state.last_analysis,
    })

@app.route('/api/calibrate', methods=['POST'])
def api_calibrate():
    if not state.serial_connected:
        return jsonify({"error": "No serial connection"}), 400
    if state.motor_busy:
        return jsonify({"error": "Motor busy"}), 400
    Thread(target=do_calibrate, daemon=True).start()
    return jsonify({"ok": True, "message": "Calibrating..."})

@app.route('/api/goto/<int:pot>', methods=['POST'])
def api_goto(pot):
    if not state.serial_connected:
        return jsonify({"error": "No serial connection"}), 400
    if state.motor_busy:
        return jsonify({"error": "Motor busy"}), 400
    if pot < 1 or pot > NUM_POTS:
        return jsonify({"error": f"Invalid pot (1-{NUM_POTS})"}), 400
    Thread(target=do_goto_pot, args=(pot,), daemon=True).start()
    return jsonify({"ok": True, "message": f"Moving to Pot {pot}"})

@app.route('/api/home', methods=['POST'])
def api_home():
    if not state.serial_connected:
        return jsonify({"error": "No serial connection"}), 400
    if state.motor_busy:
        return jsonify({"error": "Motor busy"}), 400
    Thread(target=do_go_home, daemon=True).start()
    return jsonify({"ok": True, "message": "Going home"})

@app.route('/api/scan/<int:pot>', methods=['POST'])
def api_scan(pot):
    if state.motor_busy:
        return jsonify({"error": "Motor busy"}), 400
    if pot < 1 or pot > NUM_POTS:
        return jsonify({"error": f"Invalid pot (1-{NUM_POTS})"}), 400
    Thread(target=do_scan_pot, args=(pot,), daemon=True).start()
    return jsonify({"ok": True, "message": f"Scanning Pot {pot}"})

@app.route('/api/auto_scan', methods=['POST'])
def api_auto_scan():
    if state.motor_busy:
        return jsonify({"error": "Motor busy"}), 400
    Thread(target=do_auto_scan, daemon=True).start()
    return jsonify({"ok": True, "message": "Auto scanning all pots"})

@app.route('/api/history')
def api_history():
    return jsonify(state.scan_history)

@app.route('/api/analyze', methods=['POST'])
def api_analyze():
    """Analyze current frame without moving"""
    if state.latest_frame is None:
        return jsonify({"error": "No camera"}), 400
    analysis = analyze_frame(state.latest_frame.copy())
    analysis["pot"] = state.current_pot or 0
    analysis["timestamp"] = datetime.now().isoformat()
    state.last_analysis = analysis
    return jsonify(analysis)


# ========================================
# Main
# ========================================
def main():
    parser = argparse.ArgumentParser(description="🌱 Smart Farm Web Server")
    parser.add_argument("--port", type=str, default=None, help="Serial port")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--web-port", type=int, default=8080)
    parser.add_argument("--no-serial", action="store_true")
    parser.add_argument("--save-dir", type=str, default="scan_results")
    args = parser.parse_args()

    state.save_dir = args.save_dir

    # Serial
    if not args.no_serial:
        state.arduino = ArduinoController(args.port, args.baud)
        state.serial_connected = state.arduino.connect()
    else:
        state.arduino = ArduinoController()
        print("⚠️  No-serial mode")

    # Camera
    print(f"📷 Opening camera {args.camera}...")
    state.cap = cv2.VideoCapture(args.camera)
    if not state.cap.isOpened():
        print("❌ Camera failed!")
        return
    state.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    state.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    time.sleep(1)
    state.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
    state.cap.set(cv2.CAP_PROP_EXPOSURE, -6)
    state.cap.set(cv2.CAP_PROP_AUTO_WB, 0)
    print(f"✅ Camera: {int(state.cap.get(3))}×{int(state.cap.get(4))}")

    # Camera thread
    cam_thread = Thread(target=camera_thread_fn, daemon=True)
    cam_thread.start()

    print(f"\n🌐 Starting web server on http://localhost:{args.web_port}")
    app.run(host='0.0.0.0', port=args.web_port, debug=False, threaded=True)

    # Cleanup
    if state.cap:
        state.cap.release()
    if state.arduino:
        state.arduino.disconnect()


if __name__ == "__main__":
    main()
