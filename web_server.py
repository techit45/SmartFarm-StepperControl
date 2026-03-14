#!/usr/bin/env python3
"""
🌱 Smart Farm Web Server — YOLO Object Detection Edition
Flask server + Camera Stream + Arduino Control + YOLO best.pt Analysis
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
MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "best.pt")
CONFIDENCE_THRESHOLD = 0.25
IOU_THRESHOLD = 0.45
NUM_POTS = 2

# Flask — ใช้ path ของไฟล์นี้เป็นฐาน (ไม่ขึ้นกับ CWD)
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
app = Flask(__name__,
            static_folder=os.path.join(BASE_DIR, 'static'),
            template_folder=os.path.join(BASE_DIR, 'templates'))

# ========================================
# YOLO Model
# ========================================
yolo_model = None

def load_yolo_model():
    """โหลด YOLO model จาก best.pt"""
    global yolo_model
    try:
        from ultralytics import YOLO
        if not os.path.exists(MODEL_PATH):
            print(f"❌ Model file not found: {MODEL_PATH}")
            print(f"   กรุณาวาง best.pt ไว้ใน {BASE_DIR}")
            return False
        print(f"🤖 Loading YOLO model: {MODEL_PATH}")
        yolo_model = YOLO(MODEL_PATH)
        # Warm up with dummy image
        dummy = np.zeros((640, 640, 3), dtype=np.uint8)
        yolo_model.predict(dummy, verbose=False)
        print(f"✅ YOLO model loaded — classes: {yolo_model.names}")
        return True
    except ImportError:
        print("❌ ultralytics not installed! Run: pip install ultralytics")
        return False
    except Exception as e:
        print(f"❌ YOLO load error: {e}")
        return False

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
        self.save_dir = "scan_results"
        # Pump & Soil
        self.pump_states = [False, False]
        self.soil_values = [0, 0]
        # YOLO
        self.model_loaded = False
        self.model_classes = {}  # {id: name}
        # Camera
        self.camera_id = None  # current camera index

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
            except OSError:
                raise  # ให้ caller จัดการ (เช่น device disconnected)
            except Exception as e:
                print(f"  ❌ {e}")
                return False, [str(e)]


# ========================================
# YOLO Analysis
# ========================================
# สีสำหรับแต่ละ class (จะ cycle ถ้ามากกว่า 10 คลาส)
CLASS_COLORS = [
    (0, 255, 100),   # เขียว
    (255, 100, 0),   # ส้ม
    (0, 150, 255),   # ฟ้า
    (255, 0, 100),   # ชมพู
    (255, 255, 0),   # เหลือง
    (200, 0, 255),   # ม่วง
    (0, 255, 255),   # เหลืองเขียว
    (100, 200, 255), # ฟ้าอ่อน
    (255, 150, 150), # ชมพูอ่อน
    (150, 255, 150), # เขียวอ่อน
]

def get_class_color(class_id):
    return CLASS_COLORS[class_id % len(CLASS_COLORS)]

def analyze_frame(frame):
    """วิเคราะห์ frame ด้วย YOLO model"""
    if yolo_model is None:
        return {
            "detection_count": 0,
            "detections": [],
            "class_summary": {},
            "error": "Model not loaded"
        }

    results = yolo_model.predict(
        frame,
        conf=CONFIDENCE_THRESHOLD,
        iou=IOU_THRESHOLD,
        verbose=False
    )

    detections = []
    class_counts = {}

    if results and len(results) > 0:
        result = results[0]
        boxes = result.boxes

        for box in boxes:
            cls_id = int(box.cls[0])
            cls_name = result.names[cls_id]
            conf = float(box.conf[0])
            x1, y1, x2, y2 = box.xyxy[0].tolist()

            detections.append({
                "class": cls_name,
                "class_id": cls_id,
                "confidence": round(conf, 3),
                "bbox": {
                    "x1": int(x1), "y1": int(y1),
                    "x2": int(x2), "y2": int(y2),
                    "width": int(x2 - x1),
                    "height": int(y2 - y1),
                },
                "area_px": int((x2 - x1) * (y2 - y1)),
            })

            class_counts[cls_name] = class_counts.get(cls_name, 0) + 1

    avg_conf = round(
        sum(d["confidence"] for d in detections) / len(detections), 3
    ) if detections else 0

    return {
        "detection_count": len(detections),
        "detections": detections,
        "class_summary": class_counts,
        "avg_confidence": avg_conf,
    }


def draw_detections(frame, detections):
    """วาด bounding box บน frame"""
    annotated = frame.copy()

    for det in detections:
        bb = det["bbox"]
        cls_name = det["class"]
        conf = det["confidence"]
        cls_id = det.get("class_id", 0)
        color = get_class_color(cls_id)

        # Bounding box
        cv2.rectangle(annotated, (bb["x1"], bb["y1"]), (bb["x2"], bb["y2"]), color, 2)

        # Label background
        label = f'{cls_name} {conf:.0%}'
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
        cv2.rectangle(annotated,
                      (bb["x1"], bb["y1"] - th - 10),
                      (bb["x1"] + tw + 8, bb["y1"]),
                      color, -1)
        cv2.putText(annotated, label,
                    (bb["x1"] + 4, bb["y1"] - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)

    # Summary text at top-left
    if detections:
        summary = f"Detected: {len(detections)} objects"
        cv2.putText(annotated, summary, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 100), 2, cv2.LINE_AA)

    return annotated


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


def sensor_poll_fn():
    """Background thread: อ่าน sensors จาก Arduino ทุก 2 วินาที"""
    fail_count = 0
    while True:
        if state.serial_connected and not state.motor_busy:
            try:
                # ตรวจว่า serial ยังเปิดอยู่
                if not state.arduino.ser or not state.arduino.ser.is_open:
                    state.serial_connected = False
                    fail_count = 0
                    time.sleep(5)
                    continue
                ok, lines = state.arduino.send_command("sensors", "SENSORS:", timeout=3)
                if ok:
                    fail_count = 0
                    for line in lines:
                        if line.startswith("SENSORS:"):
                            _parse_sensors(line)
                else:
                    fail_count += 1
            except OSError:
                # Device disconnected (e.g. ESP32 reset)
                print("⚠️  Serial device lost, stopping sensor poll")
                state.serial_connected = False
                fail_count = 0
            except Exception:
                fail_count += 1
        # หยุดพักนานขึ้นถ้า fail หลายครั้ง (ป้องกัน spam)
        time.sleep(5 if fail_count > 3 else 2)


def _parse_sensors(line):
    """Parse: SENSORS:soil1=2048,soil2=1890,pump1=0,pump2=1"""
    try:
        data = line.split(":", 1)[1]  # soil1=2048,soil2=...
        pairs = {}
        for part in data.split(","):
            k, v = part.split("=")
            pairs[k.strip()] = int(v.strip())
        state.soil_values[0] = pairs.get("soil1", 0)
        state.soil_values[1] = pairs.get("soil2", 0)
        state.pump_states[0] = pairs.get("pump1", 0) == 1
        state.pump_states[1] = pairs.get("pump2", 0) == 1
    except Exception:
        pass


def generate_mjpeg():
    """MJPEG stream พร้อม YOLO bounding box overlay"""
    while True:
        if state.latest_frame is not None:
            frame = state.latest_frame.copy()

            # ถ้า model โหลดแล้ว → รัน YOLO
            if yolo_model is not None:
                analysis = analyze_frame(frame)
                frame = draw_detections(frame, analysis.get("detections", []))

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
        # รอให้ sensor poll จบก่อน + flush serial buffer
        time.sleep(0.5)
        if state.arduino.ser:
            state.arduino.ser.reset_input_buffer()
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

        # Save image (original + annotated)
        os.makedirs(state.save_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        img_path = os.path.join(state.save_dir, f"pot{pot_num}_{ts}.jpg")
        cv2.imwrite(img_path, frame)

        # Save annotated
        annotated = draw_detections(frame.copy(), analysis.get("detections", []))
        ann_path = os.path.join(state.save_dir, f"pot{pot_num}_{ts}_detected.jpg")
        cv2.imwrite(ann_path, annotated)

        analysis["image"] = img_path
        analysis["image_annotated"] = ann_path

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
        "pump_states": state.pump_states,
        "soil_values": state.soil_values,
        "model_loaded": state.model_loaded,
        "model_classes": state.model_classes,
        "confidence_threshold": CONFIDENCE_THRESHOLD,
        "serial_port": state.arduino.port if state.arduino and state.arduino.ser and state.arduino.ser.is_open else None,
        "camera_opened": state.cap is not None and state.cap.isOpened(),
        "camera_id": state.camera_id,
    })

@app.route('/api/calibrate', methods=['POST'])
def api_calibrate():
    if not state.serial_connected:
        return jsonify({"error": "No serial connection"}), 400
    if state.motor_busy:
        return jsonify({"error": "Motor busy"}), 400
    state.motor_busy = True  # ตั้งก่อนเริ่ม thread เพื่อป้องกัน sensor poll แย่ง serial
    state.motor_status = "Calibrating..."
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
    state.motor_busy = True
    state.motor_status = f"Moving to Pot {pot}..."
    Thread(target=do_goto_pot, args=(pot,), daemon=True).start()
    return jsonify({"ok": True, "message": f"Moving to Pot {pot}"})

@app.route('/api/home', methods=['POST'])
def api_home():
    if not state.serial_connected:
        return jsonify({"error": "No serial connection"}), 400
    if state.motor_busy:
        return jsonify({"error": "Motor busy"}), 400
    state.motor_busy = True
    state.motor_status = "Going Home..."
    Thread(target=do_go_home, daemon=True).start()
    return jsonify({"ok": True, "message": "Going home"})

@app.route('/api/scan/<int:pot>', methods=['POST'])
def api_scan(pot):
    if state.motor_busy:
        return jsonify({"error": "Motor busy"}), 400
    if pot < 1 or pot > NUM_POTS:
        return jsonify({"error": f"Invalid pot (1-{NUM_POTS})"}), 400
    state.motor_busy = True
    state.motor_status = f"Scanning Pot {pot}..."
    Thread(target=do_scan_pot, args=(pot,), daemon=True).start()
    return jsonify({"ok": True, "message": f"Scanning Pot {pot}"})

@app.route('/api/auto_scan', methods=['POST'])
def api_auto_scan():
    if state.motor_busy:
        return jsonify({"error": "Motor busy"}), 400
    state.motor_busy = True
    state.motor_status = "Auto Scan..."
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
    if not state.model_loaded:
        return jsonify({"error": "Model not loaded"}), 400
    analysis = analyze_frame(state.latest_frame.copy())
    analysis["pot"] = state.current_pot or 0
    analysis["timestamp"] = datetime.now().isoformat()
    state.last_analysis = analysis
    return jsonify(analysis)

@app.route('/api/pump/<int:pump>/<action>', methods=['POST'])
def api_pump(pump, action):
    if not state.serial_connected:
        return jsonify({"error": "No serial connection"}), 400
    if pump < 1 or pump > 2:
        return jsonify({"error": "Pump must be 1 or 2"}), 400
    if action not in ('on', 'off'):
        return jsonify({"error": "Action must be on or off"}), 400
    try:
        ok, lines = state.arduino.send_command(f"pump {pump} {action}", "OK:", timeout=5)
    except OSError:
        state.serial_connected = False
        return jsonify({"error": "Serial device lost"}), 503
    if ok:
        state.pump_states[pump - 1] = (action == 'on')
        return jsonify({"ok": True, "message": f"Pump {pump} {action.upper()}"})
    else:
        return jsonify({"error": "Pump command failed"}), 500

@app.route('/api/confidence', methods=['POST'])
def api_confidence():
    global CONFIDENCE_THRESHOLD
    data = request.get_json(silent=True) or {}
    val = data.get('confidence')
    if val is None:
        return jsonify({"error": "Missing 'confidence' value"}), 400
    try:
        val = float(val)
    except (TypeError, ValueError):
        return jsonify({"error": "Invalid confidence value"}), 400
    if val < 0.01 or val > 1.0:
        return jsonify({"error": "Confidence must be between 0.01 and 1.0"}), 400
    CONFIDENCE_THRESHOLD = val
    print(f"🎯 Confidence threshold set to {val:.0%}")
    return jsonify({"ok": True, "confidence": val, "message": f"Confidence set to {val:.0%}"})

@app.route('/api/sensors')
def api_sensors():
    return jsonify({
        "pump_states": state.pump_states,
        "soil_values": state.soil_values,
    })

@app.route('/api/ports')
def api_ports():
    """แสดง serial port ที่มี"""
    ports = []
    for p in serial.tools.list_ports.comports():
        ports.append({
            "device": p.device,
            "description": p.description,
            "hwid": p.hwid,
        })
    return jsonify({
        "ports": ports,
        "connected": state.serial_connected,
        "current_port": state.arduino.port if state.arduino and state.arduino.ser and state.arduino.ser.is_open else None,
    })

@app.route('/api/connect', methods=['POST'])
def api_connect():
    """เชื่อมต่อ serial port"""
    if state.serial_connected:
        return jsonify({"error": "Already connected. Disconnect first."}), 400
    data = request.get_json(silent=True) or {}
    port = data.get('port')
    baud = data.get('baud', 115200)
    if not port:
        return jsonify({"error": "Missing 'port' parameter"}), 400
    # สร้าง controller ใหม่ถ้ายังไม่มี
    state.arduino = ArduinoController(port, baud)
    ok = state.arduino.connect()
    if ok:
        state.serial_connected = True
        # เริ่ม sensor poll thread ถ้ายังไม่มี
        Thread(target=sensor_poll_fn, daemon=True).start()
        return jsonify({"ok": True, "message": f"Connected to {port}", "port": port})
    else:
        state.serial_connected = False
        return jsonify({"error": f"Failed to connect to {port}"}), 500

@app.route('/api/disconnect', methods=['POST'])
def api_disconnect():
    """ตัดการเชื่อมต่อ serial"""
    if not state.serial_connected:
        return jsonify({"error": "Not connected"}), 400
    if state.motor_busy:
        return jsonify({"error": "Motor busy, cannot disconnect"}), 400
    try:
        if state.arduino:
            state.arduino.disconnect()
    except Exception:
        pass
    state.serial_connected = False
    state.is_calibrated = False
    state.total_steps = 0
    state.current_position = 0
    state.current_pot = None
    state.pot_positions = []
    state.motor_status = ""
    return jsonify({"ok": True, "message": "Disconnected"})

@app.route('/api/cameras')
def api_cameras():
    """ตรวจหากล้องที่ใช้ได้ (index 0-4)"""
    available = []
    for i in range(5):
        # ถ้ากล้องนี้เปิดอยู่แล้ว ไม่ต้องทดสอบ
        if state.cap and state.camera_id == i:
            available.append({"index": i, "name": f"Camera {i}", "active": True})
        else:
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                available.append({"index": i, "name": f"Camera {i}", "active": False})
                cap.release()
    return jsonify({
        "cameras": available,
        "current": state.camera_id,
        "opened": state.cap is not None and state.cap.isOpened(),
    })

@app.route('/api/camera/open', methods=['POST'])
def api_camera_open():
    """เปิดกล้องตาม index"""
    data = request.get_json(silent=True) or {}
    cam_id = data.get('camera', 0)
    try:
        cam_id = int(cam_id)
    except (TypeError, ValueError):
        return jsonify({"error": "Invalid camera index"}), 400
    # ปิดกล้องเดิมถ้ามี
    if state.cap and state.cap.isOpened():
        state.cap.release()
        state.cap = None
        state.latest_frame = None
        time.sleep(0.3)
    cap = cv2.VideoCapture(cam_id)
    if not cap.isOpened():
        return jsonify({"error": f"Cannot open Camera {cam_id}"}), 500
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    time.sleep(0.5)
    state.cap = cap
    state.camera_id = cam_id
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"✅ Camera {cam_id} opened: {w}×{h}")
    return jsonify({"ok": True, "message": f"Camera {cam_id} opened ({w}×{h})", "camera": cam_id, "width": w, "height": h})

@app.route('/api/camera/close', methods=['POST'])
def api_camera_close():
    """ปิดกล้อง"""
    if state.cap and state.cap.isOpened():
        state.cap.release()
    state.cap = None
    state.latest_frame = None
    state.camera_id = None
    print("📷 Camera closed")
    return jsonify({"ok": True, "message": "Camera closed"})


# ========================================
# Main
# ========================================
def main():
    parser = argparse.ArgumentParser(description="🌱 Smart Farm Web Server — YOLO Edition")
    parser.add_argument("--port", type=str, default=None, help="Serial port")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--web-port", type=int, default=8080)
    parser.add_argument("--no-serial", action="store_true")
    parser.add_argument("--save-dir", type=str, default="scan_results")
    parser.add_argument("--model", type=str, default=None, help="Path to YOLO model (best.pt)")
    parser.add_argument("--confidence", type=float, default=0.25, help="Detection confidence threshold")
    args = parser.parse_args()

    global MODEL_PATH, CONFIDENCE_THRESHOLD
    if args.model:
        MODEL_PATH = args.model
    CONFIDENCE_THRESHOLD = args.confidence
    state.save_dir = args.save_dir

    # Load YOLO model
    state.model_loaded = load_yolo_model()
    if state.model_loaded and yolo_model:
        state.model_classes = dict(yolo_model.names)

    # Serial — ไม่ auto-connect ถ้าไม่ระบุ --port (ให้เลือกจาก UI)
    state.arduino = ArduinoController(args.port, args.baud)
    if not args.no_serial and args.port:
        state.serial_connected = state.arduino.connect()
    elif not args.no_serial:
        print("ℹ️  No --port specified. Select port from the web UI.")
    else:
        print("⚠️  No-serial mode")

    # Camera — ไม่ auto-open ถ้าไม่ระบุ --camera (ให้เลือกจาก UI)
    if args.camera is not None:
        print(f"📷 Opening camera {args.camera}...")
        state.cap = cv2.VideoCapture(args.camera)
        if not state.cap.isOpened():
            print("❌ Camera failed! Select camera from the web UI.")
            state.cap = None
        else:
            state.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            state.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            time.sleep(1)
            state.camera_id = args.camera
            print(f"✅ Camera: {int(state.cap.get(3))}×{int(state.cap.get(4))}")
    else:
        print("ℹ️  No --camera specified. Select camera from the web UI.")

    # Camera thread
    cam_thread = Thread(target=camera_thread_fn, daemon=True)
    cam_thread.start()

    # Sensor polling thread
    if state.serial_connected:
        sensor_thread = Thread(target=sensor_poll_fn, daemon=True)
        sensor_thread.start()

    print(f"\n🌐 Starting web server on http://localhost:{args.web_port}")
    app.run(host='0.0.0.0', port=args.web_port, debug=False, threaded=True)

    # Cleanup
    if state.cap:
        state.cap.release()
    if state.arduino:
        state.arduino.disconnect()


if __name__ == "__main__":
    main()
