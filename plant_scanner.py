#!/usr/bin/env python3
"""
🌱 Plant Scanner — ระบบสแกนต้นไม้อัตโนมัติ
รวม Leaf Detector + Arduino Stepper Motor Control

ฟีเจอร์:
  - เชื่อมต่อ Arduino ผ่าน Serial (ส่งคำสั่ง calibrate/goto/home)
  - เลื่อนกล้องไปยังแต่ละกระถาง → ถ่ายรูป → วิเคราะห์ใบ
  - Auto Scan: วนตรวจทุกกระถางอัตโนมัติ
  - บันทึกประวัติการวิเคราะห์

วิธีใช้:
    python3 plant_scanner.py --port /dev/ttyUSB0
    python3 plant_scanner.py --port /dev/cu.usbserial-0001 --camera 0

คำสั่งในโปรแกรม:
    1 / 2        = ไปกระถาง 1 / 2 + ถ่ายรูปวิเคราะห์
    A            = Auto Scan ทุกกระถาง
    C            = Calibrate มอเตอร์
    H            = กลับ Home
    SPACE        = ถ่ายรูปวิเคราะห์ ณ ตำแหน่งปัจจุบัน
    T            = เปิด/ปิด HSV Trackbar
    S            = บันทึกรูป
    L            = แสดงประวัติ (Log)
    Q / ESC      = ออก
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
from threading import Thread, Lock

# ========================================
# HSV สำหรับตรวจจับใบเขียว
# ========================================
HSV_LOW  = np.array([25, 40, 40])
HSV_HIGH = np.array([95, 255, 255])
MIN_CONTOUR_AREA = 500

# ========================================
# ตั้งค่ากระถาง (ปรับตามจริงหลัง Calibrate)
# ========================================
# pot_positions จะถูกคำนวณอัตโนมัติหลัง calibrate
# โดยแบ่งระยะทาง totalSteps ออกเป็นส่วนเท่าๆ กัน
NUM_POTS = 2


# ========================================
# Serial Manager
# ========================================
class ArduinoController:
    """จัดการการเชื่อมต่อ Serial กับ Arduino"""

    def __init__(self, port=None, baudrate=115200):
        self.ser = None
        self.port = port
        self.baudrate = baudrate
        self.lock = Lock()
        self.is_calibrated = False
        self.total_steps = 0
        self.current_position = 0
        self.pot_positions = []
        self.serial_log = []  # เก็บ log จาก Arduino

    def find_port(self):
        """ค้นหาพอร์ต Arduino/ESP32 อัตโนมัติ"""
        ports = serial.tools.list_ports.comports()
        for p in ports:
            desc = p.description.upper()
            if any(k in desc for k in ['USB', 'UART', 'CP210', 'CH340', 'SERIAL']):
                return p.device
        # macOS: ลอง cu.usbserial
        for p in ports:
            if 'cu.usb' in p.device or 'ttyUSB' in p.device:
                return p.device
        return None

    def connect(self):
        """เชื่อมต่อ Serial"""
        port = self.port or self.find_port()
        if not port:
            print("❌ ไม่พบพอร์ต Arduino!")
            print("   พอร์ตที่มี:")
            for p in serial.tools.list_ports.comports():
                print(f"     {p.device} — {p.description}")
            return False

        try:
            self.ser = serial.Serial(port, self.baudrate, timeout=2)
            time.sleep(2)  # รอ Arduino reset
            # อ่าน startup messages
            self._read_all()
            print(f"✅ Serial connected: {port} @ {self.baudrate} baud")
            return True
        except Exception as e:
            print(f"❌ Serial error: {e}")
            return False

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("📡 Serial closed")

    def _read_all(self, timeout=0.5):
        """อ่านข้อมูลทั้งหมดจาก Serial"""
        lines = []
        end_time = time.time() + timeout
        while time.time() < end_time:
            if self.ser and self.ser.in_waiting > 0:
                try:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        lines.append(line)
                        self.serial_log.append(line)
                        end_time = time.time() + 0.3  # ยืดเวลาถ้ายังมีข้อมูล
                except:
                    pass
            time.sleep(0.01)
        return lines

    def send_command(self, cmd, wait_for="OK:", timeout=30):
        """
        ส่งคำสั่งและรอ response
        wait_for: รอจนเห็นข้อความนี้ (หรือ timeout)
        Returns: (success, response_lines)
        """
        if not self.ser or not self.ser.is_open:
            return False, ["Serial not connected"]

        with self.lock:
            try:
                # ล้าง buffer
                self.ser.reset_input_buffer()

                # ส่งคำสั่ง
                cmd_str = cmd.strip() + '\n'
                self.ser.write(cmd_str.encode())
                self.ser.flush()
                print(f"  → Sent: {cmd.strip()}")

                # รอ response
                lines = []
                start = time.time()
                found = False

                while time.time() - start < timeout:
                    if self.ser.in_waiting > 0:
                        try:
                            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                            if line:
                                lines.append(line)
                                self.serial_log.append(line)
                                print(f"  ← {line}")

                                # ตรวจ response ที่ต้องการ
                                if wait_for and wait_for in line:
                                    found = True
                                    # อ่านเพิ่มนิดนึง
                                    time.sleep(0.1)
                                    lines.extend(self._read_all(0.2))
                                    break

                                # ตรวจ error
                                if line.startswith("ERR:") or line.startswith("LIMIT:"):
                                    found = True
                                    break
                                if "STOPPED" in line:
                                    found = True
                                    break

                        except Exception as e:
                            print(f"  ⚠️ Read error: {e}")
                    time.sleep(0.01)

                if not found and wait_for:
                    print(f"  ⚠️ Timeout waiting for '{wait_for}'")

                return found, lines

            except Exception as e:
                print(f"  ❌ Command error: {e}")
                return False, [str(e)]

    def calibrate(self):
        """สั่ง Calibrate และอ่านผลลัพธ์"""
        print("\n🔧 Calibrating...")
        success, lines = self.send_command("calibrate",
                                            wait_for="--- Calibration Complete ---",
                                            timeout=120)
        if success:
            # ดึง totalSteps จาก response
            for line in lines:
                if "Total travel:" in line:
                    try:
                        self.total_steps = int(line.split(":")[1].strip().split()[0])
                    except:
                        pass

            if self.total_steps > 0:
                self.is_calibrated = True
                self._calculate_pot_positions()
                print(f"✅ Calibrated! Total: {self.total_steps} steps")
                print(f"   Pot positions: {self.pot_positions}")
            else:
                print("⚠️ Calibrated but couldn't read total steps")
        else:
            print("❌ Calibration failed!")
        return success

    def _calculate_pot_positions(self):
        """คำนวณตำแหน่งกระถาง (แบ่งเท่าๆ กัน)"""
        if self.total_steps <= 0:
            return
        # แบ่งระยะเป็น NUM_POTS ส่วน
        # กระถาง 1 อยู่ 1/4 ของระยะ, กระถาง 2 อยู่ 3/4
        spacing = self.total_steps / (NUM_POTS + 1)
        self.pot_positions = [int(spacing * (i + 1)) for i in range(NUM_POTS)]

    def goto_pot(self, pot_num):
        """ไปยังกระถางที่ระบุ (1-based)"""
        if not self.is_calibrated:
            print("⚠️ ยังไม่ได้ Calibrate! กรุณากด C ก่อน")
            return False

        idx = pot_num - 1
        if idx < 0 or idx >= len(self.pot_positions):
            print(f"❌ กระถางที่ {pot_num} ไม่มี (มี 1-{NUM_POTS})")
            return False

        target = self.pot_positions[idx]
        print(f"\n🪴 Moving to Pot {pot_num} (position {target})...")
        success, lines = self.send_command(f"goto {target}", wait_for="OK: Arrived")

        if success:
            self.current_position = target
            print(f"✅ Arrived at Pot {pot_num}")
        return success

    def go_home(self):
        """กลับ Home"""
        print("\n🏠 Going Home...")
        success, lines = self.send_command("home", wait_for="OK:")
        if success:
            self.current_position = 0
        return success

    def get_status(self):
        """ดึงสถานะ"""
        success, lines = self.send_command("status", wait_for="--------------", timeout=3)
        return lines


# ========================================
# Leaf Analysis (จาก leaf_detector.py)
# ========================================
def segment_leaves(frame, hsv_low, hsv_high):
    """แยกใบไม้ด้วย HSV segmentation"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, hsv_low, hsv_high)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    kernel_big = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_big, iterations=1)
    return mask


def analyze_leaves(frame, mask):
    """วิเคราะห์ใบไม้ — returns (annotated_frame, leaf_data_list, green_coverage)"""
    result = frame.copy()
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    leaf_data = []
    leaf_id = 0

    for contour in contours:
        area_px = cv2.contourArea(contour)
        if area_px < MIN_CONTOUR_AREA:
            continue
        leaf_id += 1

        x, y, w, h = cv2.boundingRect(contour)
        rect = cv2.minAreaRect(contour)
        box = cv2.boxPoints(rect)
        box = np.int32(box)
        rect_w = min(rect[1])
        rect_h = max(rect[1])

        hull = cv2.convexHull(contour)
        hull_area = cv2.contourArea(hull)
        perimeter_px = cv2.arcLength(contour, True)

        info = {
            "id": leaf_id,
            "area_px": int(area_px),
            "perimeter_px": int(perimeter_px),
            "width_px": int(rect_w),
            "height_px": int(rect_h),
            "solidity": round(area_px / hull_area, 3) if hull_area > 0 else 0,
        }
        leaf_data.append(info)

        # วาดผล
        cv2.drawContours(result, [contour], -1, (0, 255, 100), 2)
        cv2.drawContours(result, [box], 0, (255, 200, 0), 2)

        label1 = f"Leaf {leaf_id}: {int(area_px)} px2"
        label2 = f"{int(rect_w)}x{int(rect_h)} px"

        cv2.rectangle(result, (x, y - 45),
                      (x + max(len(label1), len(label2)) * 10 + 10, y - 2),
                      (0, 0, 0), -1)
        cv2.putText(result, label1, (x + 4, y - 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 100), 1, cv2.LINE_AA)
        cv2.putText(result, label2, (x + 4, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1, cv2.LINE_AA)

    # Green coverage
    total_pixels = mask.shape[0] * mask.shape[1]
    green_pixels = cv2.countNonZero(mask)
    coverage = round(green_pixels / total_pixels * 100, 2)

    return result, leaf_data, coverage


def draw_status_bar(frame, pot_num, leaf_count, coverage, total_area, is_scanning=False):
    """วาด status bar ด้านบน"""
    bar_h = 80
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (frame.shape[1], bar_h), (30, 30, 30), -1)
    frame = cv2.addWeighted(overlay, 0.75, frame, 0.25, 0)

    pot_text = f"Pot {pot_num}" if pot_num else "Manual"
    scan_text = " [SCANNING]" if is_scanning else ""

    cv2.putText(frame, f"{pot_text}{scan_text} | Leaves: {leaf_count}",
                (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 100), 2, cv2.LINE_AA)
    cv2.putText(frame, f"Green: {coverage:.1f}% | Total Area: {total_area} px2",
                (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 255, 200), 1, cv2.LINE_AA)

    return frame


# ========================================
# Scan History
# ========================================
class ScanHistory:
    """เก็บประวัติการสแกน"""

    def __init__(self, save_dir="scan_results"):
        self.records = []
        self.save_dir = save_dir
        os.makedirs(save_dir, exist_ok=True)

    def add_record(self, pot_num, leaf_data, coverage, frame=None):
        """เพิ่มผลการสแกน"""
        timestamp = datetime.now().isoformat()
        total_area = sum(l["area_px"] for l in leaf_data)

        record = {
            "timestamp": timestamp,
            "pot": pot_num,
            "leaf_count": len(leaf_data),
            "total_area_px": total_area,
            "green_coverage": coverage,
            "leaves": leaf_data,
        }
        self.records.append(record)

        # บันทึกรูป
        if frame is not None:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(self.save_dir, f"pot{pot_num}_{ts}.jpg")
            cv2.imwrite(filename, frame)
            record["image"] = filename
            print(f"  💾 Saved: {filename}")

        return record

    def print_history(self):
        """แสดงประวัติ"""
        if not self.records:
            print("\n📋 No scan records yet.\n")
            return

        print("\n" + "=" * 60)
        print("  📋 Scan History")
        print("=" * 60)
        for i, r in enumerate(self.records):
            ts_short = r["timestamp"][11:19]
            print(f"  [{i+1}] {ts_short} | Pot {r['pot']} | "
                  f"Leaves: {r['leaf_count']} | "
                  f"Area: {r['total_area_px']} px² | "
                  f"Green: {r['green_coverage']:.1f}%")
        print("=" * 60 + "\n")

    def save_json(self):
        """บันทึกประวัติเป็น JSON"""
        if not self.records:
            return
        filepath = os.path.join(self.save_dir, "scan_history.json")
        # ลบ key 'image' ก่อนบันทึก (เก็บแค่ชื่อไฟล์)
        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(self.records, f, indent=2, ensure_ascii=False)
        print(f"📄 History saved: {filepath}")


# ========================================
# Trackbar
# ========================================
def nothing(x):
    pass

def create_trackbars(window_name="HSV Adjust"):
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 400, 300)
    cv2.createTrackbar("H Low",  window_name, HSV_LOW[0],  179, nothing)
    cv2.createTrackbar("S Low",  window_name, HSV_LOW[1],  255, nothing)
    cv2.createTrackbar("V Low",  window_name, HSV_LOW[2],  255, nothing)
    cv2.createTrackbar("H High", window_name, HSV_HIGH[0], 179, nothing)
    cv2.createTrackbar("S High", window_name, HSV_HIGH[1], 255, nothing)
    cv2.createTrackbar("V High", window_name, HSV_HIGH[2], 255, nothing)

def get_trackbar_values(window_name="HSV Adjust"):
    h_low  = cv2.getTrackbarPos("H Low",  window_name)
    s_low  = cv2.getTrackbarPos("S Low",  window_name)
    v_low  = cv2.getTrackbarPos("V Low",  window_name)
    h_high = cv2.getTrackbarPos("H High", window_name)
    s_high = cv2.getTrackbarPos("S High", window_name)
    v_high = cv2.getTrackbarPos("V High", window_name)
    return np.array([h_low, s_low, v_low]), np.array([h_high, s_high, v_high])


# ========================================
# Main
# ========================================
def main():
    parser = argparse.ArgumentParser(description="🌱 Plant Scanner")
    parser.add_argument("--port", type=str, default=None,
                        help="Serial port (auto-detect if not specified)")
    parser.add_argument("--baud", type=int, default=115200,
                        help="Baud rate (default: 115200)")
    parser.add_argument("--camera", type=int, default=0,
                        help="Camera index (default: 0)")
    parser.add_argument("--save-dir", type=str, default="scan_results",
                        help="Directory to save results (default: scan_results)")
    parser.add_argument("--no-serial", action="store_true",
                        help="Run without Arduino (camera only)")
    args = parser.parse_args()

    # --- Serial ---
    arduino = ArduinoController(args.port, args.baud)
    serial_connected = False
    if not args.no_serial:
        serial_connected = arduino.connect()
        if not serial_connected:
            print("\n⚠️  Running without Arduino (camera only mode)")
            print("   Use --no-serial to skip this warning\n")
    else:
        print("⚠️  No-serial mode: camera only\n")

    # --- Camera ---
    print(f"📷 Opening camera {args.camera}...")
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print("❌ ไม่สามารถเปิดกล้องได้!")
        arduino.disconnect()
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"✅ Camera: {actual_w}×{actual_h}")

    # Lock camera settings — ป้องกัน auto-adjust ทำให้ค่าผันผวน
    time.sleep(1)  # ให้กล้อง warm up ก่อน lock
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)   # 1 = manual mode
    cap.set(cv2.CAP_PROP_EXPOSURE, -6)        # ค่า exposure คงที่
    cap.set(cv2.CAP_PROP_AUTO_WB, 0)          # ปิด auto white balance
    print("🔒 Camera locked: exposure + white balance")

    # --- History ---
    history = ScanHistory(args.save_dir)

    # --- State ---
    hsv_low = HSV_LOW.copy()
    hsv_high = HSV_HIGH.copy()
    show_trackbar = False
    current_pot = None
    is_scanning = False
    motor_busy = False       # True = กำลังสั่งมอเตอร์อยู่ (ห้ามกดซ้ำ)
    motor_status = ""        # ข้อความแสดงสถานะมอเตอร์บน Live feed
    pending_analyze = False  # True = มอเตอร์ถึงแล้ว รอถ่ายรูปวิเคราะห์
    pending_pot = None

    print("\n" + "=" * 50)
    print("  🌱 Plant Scanner — Ready")
    print("=" * 50)
    print("  Keyboard Controls:")
    print("    1 / 2   = Go to Pot 1 / 2 + Analyze")
    print("    A       = Auto Scan all pots")
    print("    C       = Calibrate stepper motor")
    print("    H       = Go Home")
    print("    SPACE   = Capture + Analyze (current position)")
    print("    T       = Toggle HSV Trackbar")
    print("    S       = Save current frame")
    print("    L       = Show scan history (Log)")
    print("    Q / ESC = Quit")
    print("=" * 50 + "\n")

    # --- Background thread helpers ---
    # ทุกคำสั่งที่ต้องรอ Serial จะรันใน thread แยก
    # เพื่อให้ camera loop ไม่ค้าง

    def _run_in_background(fn, *a, **kw):
        """รัน function ใน background thread (ถ้าไม่ busy)"""
        nonlocal motor_busy
        if motor_busy:
            print("⚠️ Motor is busy — please wait")
            return
        motor_busy = True
        t = Thread(target=fn, args=a, kwargs=kw, daemon=True)
        t.start()

    def _do_goto_and_scan(pot_num):
        """[Background] ไปกระถาง → ตั้ง flag ให้ main loop ถ่ายรูป"""
        nonlocal motor_busy, motor_status, current_pot, pending_analyze, pending_pot
        try:
            motor_status = f"🔄 Moving to Pot {pot_num}..."
            if serial_connected:
                if not arduino.goto_pot(pot_num):
                    motor_status = f"❌ Failed to reach Pot {pot_num}"
                    return
            current_pot = pot_num
            motor_status = f"📷 Pot {pot_num} — settling..."
            time.sleep(2.0)  # ให้กล้อง settle 2 วินาที
            # ตั้ง flag ให้ main loop ถ่ายรูปวิเคราะห์
            pending_pot = pot_num
            pending_analyze = True
            motor_status = f"✅ Pot {pot_num} — analyzing"
        finally:
            motor_busy = False

    def _do_auto_scan():
        """[Background] สแกนทุกกระถางอัตโนมัติ"""
        nonlocal motor_busy, motor_status, is_scanning, current_pot
        nonlocal pending_analyze, pending_pot
        try:
            is_scanning = True
            print("\n" + "=" * 50)
            print("  🔄 AUTO SCAN START")
            print("=" * 50)

            for pot in range(1, NUM_POTS + 1):
                motor_status = f"🔄 Auto Scan: Moving to Pot {pot}/{NUM_POTS}..."
                print(f"\n--- Pot {pot}/{NUM_POTS} ---")

                if serial_connected:
                    if not arduino.goto_pot(pot):
                        continue
                current_pot = pot

                motor_status = f"📷 Auto Scan: Pot {pot} settling..."
                time.sleep(2.0)

                # ตั้ง flag แล้วรอให้ main loop ถ่ายรูปเสร็จ
                pending_pot = pot
                pending_analyze = True
                # รอจน main loop ประมวลผลเสร็จ
                while pending_analyze:
                    time.sleep(0.05)

            # กลับ Home
            motor_status = "🏠 Auto Scan: Returning home..."
            if serial_connected:
                arduino.go_home()

            is_scanning = False
            motor_status = "✅ Auto Scan Complete"
            print("=" * 50)
            print("  ✅ AUTO SCAN COMPLETE")
            print("=" * 50)
            history.print_history()

            # ลบสถานะหลัง 3 วินาที
            time.sleep(3)
            motor_status = ""
        finally:
            motor_busy = False
            is_scanning = False

    def _do_calibrate():
        """[Background] Calibrate มอเตอร์"""
        nonlocal motor_busy, motor_status
        try:
            motor_status = "🔧 Calibrating..."
            arduino.calibrate()
            if arduino.is_calibrated:
                motor_status = f"✅ Calibrated ({arduino.total_steps} steps)"
            else:
                motor_status = "❌ Calibration failed"
            time.sleep(2)
            motor_status = ""
        finally:
            motor_busy = False

    def _do_go_home():
        """[Background] กลับ Home"""
        nonlocal motor_busy, motor_status, current_pot
        try:
            motor_status = "🏠 Going Home..."
            arduino.go_home()
            current_pot = None
            motor_status = "✅ Home"
            time.sleep(1)
            motor_status = ""
        finally:
            motor_busy = False

    def _do_capture_and_analyze(pot_num):
        """ถ่ายรูปและวิเคราะห์ (เรียกจาก main loop, ไม่ block)"""
        # ถ่ายหลายเฟรมทิ้ง (ล้าง buffer ของกล้อง)
        for _ in range(5):
            cap.read()

        ret, frame = cap.read()
        if not ret:
            print("  ❌ Failed to capture frame")
            return

        mask = segment_leaves(frame, hsv_low, hsv_high)
        result, leaf_data, coverage = analyze_leaves(frame, mask)
        total_area = sum(l["area_px"] for l in leaf_data)

        result = draw_status_bar(result, pot_num, len(leaf_data), coverage, total_area, is_scanning)
        cv2.imshow("Plant Scanner", result)

        record = history.add_record(pot_num, leaf_data, coverage, result)

        print(f"\n  🌿 Pot {pot_num} Results:")
        print(f"     Leaves found: {len(leaf_data)}")
        print(f"     Total area:   {total_area} px²")
        print(f"     Green cover:  {coverage:.1f}%")
        for leaf in leaf_data:
            print(f"     Leaf #{leaf['id']}: {leaf['area_px']} px² "
                  f"({leaf['width_px']}×{leaf['height_px']} px)")
        print()

    # === Main Loop (ไม่ block — กล้อง realtime ตลอด) ===
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # --- ถ้ามี pending analyze (มอเตอร์ถึงแล้ว) → ถ่ายรูปวิเคราะห์ ---
        if pending_analyze and pending_pot is not None:
            _do_capture_and_analyze(pending_pot)
            pending_analyze = False
            pending_pot = None

        # Live preview + overlay
        display = frame.copy()
        mask = segment_leaves(frame, hsv_low, hsv_high)

        # Live overlay
        green_overlay = np.zeros_like(frame)
        green_overlay[:] = (0, 200, 100)
        mask_3ch = cv2.merge([mask, mask, mask])
        green_overlay = cv2.bitwise_and(green_overlay, mask_3ch)
        display = cv2.addWeighted(display, 1.0, green_overlay, 0.25, 0)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        leaf_count = sum(1 for c in contours if cv2.contourArea(c) >= MIN_CONTOUR_AREA)

        # Status text
        serial_status = "Connected" if serial_connected else "No Serial"
        cal_status = f"Cal({arduino.total_steps})" if arduino.is_calibrated else "Not cal"
        pot_text = f"Pot {current_pot}" if current_pot else "-"

        cv2.putText(display, f"LIVE | Leaves: {leaf_count} | {serial_status} | {cal_status}",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 100), 2, cv2.LINE_AA)
        cv2.putText(display, f"Pot: {pot_text} | 1-2=Pot A=Scan C=Cal H=Home SPACE=Analyze",
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1, cv2.LINE_AA)

        # แสดงสถานะมอเตอร์ (ถ้ามี)
        if motor_status:
            cv2.putText(display, motor_status,
                        (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2, cv2.LINE_AA)

        if show_trackbar:
            hsv_low, hsv_high = get_trackbar_values()

        # แสดง
        cv2.imshow("Plant Scanner", display)

        # Mask window
        mask_small = cv2.resize(mask, (320, 180))
        mask_colored = cv2.cvtColor(mask_small, cv2.COLOR_GRAY2BGR)
        cv2.imshow("Mask", mask_colored)

        # --- Keys (ไม่ block, ทุกคำสั่งรันใน thread) ---
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q') or key == 27:
            break

        elif key == ord('1'):
            _run_in_background(_do_goto_and_scan, 1)

        elif key == ord('2'):
            _run_in_background(_do_goto_and_scan, 2)

        elif key == ord('a') or key == ord('A'):
            if serial_connected and arduino.is_calibrated:
                _run_in_background(_do_auto_scan)
            elif not serial_connected:
                print("⚠️ ไม่ได้เชื่อมต่อ Serial — Auto Scan ต้องใช้ Arduino")
            else:
                print("⚠️ ยัง Calibrate ไม่ได้ — กด C ก่อน")

        elif key == ord('c') or key == ord('C'):
            if serial_connected:
                _run_in_background(_do_calibrate)
            else:
                print("⚠️ ไม่ได้เชื่อมต่อ Serial")

        elif key == ord('h') or key == ord('H'):
            if serial_connected:
                _run_in_background(_do_go_home)

        elif key == ord(' '):
            # วิเคราะห์ ณ ตำแหน่งปัจจุบัน (ทำใน main loop โดยตรง, ไม่ต้อง thread)
            if not motor_busy:
                pot = current_pot or 0
                _do_capture_and_analyze(pot)
                print(f"  🌿 Analyzed at current position\n")

        elif key == ord('t') or key == ord('T'):
            show_trackbar = not show_trackbar
            if show_trackbar:
                create_trackbars()
            else:
                cv2.destroyWindow("HSV Adjust")

        elif key == ord('s') or key == ord('S'):
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(args.save_dir, f"manual_{ts}.jpg")
            cv2.imwrite(filename, display)
            print(f"💾 Saved: {filename}")

        elif key == ord('l') or key == ord('L'):
            history.print_history()

    # === Cleanup ===
    history.save_json()
    cap.release()
    cv2.destroyAllWindows()
    arduino.disconnect()
    print("👋 Done!")


if __name__ == "__main__":
    main()
