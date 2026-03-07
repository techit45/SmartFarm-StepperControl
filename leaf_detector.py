#!/usr/bin/env python3
"""
🌿 Leaf Size Detector — ตรวจจับและวัดขนาดใบไม้ผ่าน USB Webcam
ใช้ OpenCV ทำ HSV segmentation + contour detection

วิธีใช้:
    python3 leaf_detector.py
    python3 leaf_detector.py --camera 1
    python3 leaf_detector.py --ref-width 5.0    # ใส่ค่าความกว้างวัตถุอ้างอิง (cm)

ปุ่มกด:
    SPACE  = Capture + วิเคราะห์ใบ
    C      = สลับโหมด (Live / Captured)
    T      = เปิด/ปิด Trackbar ปรับค่า HSV
    S      = บันทึกรูปผลลัพธ์
    R      = รีเซ็ตกลับ Live
    Q/ESC  = ออก
"""

import cv2
import numpy as np
import argparse
import time
import os
from datetime import datetime


# ========================================
# Default HSV Range สำหรับใบไม้สีเขียว
# ========================================
HSV_LOW  = np.array([25, 40, 40])
HSV_HIGH = np.array([95, 255, 255])

# กรองใบขนาดเล็กเกินไป (noise)
MIN_CONTOUR_AREA = 500  # pixels


def nothing(x):
    pass


def create_trackbars(window_name="HSV Adjust"):
    """สร้าง Trackbar สำหรับปรับค่า HSV"""
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 400, 300)
    cv2.createTrackbar("H Low",  window_name, HSV_LOW[0],  179, nothing)
    cv2.createTrackbar("S Low",  window_name, HSV_LOW[1],  255, nothing)
    cv2.createTrackbar("V Low",  window_name, HSV_LOW[2],  255, nothing)
    cv2.createTrackbar("H High", window_name, HSV_HIGH[0], 179, nothing)
    cv2.createTrackbar("S High", window_name, HSV_HIGH[1], 255, nothing)
    cv2.createTrackbar("V High", window_name, HSV_HIGH[2], 255, nothing)


def get_trackbar_values(window_name="HSV Adjust"):
    """อ่านค่า HSV จาก Trackbar"""
    h_low  = cv2.getTrackbarPos("H Low",  window_name)
    s_low  = cv2.getTrackbarPos("S Low",  window_name)
    v_low  = cv2.getTrackbarPos("V Low",  window_name)
    h_high = cv2.getTrackbarPos("H High", window_name)
    s_high = cv2.getTrackbarPos("S High", window_name)
    v_high = cv2.getTrackbarPos("V High", window_name)
    return np.array([h_low, s_low, v_low]), np.array([h_high, s_high, v_high])


def segment_leaves(frame, hsv_low, hsv_high):
    """
    แยกใบไม้ออกจากพื้นหลังด้วย HSV segmentation
    Returns: mask (binary image)
    """
    # แปลงเป็น HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # สร้าง mask สำหรับสีเขียว
    mask = cv2.inRange(hsv, hsv_low, hsv_high)

    # ลด noise ด้วย morphology
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    # เติมรู (fill holes)
    kernel_big = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_big, iterations=1)

    return mask


def analyze_leaves(frame, mask, px_per_cm=None):
    """
    วิเคราะห์ใบไม้จาก mask
    Returns: (annotated_frame, leaf_data_list)
    """
    result = frame.copy()
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    leaf_data = []
    leaf_id = 0

    for contour in contours:
        area_px = cv2.contourArea(contour)
        if area_px < MIN_CONTOUR_AREA:
            continue

        leaf_id += 1

        # Bounding box
        x, y, w, h = cv2.boundingRect(contour)

        # Minimum area rotated rectangle
        rect = cv2.minAreaRect(contour)
        box = cv2.boxPoints(rect)
        box = np.int32(box)
        rect_w = min(rect[1])
        rect_h = max(rect[1])

        # Convex hull
        hull = cv2.convexHull(contour)
        hull_area = cv2.contourArea(hull)

        # Perimeter
        perimeter_px = cv2.arcLength(contour, True)

        # สร้างข้อมูลใบ
        info = {
            "id": leaf_id,
            "area_px": area_px,
            "perimeter_px": perimeter_px,
            "width_px": rect_w,
            "height_px": rect_h,
            "bbox": (x, y, w, h),
            "solidity": area_px / hull_area if hull_area > 0 else 0,
        }

        # แปลงเป็น cm ถ้ามี calibration
        if px_per_cm and px_per_cm > 0:
            px_per_cm_sq = px_per_cm ** 2
            info["area_cm2"] = area_px / px_per_cm_sq
            info["perimeter_cm"] = perimeter_px / px_per_cm
            info["width_cm"] = rect_w / px_per_cm
            info["height_cm"] = rect_h / px_per_cm

        leaf_data.append(info)

        # --- วาดผลลัพธ์ ---

        # วาด contour + rotated rect
        color = (0, 255, 100)
        cv2.drawContours(result, [contour], -1, color, 2)
        cv2.drawContours(result, [box], 0, (255, 200, 0), 2)

        # Label
        if px_per_cm and px_per_cm > 0:
            label1 = f"Leaf {leaf_id}: {info['area_cm2']:.1f} cm2"
            label2 = f"{info['width_cm']:.1f}x{info['height_cm']:.1f} cm"
        else:
            label1 = f"Leaf {leaf_id}: {int(area_px)} px2"
            label2 = f"{int(rect_w)}x{int(rect_h)} px"

        # พื้นหลังข้อความ
        cv2.rectangle(result, (x, y - 45), (x + max(len(label1), len(label2)) * 10 + 10, y - 2),
                      (0, 0, 0), -1)
        cv2.putText(result, label1, (x + 4, y - 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 100), 1, cv2.LINE_AA)
        cv2.putText(result, label2, (x + 4, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1, cv2.LINE_AA)

    # Green Coverage
    total_pixels = mask.shape[0] * mask.shape[1]
    green_pixels = cv2.countNonZero(mask)
    coverage = green_pixels / total_pixels * 100

    # Summary bar ด้านบน
    bar_h = 70
    overlay = result.copy()
    cv2.rectangle(overlay, (0, 0), (result.shape[1], bar_h), (30, 30, 30), -1)
    result = cv2.addWeighted(overlay, 0.7, result, 0.3, 0)

    cv2.putText(result, f"Leaves: {leaf_id}", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 100), 2, cv2.LINE_AA)
    cv2.putText(result, f"Green Coverage: {coverage:.1f}%", (10, 55),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 255, 200), 1, cv2.LINE_AA)

    if leaf_data:
        total_area = sum(l["area_px"] for l in leaf_data)
        if px_per_cm and px_per_cm > 0:
            total_area_cm = total_area / (px_per_cm ** 2)
            area_text = f"Total Leaf Area: {total_area_cm:.1f} cm2"
        else:
            area_text = f"Total Leaf Area: {int(total_area)} px2"
        cv2.putText(result, area_text, (300, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 100), 2, cv2.LINE_AA)

    return result, leaf_data


def print_leaf_report(leaf_data, px_per_cm=None):
    """พิมพ์รายงานใบไม้"""
    print("\n" + "=" * 50)
    print(f"  🌿 Leaf Analysis Report — {len(leaf_data)} leaves found")
    print("=" * 50)

    for leaf in leaf_data:
        print(f"\n  Leaf #{leaf['id']}:")
        if "area_cm2" in leaf:
            print(f"    Area:      {leaf['area_cm2']:.2f} cm²")
            print(f"    Size:      {leaf['width_cm']:.1f} × {leaf['height_cm']:.1f} cm")
            print(f"    Perimeter: {leaf['perimeter_cm']:.1f} cm")
        else:
            print(f"    Area:      {int(leaf['area_px'])} px²")
            print(f"    Size:      {int(leaf['width_px'])} × {int(leaf['height_px'])} px")
            print(f"    Perimeter: {int(leaf['perimeter_px'])} px")
        print(f"    Solidity:  {leaf['solidity']:.2f}")

    if leaf_data:
        total = sum(l["area_px"] for l in leaf_data)
        if "area_cm2" in leaf_data[0]:
            total_cm = sum(l["area_cm2"] for l in leaf_data)
            print(f"\n  📊 Total leaf area: {total_cm:.2f} cm²")
        else:
            print(f"\n  📊 Total leaf area: {int(total)} px²")

    print("=" * 50 + "\n")


def main():
    parser = argparse.ArgumentParser(description="🌿 Leaf Size Detector")
    parser.add_argument("--camera", type=int, default=0, help="Camera index (default: 0)")
    parser.add_argument("--ref-width", type=float, default=0,
                        help="Reference object width in cm for calibration (0 = pixel mode)")
    parser.add_argument("--save-dir", type=str, default="captures",
                        help="Directory to save captures (default: captures)")
    args = parser.parse_args()

    # เปิดกล้อง
    print(f"📷 Opening camera {args.camera}...")
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print("❌ ไม่สามารถเปิดกล้องได้!")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"✅ Camera opened: {actual_w}×{actual_h}")

    # Calibration
    px_per_cm = None
    if args.ref_width > 0:
        print(f"📏 Calibration mode: reference object = {args.ref_width} cm wide")
        print("   Place reference object in frame, press SPACE to calibrate")

    # State
    show_trackbar = False
    captured_frame = None
    mode = "live"  # live / captured

    print("\n🎮 Controls:")
    print("   SPACE = Capture + Analyze")
    print("   T     = Toggle HSV Trackbar")
    print("   S     = Save result")
    print("   R     = Reset to live")
    print("   Q/ESC = Quit\n")

    os.makedirs(args.save_dir, exist_ok=True)

    hsv_low = HSV_LOW.copy()
    hsv_high = HSV_HIGH.copy()

    while True:
        # อ่านเฟรม
        if mode == "live":
            ret, frame = cap.read()
            if not ret:
                print("❌ อ่านเฟรมไม่ได้")
                break
        else:
            frame = captured_frame.copy()

        # อ่านค่า HSV จาก trackbar (ถ้าเปิด)
        if show_trackbar:
            hsv_low, hsv_high = get_trackbar_values()

        # Segment ใบ
        mask = segment_leaves(frame, hsv_low, hsv_high)

        # สร้าง display
        if mode == "captured":
            display, leaf_data = analyze_leaves(frame, mask, px_per_cm)
            cv2.putText(display, "[CAPTURED] Press R to reset", (10, display.shape[0] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1, cv2.LINE_AA)
        else:
            # Live mode — แสดง mask preview แบบเบาๆ
            display = frame.copy()

            # สร้าง overlay เขียวจาง แสดงส่วนที่ detect ได้
            green_overlay = np.zeros_like(frame)
            green_overlay[:] = (0, 200, 100)
            mask_3ch = cv2.merge([mask, mask, mask])
            green_overlay = cv2.bitwise_and(green_overlay, mask_3ch)
            display = cv2.addWeighted(display, 1.0, green_overlay, 0.3, 0)

            # แสดง contour แบบเบา
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            leaf_count = sum(1 for c in contours if cv2.contourArea(c) >= MIN_CONTOUR_AREA)
            cv2.drawContours(display, [c for c in contours if cv2.contourArea(c) >= MIN_CONTOUR_AREA],
                             -1, (0, 255, 100), 1)

            # Info
            cv2.putText(display, f"LIVE | Detected: {leaf_count} leaves | Press SPACE to analyze",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 100), 2, cv2.LINE_AA)

            if px_per_cm:
                cv2.putText(display, f"Calibrated: {px_per_cm:.1f} px/cm",
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 100), 1, cv2.LINE_AA)

        # แสดง mask ในหน้าต่างเล็ก
        mask_small = cv2.resize(mask, (320, 180))
        mask_colored = cv2.cvtColor(mask_small, cv2.COLOR_GRAY2BGR)
        cv2.putText(mask_colored, "Mask", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        cv2.imshow("Leaf Detector", display)
        cv2.imshow("Mask", mask_colored)

        # ปุ่มกด
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q') or key == 27:  # Q / ESC
            break

        elif key == ord(' '):  # SPACE — Capture + Analyze
            ret, fresh_frame = cap.read()
            if ret:
                captured_frame = fresh_frame.copy()
                mode = "captured"
                mask = segment_leaves(captured_frame, hsv_low, hsv_high)
                _, leaf_data = analyze_leaves(captured_frame, mask, px_per_cm)
                print_leaf_report(leaf_data, px_per_cm)

        elif key == ord('t') or key == ord('T'):  # Toggle trackbar
            show_trackbar = not show_trackbar
            if show_trackbar:
                create_trackbars()
                print("🎛️  HSV Trackbar ON")
            else:
                cv2.destroyWindow("HSV Adjust")
                print("🎛️  HSV Trackbar OFF")

        elif key == ord('s') or key == ord('S'):  # Save
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(args.save_dir, f"leaf_{timestamp}.jpg")
            save_frame = display if mode == "captured" else frame
            cv2.imwrite(filename, save_frame)
            # บันทึก mask ด้วย
            mask_filename = os.path.join(args.save_dir, f"mask_{timestamp}.jpg")
            cv2.imwrite(mask_filename, mask)
            print(f"💾 Saved: {filename}")

        elif key == ord('r') or key == ord('R'):  # Reset
            mode = "live"
            captured_frame = None
            print("🔄 Reset to live mode")

        elif key == ord('k') or key == ord('K'):  # Calibrate px/cm
            if mode == "captured" and captured_frame is not None and args.ref_width > 0:
                print("📏 Click on the LEFT edge of reference object, then RIGHT edge")
                # จะต้อง implement mouse callback — ข้ามไว้ก่อน ใช้ --ref-width แทน

    cap.release()
    cv2.destroyAllWindows()
    print("👋 Done!")


if __name__ == "__main__":
    main()
