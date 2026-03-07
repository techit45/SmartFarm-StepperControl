# Stepper Motor Control — ESP32

ระบบควบคุม Stepper Motor ผ่าน Serial สำหรับ ESP32

## 🔌 การต่อสาย

| อุปกรณ์               | Pin ESP32 | หมายเหตุ              |
| --------------------- | --------- | --------------------- |
| PUL+ (Stepper Driver) | GND       |                       |
| DIR+ (Stepper Driver) | GND       |                       |
| PUL-                  | GPIO 27   | Pulse signal          |
| DIR-                  | GPIO 14   | Direction signal      |
| Limit Switch Start    | GPIO 16   | INPUT_PULLUP (NO→GND) |
| Limit Switch End      | GPIO 17   | INPUT_PULLUP (NO→GND) |

## 🚀 Serial Commands (115200 baud)

| คำสั่ง            | ตัวอย่าง     | อธิบาย                              |
| ----------------- | ------------ | ----------------------------------- |
| `calibrate`       | `calibrate`  | Home → วัดระยะ → กลับ Home          |
| `home`            | `home`       | กลับตำแหน่ง Start                   |
| `move <steps>`    | `move 500`   | เลื่อนสัมพัทธ์ (+ไป End, -ไป Start) |
| `goto <position>` | `goto 1000`  | ไปตำแหน่งสัมบูรณ์                   |
| `speed <sps>`     | `speed 2000` | ตั้งความเร็ว (steps/sec)            |
| `status`          | `status`     | แสดงสถานะ                           |
| `stop`            | `stop`       | หยุดทันที                           |

## 🔄 Calibration Flow

1. เลื่อนไปหา **Start** limit switch (Homing)
2. ถอยออกจาก switch เล็กน้อย → ตั้งตำแหน่ง = 0
3. เลื่อนไปหา **End** limit switch → นับ steps = ระยะทางทั้งหมด
4. กลับ Home (position = 0)
