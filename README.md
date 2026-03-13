# Smart Farm — ESP32 Stepper + Pump + Soil Moisture

ระบบ Smart Farm ควบคุม Stepper Motor, ปั้มน้ำ, และ Soil Moisture Sensor ผ่าน Serial สำหรับ ESP32

## 🔌 การต่อสาย

### Stepper Motor Driver

| อุปกรณ์               | Pin ESP32 | หมายเหตุ              |
| --------------------- | --------- | --------------------- |
| PUL+ (Stepper Driver) | GND       |                       |
| DIR+ (Stepper Driver) | GND       |                       |
| PUL-                  | GPIO 27   | Pulse signal          |
| DIR-                  | GPIO 14   | Direction signal      |
| Limit Switch Start    | GPIO 16   | INPUT_PULLUP (NO→GND) |
| Limit Switch End      | GPIO 17   | INPUT_PULLUP (NO→GND) |

### Relay (ปั้มน้ำ) — Active LOW

| อุปกรณ์ | Pin ESP32 | หมายเหตุ         |
| ------- | --------- | ---------------- |
| Pump 1  | GPIO 12   | Relay Active LOW |
| Pump 2  | GPIO 13   | Relay Active LOW |

### Soil Moisture Sensor (Analog)

| อุปกรณ์  | Pin ESP32 | หมายเหตุ      |
| -------- | --------- | ------------- |
| Sensor 1 | GPIO 32   | Analog 0-4095 |
| Sensor 2 | GPIO 33   | Analog 0-4095 |

## 🚀 Serial Commands (115200 baud)

| คำสั่ง                | ตัวอย่าง     | อธิบาย                                 |
| --------------------- | ------------ | -------------------------------------- |
| `calibrate`           | `calibrate`  | Home → วัดระยะ → กลับ Home             |
| `home`                | `home`       | กลับตำแหน่ง Start                      |
| `move <steps>`        | `move 500`   | เลื่อนสัมพัทธ์ (+ไป End, -ไป Start)    |
| `goto <position>`     | `goto 1000`  | ไปตำแหน่งสัมบูรณ์                      |
| `speed <sps>`         | `speed 2000` | ตั้งความเร็ว (steps/sec)               |
| `status`              | `status`     | แสดงสถานะ                              |
| `stop`                | `stop`       | หยุดทันที                              |
| `pump <1/2> <on/off>` | `pump 1 on`  | เปิด/ปิดปั้มน้ำ                        |
| `soil`                | `soil`       | อ่านค่า soil moisture                  |
| `sensors`             | `sensors`    | อ่าน sensor ทั้งหมด (machine-readable) |

## 🔄 Calibration Flow

1. เลื่อนไปหา **Start** limit switch (Homing)
2. ถอยออกจาก switch เล็กน้อย → ตั้งตำแหน่ง = 0
3. เลื่อนไปหา **End** limit switch → นับ steps = ระยะทางทั้งหมด
4. กลับ Home (position = 0)
