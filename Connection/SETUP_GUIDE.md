# UAV → UGV GPS-Denied Navigation: Setup & Testing Guide

**System:** Jetson Nano (UAV) + ESP32 pair (ESP-NOW bridge) + Raspberry Pi 5 (UGV) + ArduPilot

---

## Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [Part A — PC Testing with SITL (No Hardware)](#part-a--pc-testing-with-sitl-no-hardware)
3. [Part B — Hardware Setup & Wiring](#part-b--hardware-setup--wiring)
4. [Part C — ArduPilot Parameter Configuration](#part-c--ardupilot-parameter-configuration)
5. [Part D — Running the Full System](#part-d--running-the-full-system)
6. [Troubleshooting](#troubleshooting)

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                          UAV (Drone)                            │
│                                                                 │
│  [ArduCopter FC] ──USB/UART──▶ [Jetson Nano]                   │
│   LOCAL_POSITION_NED           jetson_uav_sender.py             │
│   (EKF from IMU+baro)          builds JSON packet               │
│                                        │                        │
│                                   UART TX/RX                    │
│                                        ▼                        │
│                                [ESP32 server.cpp]               │
│                                                                 │
└───────────────────────────────────────┬─────────────────────────┘
                                        │
                                   ESP-NOW 2.4 GHz
                                   (replaces Wi-Fi/GPS link)
                                        │
┌───────────────────────────────────────▼─────────────────────────┐
│                          UGV (Rover)                            │
│                                                                 │
│                                [ESP32 client.cpp]               │
│                                        │                        │
│                                   UART TX/RX                    │
│                                        ▼                        │
│  [ArduRover FC] ◀──USB/UART── [Raspberry Pi 5]                 │
│   GUIDED velocity cmds         rpi_ugv_navigator.py             │
│   LOCAL_POSITION_NED           drives UGV to UAV's coords       │
│   (EKF from wheel encoders)                                     │
└─────────────────────────────────────────────────────────────────┘
```

### Data flow (JSON packet)
```
{"x":5.23,"y":1.87,"z":-9.8,"vx":0.2,"vy":0.0,"t":128450}
  │         │        │        └────────── UAV velocity (m/s)
  │         │        └─────────────────── Altitude below origin (m, negative = above)
  └─────────┴──────────────────────────── UAV North/East position in LOCAL NED frame (m)
```

---

## Part A — PC Testing with SITL (No Hardware)

Test the complete navigation logic on your PC using ArduPilot SITL — no ESP32, no Jetson, no RPi needed.

```
[ArduCopter SITL] ──MAVLink UDP:14550──▶ [sitl_uav_sender.py]
                                                  │
                                             TCP :5555
                                                  ▼
                                    [virtual_esp_bridge.py]
                                                  │
                                             TCP :5556
                                                  ▼
                                    [sitl_ugv_navigator.py]
                                          │
                                   MAVLink UDP:14560
                                          ▼
                               [ArduRover SITL]
```

### A.1 — Install Prerequisites

**Python packages** (all platforms):
```bash
pip install pymavlink pyserial
```

**ArduPilot SITL** (Linux recommended; WSL2 works on Windows):
```bash
# Install dependencies
sudo apt-get update
sudo apt-get install -y git python3 python3-pip python3-dev \
    gcc g++ make gawk libxml2-dev libxslt-dev python3-lxml

# Clone ArduPilot
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot

# Install Python requirements
pip3 install -r requirements.txt

# Set up the SITL build environment
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile

# Build ArduCopter SITL (UAV)
cd ArduCopter
sim_vehicle.py -v ArduCopter --no-rebuild --help   # verifies install
```

**MAVProxy** (installed automatically with ardupilot, or separately):
```bash
pip install MAVProxy
```

### A.2 — Open 4 Terminal Windows

You will run 4 processes simultaneously. Open 4 separate terminal tabs/windows and label them.

### A.3 — Terminal 1: Start ArduCopter SITL (UAV)

```bash
cd ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter --console --map -L CMAC --out udp:127.0.0.1:14550
```

Wait for the console to show:
```
APM: EKF2 IMU0 initial yaw alignment complete
APM: EKF2 IMU0 is using GPS
```

> **Tip:** The `-L CMAC` flag starts at Canberra Model Aircraft Club (ArduPilot default test location). Any location works since we use LOCAL_POSITION_NED, not GPS coordinates.

### A.4 — Terminal 2: Start ArduRover SITL (UGV)

```bash
cd ardupilot/Rover
sim_vehicle.py -v Rover --console --map -L CMAC --out udp:127.0.0.1:14560
```

Wait for:
```
APM: EKF3 IMU0 is using GPS
```

> **Note:** Both SITLs start at the same GPS location (CMAC), which means their LOCAL_POSITION_NED origins are aligned — exactly how you will set up hardware (both powered up at the same spot).

### A.5 — Terminal 3: Start the Virtual ESP Bridge

```bash
python virtual_esp_bridge.py
```

Expected output:
```
============================================================
  Virtual ESP32 Bridge  —  PC SITL Testing
============================================================
  UAV sender   →  connect to 127.0.0.1:5555
  UGV navigator→  connect to 127.0.0.1:5556
  Simulated RF delay: 10 ms
============================================================
[BRIDGE] UAV receiver listening on 127.0.0.1:5555
[BRIDGE] UGV sender listening on 127.0.0.1:5556
[BRIDGE] All threads running. Press Ctrl-C to stop.
```

**Optional — Run loopback test first** (no SITL required):
```bash
# In a new terminal, with virtual_esp_bridge.py already running:
python bridge_loopback_test.py
```
Expected:
```
[RESULT] 10/10 packets relayed correctly ✓
```

### A.6 — Terminal 4: Start UGV Navigator (before UAV sender)

Start the UGV side first so it is ready to receive:
```bash
python sitl_ugv_navigator.py
```

Watch for:
```
[UGV-SITL] Heartbeat OK  sys=1  comp=1
[TCP] Connected to bridge at 127.0.0.1:5556
[UGV-SITL] Mode → GUIDED
[UGV-SITL] Armed.
[NAV] Navigation loop started.
[NAV] Waiting for first UAV position...
```

### A.7 — Terminal 5 (or reuse a free one): Start UAV Sender

```bash
python sitl_uav_sender.py
```

Watch for:
```
[UAV-SITL] Heartbeat OK  sys=1  comp=1
[UAV-SITL] Connected to bridge at 127.0.0.1:5555
[TX #0001] {"x":0.000,"y":0.000,"z":0.000,"vx":0.0,"vy":0.0,"t":12345}
```

### A.8 — Verify End-to-End Flow

In the UGV navigator terminal you should now see:
```
[RX] UAV → x=0.000 m N  y=0.000 m E
[NAV] [AT_TARGET] UGV=(0.00,0.00)  UAV=(0.00,0.00)  dist=0.00m  age=0.4s
```

**To test movement:** In the ArduCopter SITL console, fly the UAV north:
```
GUIDED
arm throttle
takeoff 10
velocity 2 0 0    # fly north at 2 m/s
```

You should see the UGV SITL map window start driving north in response.

### A.9 — Shutdown Order

1. Ctrl-C `sitl_uav_sender.py`
2. Ctrl-C `sitl_ugv_navigator.py`
3. Ctrl-C `virtual_esp_bridge.py`
4. Ctrl-C both SITL instances

---

## Part B — Hardware Setup & Wiring

### B.1 — Hardware Required

| Item | Qty | Notes |
|------|-----|-------|
| ESP32 DevKit (38-pin) | 2 | One per vehicle. Any ESP32 with WiFi works. |
| Jetson Nano 4GB | 1 | UAV compute board |
| Raspberry Pi 5 (4/8 GB) | 1 | UGV compute board |
| ArduPilot FC (Pixhawk, etc.) | 2 | One per vehicle |
| USB-A to Micro-USB / USB-C | 2 | FC to Jetson/RPi |
| Jumper wires | 6+ | For UART connections |
| 3.3V logic-level converter | 2 | **Required** — ESP32 is 3.3V, RPi is 3.3V (compatible), Jetson GPIO is 3.3V (compatible) |

> ⚠️ **Voltage Warning:** Jetson Nano and RPi5 GPIO are 3.3V. ESP32 is also 3.3V. The flight controller TELEM port is typically 3.3V. All are compatible. Do **NOT** connect 5V logic directly to ESP32 or GPIO pins.

### B.2 — UAV Wiring (Jetson Nano ↔ ESP32 server.cpp)

```
Jetson Nano                    ESP32 (server.cpp)
────────────────               ──────────────────
Pin 8  (UART TX) ─────────────▶ Pin 16 (GPIO16 / RX2)
Pin 10 (UART RX) ◀───────────── Pin 17 (GPIO17 / TX2)
Pin 6  (GND)     ───────────── GND
Pin 1  (3.3V)    (DO NOT CONNECT — power ESP32 from USB separately)

FC USB port      ──USB cable──▶ Jetson USB-A port  (/dev/ttyACM0)
```

> **Jetson UART port:** `/dev/ttyTHS1` (GPIO pins 8/10) or use a USB-serial adapter on `/dev/ttyUSB0`.  
> Enable UART: `sudo systemctl disable nvgetty`

**In `jetson_uav_sender.py` set:**
```python
FC_PORT  = "/dev/ttyACM0"    # Flight controller
ESP_PORT = "/dev/ttyTHS1"    # GPIO UART to ESP32
```

### B.3 — UGV Wiring (Raspberry Pi 5 ↔ ESP32 client.cpp)

```
Raspberry Pi 5                 ESP32 (client.cpp)
──────────────────             ──────────────────
Pin 8  (GPIO14/TX) ───────────▶ Pin 5  (GPIO5  / RX, as configured in client.cpp)
Pin 10 (GPIO15/RX) ◀─────────── Pin 4  (GPIO4  / TX, as configured in client.cpp)
Pin 6  (GND)       ─────────── GND
(Power ESP32 from USB separately)

FC USB port        ──USB cable──▶ RPi5 USB-A port  (/dev/ttyACM0)
```

> **RPi5 UART:** Enable `/dev/ttyAMA0` via `raspi-config` → Interface Options → Serial Port.  
> Select: "No" to login shell, "Yes" to serial hardware.

**In `rpi_ugv_navigator.py` set:**
```python
ESP_PORT = "/dev/ttyAMA0"    # GPIO UART from ESP32
FC_PORT  = "/dev/ttyACM0"    # Flight controller
```

### B.4 — ESP32 Flashing Instructions

**Install Arduino IDE + ESP32 board support:**

1. Download Arduino IDE from [arduino.cc](https://www.arduino.cc/en/software)
2. In Arduino IDE → File → Preferences → Additional Boards Manager URLs, add:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
3. Tools → Board → Boards Manager → search "esp32" → install "esp32 by Espressif Systems"

**Find MAC addresses (do this before final flash):**

1. Flash a minimal MAC-printer sketch to each ESP32:
```cpp
#include <WiFi.h>
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  Serial.println(WiFi.macAddress());
}
void loop() {}
```
2. Open Serial Monitor at 115200 baud
3. Note the MAC for each ESP32

**Flash UAV ESP32 (server.cpp):**
1. Open `server.cpp` in Arduino IDE
2. Update `UGV_MAC[]` with the UGV ESP32's MAC address
3. Tools → Board → "ESP32 Dev Module"
4. Tools → Port → select the COM port / ttyUSB0
5. Upload

**Flash UGV ESP32 (client.cpp):**
1. Open `client.cpp` in Arduino IDE
2. Update `ESPNOW_SENDER_MAC[]` with the UAV ESP32's MAC address
3. Upload the same way

### B.5 — Verify ESP32 Communication

1. Power both ESP32s
2. Open Serial Monitor on UGV ESP32 (client.cpp)
3. Open Serial Monitor on UAV ESP32 (server.cpp)
4. Type any text + Enter into the UAV serial monitor

Expected in UGV serial monitor:
```
[RX] {"x":1.23,...}
[ESP-NOW] Send status: OK
```

### B.6 — Raspberry Pi 5 Setup

```bash
# Install Python deps
pip3 install pymavlink pyserial

# Enable UART (if not done via raspi-config)
echo "enable_uart=1" | sudo tee -a /boot/firmware/config.txt
sudo systemctl disable hciuart  # Free UART from Bluetooth if needed
sudo reboot

# Verify UART is available
ls /dev/ttyAMA*   # Should show /dev/ttyAMA0

# Test serial from ESP32 (monitor raw data)
python3 -c "
import serial
s = serial.Serial('/dev/ttyAMA0', 115200, timeout=2)
while True:
    line = s.readline()
    if line: print(repr(line))
"
```

### B.7 — Jetson Nano Setup

```bash
# Install Python deps
pip3 install pymavlink pyserial

# Disable nvgetty to free UART
sudo systemctl disable nvgetty
sudo systemctl stop nvgetty
sudo reboot

# Verify UART
ls /dev/ttyTHS*   # Should show /dev/ttyTHS1

# Test FC MAVLink connection
python3 -c "
from pymavlink import mavutil
m = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
m.wait_heartbeat()
print('FC connected: sys', m.target_system)
"
```

---

## Part C — ArduPilot Parameter Configuration

### C.1 — GPS-Denied GUIDED for ArduRover (UGV)

Connect to the UGV rover FC via Mission Planner or QGroundControl and set these parameters:

**EKF Source (GPS-denied, wheel encoder odometry):**

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `EK3_SRC1_POSXY` | 5 | Wheel encoders for XY position |
| `EK3_SRC1_VELXY` | 5 | Wheel encoders for XY velocity |
| `EK3_SRC1_POSZ` | 1 | Barometer for Z |
| `EK3_SRC1_VELZ` | 0 | None |
| `EK3_SRC1_YAW` | 1 | Compass |

**Wheel encoder enable:**

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `WHEEL_ENC_ENABLE` | 1 | Enable wheel encoder |
| `WHEEL_ENC_RADIUS` | *measured* | Wheel radius in meters |
| `WHEEL_ENC_POS_X` | *measured* | Encoder X offset from CG |

**GUIDED mode without GPS:**

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `GPS_TYPE` | 0 | Disable GPS entirely (forces EKF to use other sources) |
| `ARMING_CHECK` | 0 | Disable arming checks (useful for initial testing only — re-enable after) |
| `AHRS_EKF_TYPE` | 3 | Use EKF3 |

> **Note:** If you have a Pixhawk with no wheel encoders, alternative is to use visual odometry (EK3_SRC1_POSXY=6) or an optical flow sensor (EK3_SRC1_POSXY=5 with OF sensor).

### C.2 — ArduCopter UAV Parameters

The UAV just needs LOCAL_POSITION_NED working reliably. For GPS-enabled UAV (typical):

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `EK3_ENABLE` | 1 | Use EKF3 |
| `AHRS_EKF_TYPE` | 3 | Use EKF3 |
| `EK2_ENABLE` | 0 | Disable EKF2 |

For the Jetson MAVLink connection via USB (Telem2 / USB):

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `SERIAL0_BAUD` | 115 | 115200 baud on USB |
| `SERIAL0_PROTOCOL` | 2 | MAVLink 2 |

---

## Part D — Running the Full System

### D.1 — Pre-Flight Checklist

- [ ] Both vehicles powered on at the **same physical location** (EKF origin alignment)
- [ ] Both FC systems show **EKF initialised** (no EKF errors in GCS)
- [ ] UGV FC shows LOCAL_POSITION_NED updating (check in Mission Planner)
- [ ] UAV FC shows LOCAL_POSITION_NED updating
- [ ] Both ESP32s powered and show no error on serial monitor
- [ ] Jetson Nano boot complete, SSH accessible
- [ ] RPi5 boot complete, SSH accessible

### D.2 — Boot Order

Follow this order every time:

```
Step 1:  Power on UGV + UGV FC (wait 30 s for EKF to warm up)
Step 2:  Power on UAV + UAV FC (wait 30 s for EKF to warm up)
         ── Both at same location ──
Step 3:  Power on both ESP32s (they auto-connect via ESP-NOW on boot)
Step 4:  On RPi5:     python3 rpi_ugv_navigator.py
Step 5:  On Jetson:   python3 jetson_uav_sender.py
Step 6:  Verify JSON is flowing (check terminal output on both boards)
Step 7:  Arm UGV rover — it will now track UAV position
Step 8:  Take off / move UAV
```

### D.3 — Running on Jetson Nano

```bash
ssh user@jetson-nano-ip

# Start sender (runs in foreground; use screen/tmux for persistence)
screen -S uav_sender
python3 jetson_uav_sender.py

# Detach: Ctrl-A, D
# Reattach: screen -r uav_sender
```

Expected output:
```
[UAV] Connecting to flight controller on /dev/ttyACM0 @ 115200 baud...
[UAV] Heartbeat OK  sys=1  comp=1
[UAV] Opening ESP32 serial on /dev/ttyTHS1 @ 115200 baud...
[UAV] ESP32 serial open.
[UAV] Streaming position at 2 Hz. Press Ctrl-C to stop.

[TX #1] {"x":0.023,"y":-0.001,"z":-0.004,"vx":0.0,"vy":0.0,"t":18432}
[TX #2] {"x":0.023,"y":-0.001,"z":-0.004,"vx":0.0,"vy":0.0,"t":18932}
```

### D.4 — Running on Raspberry Pi 5

```bash
ssh user@raspberrypi-ip

screen -S ugv_nav
python3 rpi_ugv_navigator.py
```

Expected output:
```
[SERIAL] Opening /dev/ttyAMA0 @ 115200 baud...
[SERIAL] ESP32 UART open — waiting for packets...
[FC] Connecting to flight controller on /dev/ttyACM0 @ 115200 baud...
[FC] Heartbeat OK  sys=1  comp=1
[FC] Mode → GUIDED
[FC] Armed.
[RX] UAV position  x=0.023 m N  y=-0.001 m E
[NAV] [AT_TARGET] UGV=(0.02,0.00)  UAV=(0.02,0.00)  dist=0.01m  age=0.3s
```

---

## Troubleshooting

### "No LOCAL_POSITION_NED" on Jetson

The UAV EKF hasn't initialised. Check:
- FC status in GCS — look for EKF errors
- `request_message_streams()` is being called
- FC TELEM port baud rate matches `FC_BAUD` in the script

### "Bridge not ready" in SITL scripts

Start `virtual_esp_bridge.py` **before** the sender/navigator scripts. It needs to bind its ports first.

### UGV doesn't move after receiving position

1. Confirm GUIDED mode is active: check GCS heartbeat mode
2. Confirm rover is armed: `MAV_MODE_FLAG_SAFETY_ARMED` in heartbeat
3. Check `ARMING_CHECK=0` if GPS is disabled and arming fails
4. Confirm LOCAL_POSITION_NED is available on UGV side: look for `[WARN] No LOCAL_POSITION_NED` in navigator output

### ESP-NOW packets not arriving on UGV

1. Both ESP32s must be on the **same Wi-Fi channel**. If a router is nearby and one auto-connects, the channel may drift. Set `peerInfo.channel = 1` (or any fixed value 1–13) in `server.cpp` and `client.cpp` to lock both to the same channel.
2. Confirm MAC addresses are correct — run the MAC printer sketch if unsure.
3. Distance: ESP-NOW range is ~200 m line-of-sight outdoors. Reduce to test.

### UGV overshoots the target

Reduce `SPEED_MPS` in `rpi_ugv_navigator.py` or increase `ARRIVE_DIST_M`. The proportional speed control will slow down on approach but there is no deceleration ramp — add one if needed:

```python
# Replace the speed calculation in navigate() with:
scaled_speed = min(speed_mps, dist * 1.0)   # Slow to dist*1.0 m/s within 1 m
scaled_speed = max(scaled_speed, 0.1)        # Never slower than 0.1 m/s
vx = (ex / dist) * scaled_speed
vy = (ey / dist) * scaled_speed
```

### Position drift (EKF origin mismatch)

If the UGV drives to the wrong location, the two EKF origins aren't aligned. To verify:

1. Before moving either vehicle, read `LOCAL_POSITION_NED` from both FCs
2. Both should be near `x=0, y=0`
3. If not, power both down, place them side-by-side, and power on together

### Serial port permission denied

```bash
sudo usermod -aG dialout $USER
sudo usermod -aG tty $USER
# Log out and back in, or:
newgrp dialout
```

---

## File Reference

| File | Runs On | Purpose |
|------|---------|---------|
| `jetson_uav_sender.py` | Jetson Nano | Reads UAV position from FC, sends JSON to ESP32 |
| `rpi_ugv_navigator.py` | Raspberry Pi 5 | Reads JSON from ESP32, drives UGV to UAV position |
| `server.cpp` | UAV ESP32 | Receives JSON from Jetson UART, sends via ESP-NOW |
| `client.cpp` | UGV ESP32 | Receives ESP-NOW, forwards to RPi UART |
| `sitl_uav_sender.py` | PC (testing) | SITL version of jetson_uav_sender.py using TCP |
| `sitl_ugv_navigator.py` | PC (testing) | SITL version of rpi_ugv_navigator.py using TCP |
| `virtual_esp_bridge.py` | PC (testing) | Simulates full ESP-NOW + serial chain via TCP |
| `bridge_loopback_test.py` | PC (testing) | Quick sanity check for virtual_esp_bridge.py |
