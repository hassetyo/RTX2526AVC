# RTX2526AVC

This repository contains UAV, UGV, simulation, and radio-bridge code for a cooperative autonomous vehicle challenge workflow. The project is organized as mission scripts and experiments rather than a single packaged application, so the right entrypoint depends on whether you are testing flight, ground movement, comms, or simulation.

## Overview

The codebase covers:

- UAV flight control over MAVLink with `pymavlink` and, in some scripts, `dronekit`
- UGV drive control and challenge-specific navigation flows
- ArUco-based vision for search, tracking, rendezvous, and landing
- ESP32-to-ESP32 vehicle-to-vehicle messaging over ESP-NOW
- Webots simulation for validating UAV/UGV behavior before hardware runs
- Qualification and mission test scripts for isolated subsystem validation

From the script headers and directory layout, the main mission themes are:

- Challenge 1: UAV takes off from the UGV, tracks it, and lands back on a moving UGV
- Challenge 2: UAV searches for a goal marker, relays coordinates to the UGV, then returns to reconnect
- Challenge 3: UGV navigation with obstacle detection and avoidance

## Repository Layout

| Path | Purpose |
| --- | --- |
| `UAV_Flight/` | UAV mission scripts, camera workflows, ArUco landing/search logic, and flight experiments |
| `UGV_Drive/` | UGV challenge scripts, waypoint driving, lidar integration, camera tests, and the Python V2V bridge |
| `Connection/` | ESP32 firmware and C++/local bridge experiments for serial and ESP-NOW communication |
| `Sim/` | Webots world, robot controllers, and Python simulation scripts for UAV/UGV testing |
| `Mission_Tests/` | Focused mission validation scripts for guided, stabilize, and mixed UAV/UGV flows |
| `Qual_Tests/` | Safety and qualification checks such as safe-land and kill-switch behavior |
| `TrackingCoordsTest.py` | Standalone tracking and coordinate handoff test script |

## Key Components

### Mission entrypoints

There is no single "main" script. Common starting points include:

- `UAV_Flight/Challenge_1_subscripts/Challenge1.py`
- `UGV_Drive/Challenge1_bridge.py`
- `UGV_Drive/Challenge2_bridge.py`
- `UGV_Drive/Challenge3v2.py`
- `Mission_Tests/Mission2_Guided.py`
- `Qual_Tests/Safeland.py`

For simulation-specific runs, look at:

- `Sim/uav_sim_code/sim_LandOnUGV.py`
- `Sim/uav_sim_code/sim_SamTest.py`
- `Sim/ugv_sim_code/sim_ground_station.py`
- `Sim/webots_sim/worlds/football_field.wbt`

### Communication stack

The communication path is split across several layers:

1. UAV Python scripts talk to the flight controller over MAVLink.
2. UGV Python scripts talk to the rover controller/autopilot over MAVLink or DroneKit.
3. `UGV_Drive/v2v_bridge.py` defines the framed serial protocol used between Python and ESP32 bridges.
4. `Connection/ESP32/ESP32/src/uav/esp_uav.cpp` and `Connection/ESP32/ESP32/src/ugv/esp_ugv.cpp` rebroadcast data over ESP-NOW.
5. In simulation, `sim_v2v_bridge.py` replaces the hardware radio bridge.

### Vision assets

The repo also includes vision-related assets and experiments, including:

- `UAV_Flight/calibration_chessboard.yaml` for camera calibration
- `UGV_Drive/best.pt` for YOLO-based object detection tests
- `UGV_Drive/akida_model.fbz` for BrainChip/Akida-related experiments

## Prerequisites

Because this repository does not currently ship with a `requirements.txt`, `pyproject.toml`, or other dependency lockfile, setup is manual.

### Base tools

- Python 3
- `pip` and `venv`
- Webots, if you want to use the simulator
- PlatformIO, if you want to build the ESP32 firmware
- MAVLink-compatible flight/vehicle controllers for hardware runs

### Common Python packages

Many scripts depend on some combination of:

- `pymavlink`
- `dronekit`
- `pyserial`
- `numpy`
- `opencv-contrib-python` for `cv2.aruco`
- `gpiozero`

A reasonable starting environment is:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install pymavlink dronekit pyserial numpy opencv-contrib-python gpiozero
```

Optional packages used by some scripts:

- `pyzed` via the ZED SDK Python API for ZED camera workflows
- `ultralytics` for `UGV_Drive/YOLOcamera.py`
- `depthai` for OAK camera workflows

## Running the Webots Simulation

The simulator is the safest place to start because it removes most of the serial-port and hardware dependencies.

1. Install Webots.
2. Open `Sim/webots_sim/worlds/football_field.wbt`.
3. In one terminal, run the UGV-side simulation script:

```bash
cd Sim/ugv_sim_code
python3 sim_ground_station.py
```

4. In a second terminal, run a UAV-side simulation script:

```bash
cd Sim/uav_sim_code
python3 sim_LandOnUGV.py
```

5. Start the simulation in Webots and watch the controller output.

The existing `Sim/README.txt` documents the same two-terminal workflow in shorter form.

## Running Hardware Scripts

Hardware execution is mission-specific. Before running any script, check and update:

- Serial ports such as `/dev/ttyACM0`, `/dev/ttyUSB0`, and `/dev/ttyAMA0`
- Camera choice and calibration files
- MAVLink connection strings and baud rates
- ESP32 MAC addresses in the firmware
- Any challenge-specific constants near the top of the selected script

Recommended approach:

1. Pick one challenge flow.
2. Identify the paired UAV and UGV scripts for that flow.
3. Verify the ports and baud rates in both files.
4. Confirm the radio bridge firmware matches the Python framing code.
5. Run the mission in a controlled environment before field testing.

## Building the ESP32 Firmware

PlatformIO configuration lives in `Connection/ESP32/ESP32/platformio.ini` and defines two environments:

- `esp_uav`
- `esp_ugv`

Example build commands:

```bash
cd Connection/ESP32/ESP32
pio run -e esp_uav
pio run -e esp_ugv
```

Before flashing hardware, verify the hard-coded peer MAC addresses in the corresponding source files.

## Notes and Caveats

- This repository mixes active mission scripts, test harnesses, and older experiments. Read the header comments in a script before treating it as the canonical implementation.
- `UAV_Flight/Challenge_2_subscripts/ReadMe.txt` explicitly describes those files as subsystem tests, not the final Challenge 2 implementation.
- Several directories contain spaces, such as `UAV_Flight/Camera Code/`, so quote paths in shell commands when needed.
- Many hardware scripts assume a Linux-style deployment environment with USB serial devices already mapped.
- The repo currently has no unified launcher, dependency manifest, or top-level test suite.

## Suggested Starting Points

If you are new to the repo:

1. Start in `Sim/` and get Webots running.
2. Read `UGV_Drive/v2v_bridge.py` to understand the serial protocol shared with the ESP32 firmware.
3. Use the challenge-specific scripts only after confirming the required hardware, ports, and camera stack are available.
