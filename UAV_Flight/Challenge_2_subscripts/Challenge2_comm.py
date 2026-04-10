import time
import v2v_bridge2

# ==========================================
# UAV CHALLENGE 2 SCRIPT
# Sends preset coordinates to the UGV:
#   x = 8 ft
#   y = 8 ft
# ==========================================

UAV_BRIDGE_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200

PRESET_X_FT = 8.0
PRESET_Y_FT = 8.0

FT_TO_M = 0.3048
PRESET_X_M = PRESET_X_FT * FT_TO_M
PRESET_Y_M = PRESET_Y_FT * FT_TO_M


def main():
    print("==========================================")
    print("UAV Challenge 2 Sender")
    print("==========================================")
    print(f"Connecting to UAV bridge on {UAV_BRIDGE_PORT}...")

    bridge = v2v_bridge.V2VBridge(UAV_BRIDGE_PORT, baud=BAUD_RATE, name="UAV-Bridge")

    try:
        bridge.connect()
        time.sleep(1.0)

        print("Bridge connected.")
        print(f"Preset coordinates: x={PRESET_X_FT:.1f} ft ({PRESET_X_M:.3f} m), "
              f"y={PRESET_Y_FT:.1f} ft ({PRESET_Y_M:.3f} m)")

        # Optional startup message
        bridge.send_message("uav challenge 2 sender live")

        # Tell ground station we are beginning challenge 2
        print("Sending Challenge 2 start command...")
        bridge.send_challenge2_start(seq=1)
        time.sleep(0.5)

        # Send coordinates in meters
        print("Sending Challenge 2 coordinates...")
        bridge.send_challenge2_coords_ft(PRESET_X_FT, PRESET_Y_FT)

        print("Waiting for completion message from ground station...")
        while True:
            msg = bridge.get_message()
            if msg:
                print(f"[UAV] Incoming message: {msg}")
                if bridge.is_challenge2_complete_message(msg):
                    coords = bridge.parse_challenge2_complete_message(msg)
                    if coords is not None:
                        x_done, y_done = coords
                        print("==========================================")
                        print("Challenge 2 complete acknowledgement received")
                        print(f"Ground station reported completion for x={x_done:.2f} m, y={y_done:.2f} m")
                        print("==========================================")
                        break

            cmd = bridge.get_command()
            if cmd:
                print(f"[UAV] Incoming command packet: {cmd}")

            telem = bridge.get_telemetry()
            if telem:
                seq, t_ms, vx, vy, marker, estop = telem
                print(
                    f"[UAV] Telemetry | seq={seq} t_ms={t_ms} "
                    f"vx={vx:.3f} vy={vy:.3f} marker={marker} estop={estop}"
                )

            time.sleep(0.2)

    except KeyboardInterrupt:
        print("Keyboard interrupt received. Exiting.")
    finally:
        bridge.stop()


if __name__ == "__main__":
    main()