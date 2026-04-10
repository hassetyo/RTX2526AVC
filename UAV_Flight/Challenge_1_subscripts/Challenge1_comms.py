import time
import v2v_bridge

# UAV-SIDE SCRIPT FOR CHALLENGE 1
# Sends only the destination to the UGV.
# Challenge 1:
#   - no turning
#   - no x movement
#   - UGV goes straight to y only

UAV_BRIDGE_PORT = "/dev/ttyUSB0"   # update if needed

# Destination for Challenge 1
# Example: go straight 8 ft
Y_DISTANCE_FT = 8.0


def main():
    bridge = v2v_bridge.V2VBridge(UAV_BRIDGE_PORT, name="UAV-Bridge")

    try:
        bridge.connect()
        time.sleep(1.0)

        # x must stay 0 for Challenge 1
        x_ft = 0.0
        y_ft = Y_DISTANCE_FT

        # Send destination as GOTO:x,y in meters
        bridge.send_challenge2_coords_ft(x_ft, y_ft)

        print(f"[UAV] Sent Challenge 1 destination: x={x_ft:.1f} ft, y={y_ft:.1f} ft")

        # Optional: wait briefly for a completion message
        start = time.time()
        timeout_s = 15.0

        while time.time() - start < timeout_s:
            msg = bridge.get_message()
            if msg:
                print(f"[UAV] Received: {msg}")
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("[UAV] Interrupted by user.")
    except Exception as e:
        print(f"[UAV] Error: {e}")
    finally:
        try:
            bridge.stop()
        except Exception:
            pass


if __name__ == "__main__":
    main()