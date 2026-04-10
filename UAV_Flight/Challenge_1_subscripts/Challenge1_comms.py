import time
import v2v_bridge

# UAV-SIDE COMMAND SCRIPT
# Sends commands to the UGV through the V2V bridge.
# Mission:
#   1) move forward 8 ft total = 4 x 2 ft
#   2) stop
#   3) turn left 90 degrees
#   4) stop

UAV_BRIDGE_PORT = "/dev/ttyUSB0"   # update if needed


def send_cmd(bridge, seq, cmd, label, estop=0):
    extra = " | ESTOP=1" if estop else ""
    print(f"[UAV] Sending command {seq}: {label}{extra}")
    bridge.send_command(seq, cmd, estop)


def main():
    bridge = v2v_bridge.V2VBridge(UAV_BRIDGE_PORT, name="UAV-Bridge")

    try:
        bridge.connect()
        time.sleep(1.0)
        bridge.send_message("uav online - sending 8ft left-turn mission")

        seq = 1

        # Move forward 8 ft total = 4 x 2 ft
        for i in range(4):
            send_cmd(bridge, seq, v2v_bridge.CMD_MOVE_2FT, f"MOVE_2FT #{i+1}")
            seq += 1
            time.sleep(2.5)

        # Explicit stop after forward motion
        send_cmd(bridge, seq, v2v_bridge.CMD_STOP, "STOP")
        seq += 1
        time.sleep(2.0)

        # Turn left 90 degrees
        send_cmd(bridge, seq, v2v_bridge.CMD_TURN_LEFT, "TURN_LEFT_90")
        seq += 1
        time.sleep(4.0)

        # Final stop for safety
        send_cmd(bridge, seq, v2v_bridge.CMD_STOP, "STOP_AFTER_TURN")
        seq += 1
        time.sleep(2.0)

        print("[UAV] Mission commands sent.")

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