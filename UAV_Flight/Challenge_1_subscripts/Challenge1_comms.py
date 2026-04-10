import time
from v2v_bridge import (
    V2VBridge,
    CMD_MOVE_FORWARD,
    CMD_STOP,
)

# =========================
# UAV SIDE CONFIG
# =========================
RADIO_PORT = "/dev/ttyUSB0"   # change if your radio is on a different port
BAUD_RATE = 115200

WAIT_FOR_RESPONSE_SECONDS = 10.0


def main():
    print("=============================================")
    print("UAV Challenge 1: Tell UGV to Move 10 Feet")
    print("=============================================")
    print(f"Connecting to radio on {RADIO_PORT} at {BAUD_RATE}...")

    bridge = V2VBridge(RADIO_PORT, BAUD_RATE, name="UAV")

    try:
        bridge.connect()
        time.sleep(2.0)

        cmd_seq = 1
        estop = 0

        print("Sending CMD_MOVE_FORWARD to UGV...")
        bridge.send_command(cmd_seq, CMD_MOVE_FORWARD, estop)
        print("Command sent.")

        print("Waiting for UGV responses...")
        start = time.time()

        while (time.time() - start) < WAIT_FOR_RESPONSE_SECONDS:
            msg = bridge.get_message()
            if msg:
                print(f"[UGV MSG] {msg}")

            telem = bridge.get_telemetry()
            if telem:
                seq, t_ms, vx, vy, marker, estop_flag = telem
                print(
                    f"[UGV TELEM] seq={seq} "
                    f"t_ms={t_ms} "
                    f"vx={vx:.3f} "
                    f"vy={vy:.3f} "
                    f"marker={marker} "
                    f"estop={estop_flag}"
                )

            time.sleep(0.25)

        print("Done waiting for response.")

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received. Sending STOP...")
        try:
            bridge.send_command(999, CMD_STOP, 1)
            time.sleep(0.5)
        except Exception:
            pass

    finally:
        print("Stopping bridge...")
        bridge.stop()
        print("Done.")


if __name__ == "__main__":
    main()