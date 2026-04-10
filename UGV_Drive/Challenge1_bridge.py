import time
from v2v_bridge import (
    V2VBridge,
    CMD_MOVE_FORWARD,
    CMD_STOP,
    TYPE_TELEM,
)

# =========================
# UAV SIDE CONFIG
# =========================
RADIO_PORT = "/dev/ttyUSB0"   # change if your radio is on a different port
BAUD_RATE = 115200

SEND_DEBUG_MESSAGE = True
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

        if SEND_DEBUG_MESSAGE:
            bridge.send_message("UAV online. Sending Challenge 1 command.")
            time.sleep(0.25)

        cmd_seq = 1
        estop = 0

        print("Sending CMD_MOVE_FORWARD to UGV...")
        bridge.send_command(cmd_seq, CMD_MOVE_FORWARD, estop)
        print("Command sent.")

        print("Waiting for UGV responses/telemetry...")
        start = time.time()
        last_telem_print = 0.0

        while (time.time() - start) < WAIT_FOR_RESPONSE_SECONDS:
            msg = bridge.get_message()
            if msg:
                print(f"[UGV MSG] {msg}")

            cmd = bridge.get_command(consume=False)
            if cmd:
                print(f"[INFO] Saw command on bridge buffer: {cmd}")

            telem = bridge.get_telemetry()
            now = time.time()
            if telem and (now - last_telem_print) >= 1.0:
                seq, t_ms, vx, vy, marker, estop_flag = telem
                print(
                    f"[UGV TELEM] seq={seq} t_ms={t_ms} "
                    f"vx={vx:.3f} vy={vy:.3f} marker={marker} estop={estop_flag}"
                )
                last_telem_print = now

            time.sleep(0.1)

        print("Finished waiting for UGV response.")

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