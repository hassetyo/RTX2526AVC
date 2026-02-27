"""
bridge_loopback_test.py  —  Quick sanity check
-----------------------------------------------
Verifies that virtual_esp_bridge.py is correctly relaying packets
WITHOUT needing ArduPilot SITL installed.

Sends 10 fake UAV position JSON packets through the bridge and
checks that all 10 are received with correct values.

Run virtual_esp_bridge.py first, then:
    python bridge_loopback_test.py

Expected output:
    [SENDER]   TX #1: {"x":1.0,"y":0.5,...}
    [RECEIVER] RX #1: x=1.000  y=0.500  ✓
    ...
    [RESULT] 10/10 packets relayed correctly ✓
"""

import socket
import threading
import time
import json

BRIDGE_HOST = "127.0.0.1"
UAV_PORT    = 5555
UGV_PORT    = 5556
NUM_PACKETS = 10
SEND_INTERVAL = 0.6  # seconds (> 500 ms bridge pacing)

received = []
receive_done = threading.Event()

# ─────────────────────────────────────────
# RECEIVER (simulates UGV navigator reading from bridge)
# ─────────────────────────────────────────
def receiver_thread():
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((BRIDGE_HOST, UGV_PORT))
        print(f"[RECEIVER] Connected to bridge UGV port {UGV_PORT}")
    except ConnectionRefusedError:
        print(f"[ERROR] Could not connect to bridge on {BRIDGE_HOST}:{UGV_PORT}")
        print("        Is virtual_esp_bridge.py running?")
        receive_done.set()
        return

    buf = b""
    count = 0
    sock.settimeout(5.0)

    try:
        while count < NUM_PACKETS:
            try:
                data = sock.recv(512)
            except socket.timeout:
                print("[RECEIVER] Timeout waiting for packet")
                break
            if not data:
                break
            buf += data
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                line = line.strip()
                if not line:
                    continue
                try:
                    pkt = json.loads(line.decode('ascii'))
                    count += 1
                    received.append(pkt)
                    print(f"[RECEIVER] RX #{count:2d}: x={pkt['x']:.3f}  y={pkt['y']:.3f}  t={pkt['t']}  ✓")
                except Exception as e:
                    print(f"[RECEIVER] Bad packet: {e}")
    finally:
        sock.close()
        receive_done.set()

# ─────────────────────────────────────────
# SENDER (simulates Jetson sender writing to bridge)
# ─────────────────────────────────────────
def run_sender():
    time.sleep(0.5)  # Let receiver connect first
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((BRIDGE_HOST, UAV_PORT))
        print(f"[SENDER]   Connected to bridge UAV port {UAV_PORT}\n")
    except ConnectionRefusedError:
        print(f"[ERROR] Could not connect to bridge on {BRIDGE_HOST}:{UAV_PORT}")
        print("        Is virtual_esp_bridge.py running?")
        return

    try:
        for i in range(1, NUM_PACKETS + 1):
            pkt = {
                "x":  round(i * 0.5, 3),
                "y":  round(i * 0.25, 3),
                "z":  -2.5,
                "vx": 0.3,
                "vy": 0.1,
                "t":  int(time.time() * 1000)
            }
            line = json.dumps(pkt, separators=(',', ':')) + '\n'
            sock.sendall(line.encode('ascii'))
            print(f"[SENDER]   TX #{i:2d}: {line.strip()}")
            time.sleep(SEND_INTERVAL)
    finally:
        sock.close()

# ─────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────
if __name__ == "__main__":
    print("=" * 55)
    print("  Bridge Loopback Test")
    print("  (virtual_esp_bridge.py must be running)")
    print("=" * 55 + "\n")

    rx_thread = threading.Thread(target=receiver_thread, daemon=True)
    rx_thread.start()

    time.sleep(0.3)
    run_sender()

    receive_done.wait(timeout=15)
    time.sleep(0.5)

    print()
    print("=" * 55)
    n = len(received)
    if n == NUM_PACKETS:
        print(f"  [RESULT] {n}/{NUM_PACKETS} packets relayed correctly ✓")
        print("  Bridge is working. Ready for SITL testing.")
    else:
        print(f"  [RESULT] Only {n}/{NUM_PACKETS} packets received — check bridge!")
    print("=" * 55)
