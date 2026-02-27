"""
virtual_esp_bridge.py  —  PC Testing Tool
------------------------------------------
Simulates the entire ESP32 hardware chain (server.cpp + ESP-NOW + client.cpp)
with a simple TCP socket server.

On real hardware the data path is:
    Jetson UART → ESP32 server → ESP-NOW RF → ESP32 client → RPi UART

This script replaces that entire chain with:
    sitl_uav_sender.py  ──TCP──▶  virtual_esp_bridge.py  ──TCP──▶  sitl_ugv_navigator.py

Run this FIRST before starting either SITL script.

Usage:
    python virtual_esp_bridge.py [--host 127.0.0.1] [--uav-port 5555] [--ugv-port 5556]

Default ports:
    UAV sender  connects to  127.0.0.1:5555  (simulates ESP32 server UART RX)
    UGV navigator connects to 127.0.0.1:5556  (simulates ESP32 client UART TX)
"""

import socket
import threading
import argparse
import time
import queue

# ─────────────────────────────────────────
# CONFIG (override with CLI args)
# ─────────────────────────────────────────
DEFAULT_HOST     = "127.0.0.1"
DEFAULT_UAV_PORT = 5555   # UAV sender writes here
DEFAULT_UGV_PORT = 5556   # UGV navigator reads here
SIMULATED_DELAY_S = 0.01  # Simulated ESP-NOW air latency (~10 ms realistic)

# ─────────────────────────────────────────
# SHARED QUEUE (thread-safe relay)
# ─────────────────────────────────────────
relay_queue = queue.Queue(maxsize=20)  # Buffer up to 20 packets

# ─────────────────────────────────────────
# UAV RECEIVER THREAD
# Listens for JSON lines from sitl_uav_sender.py
# ─────────────────────────────────────────
def uav_receiver(host, port):
    """Accept connection from UAV sender and queue incoming JSON lines."""
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind((host, port))
    server_sock.listen(1)
    print(f"[BRIDGE] UAV receiver listening on {host}:{port}")

    while True:
        conn, addr = server_sock.accept()
        print(f"[BRIDGE] UAV sender connected from {addr}")
        buf = b""
        try:
            while True:
                data = conn.recv(512)
                if not data:
                    break
                buf += data
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    line = line.strip()
                    if line:
                        try:
                            relay_queue.put_nowait(line.decode('ascii'))
                            print(f"[BRIDGE→] Queued: {line.decode('ascii')}")
                        except queue.Full:
                            print("[BRIDGE] Queue full — dropping oldest packet")
                            try:
                                relay_queue.get_nowait()
                                relay_queue.put_nowait(line.decode('ascii'))
                            except Exception:
                                pass
        except Exception as e:
            print(f"[BRIDGE] UAV receiver error: {e}")
        finally:
            conn.close()
            print("[BRIDGE] UAV sender disconnected — waiting for reconnect...")


# ─────────────────────────────────────────
# UGV SENDER THREAD
# Accepts connection from sitl_ugv_navigator.py and forwards queued lines
# ─────────────────────────────────────────
def ugv_sender(host, port):
    """Accept connection from UGV navigator and forward queued JSON lines."""
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind((host, port))
    server_sock.listen(1)
    print(f"[BRIDGE] UGV sender listening on {host}:{port}")

    while True:
        conn, addr = server_sock.accept()
        print(f"[BRIDGE] UGV navigator connected from {addr}")
        try:
            while True:
                try:
                    packet = relay_queue.get(timeout=1.0)
                except queue.Empty:
                    continue

                time.sleep(SIMULATED_DELAY_S)  # Simulate ESP-NOW latency

                line = packet + '\n'
                conn.sendall(line.encode('ascii'))
                print(f"[BRIDGE←] Forwarded: {packet}")

        except (BrokenPipeError, ConnectionResetError):
            print("[BRIDGE] UGV navigator disconnected — waiting for reconnect...")
        except Exception as e:
            print(f"[BRIDGE] UGV sender error: {e}")
        finally:
            conn.close()


# ─────────────────────────────────────────
# STATS THREAD
# ─────────────────────────────────────────
def print_stats():
    """Print queue depth every 10 seconds."""
    while True:
        time.sleep(10)
        print(f"[BRIDGE] Queue depth: {relay_queue.qsize()} packets")


# ─────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Virtual ESP32 bridge for SITL testing")
    parser.add_argument("--host",     default=DEFAULT_HOST,     help="Bind address")
    parser.add_argument("--uav-port", default=DEFAULT_UAV_PORT, type=int, help="UAV sender port")
    parser.add_argument("--ugv-port", default=DEFAULT_UGV_PORT, type=int, help="UGV navigator port")
    args = parser.parse_args()

    print("=" * 60)
    print("  Virtual ESP32 Bridge  —  PC SITL Testing")
    print("=" * 60)
    print(f"  UAV sender   →  connect to {args.host}:{args.uav_port}")
    print(f"  UGV navigator→  connect to {args.host}:{args.ugv_port}")
    print(f"  Simulated RF delay: {SIMULATED_DELAY_S*1000:.0f} ms")
    print("=" * 60)

    threads = [
        threading.Thread(target=uav_receiver, args=(args.host, args.uav_port), daemon=True, name="uav_rx"),
        threading.Thread(target=ugv_sender,   args=(args.host, args.ugv_port), daemon=True, name="ugv_tx"),
        threading.Thread(target=print_stats,  daemon=True, name="stats"),
    ]
    for t in threads:
        t.start()

    print("[BRIDGE] All threads running. Press Ctrl-C to stop.\n")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[BRIDGE] Shutting down.")
