"""
3-Sensor Obstacle Detection Test
---------------------------------
Center : TF-Nova lidar  (serial /dev/ttyAMA0)
Right  : Ultrasonic     (trig=5,  echo=6)
Left   : Ultrasonic     (trig=23, echo=22)

Prints a line every 0.25 s showing each sensor's distance
and whether it is within the detection threshold.

Thresholds:
  Ultrasonic (left / right) : 1.9 ft  (0.579 m)
  Lidar      (center)       : 2.1 ft  (0.640 m)

Press Ctrl-C to stop.
"""

from gpiozero import DistanceSensor
import serial
import time

try:
    from gpiozero.pins.lgpio import LGPIOFactory
    _pin_factory = LGPIOFactory()
except ImportError:
    _pin_factory = None

# â”€â”€ GPIO pins â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
RIGHT_TRIG = 5
RIGHT_ECHO = 6
LEFT_TRIG  = 23
LEFT_ECHO  = 22

# â”€â”€ Lidar serial settings â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
LIDAR_PORT      = "/dev/ttyAMA0"
LIDAR_BAUD_RATE = 115200

# â”€â”€ Thresholds â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
FT_TO_M = 0.3048

ULTRASONIC_THRESHOLD_FT = 1.9
LIDAR_THRESHOLD_FT      = 2.1

ULTRASONIC_THRESHOLD_M = ULTRASONIC_THRESHOLD_FT * FT_TO_M   # ~0.579 m
LIDAR_THRESHOLD_M      = LIDAR_THRESHOLD_FT * FT_TO_M        # ~0.640 m

ULTRASONIC_MAX_DISTANCE_M = 4.0   # gpiozero max_distance param

# â”€â”€ TF-Nova constants â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
FRAME_HEADER         = 0x59
LIDAR_MIN_CONFIDENCE = 10
LIDAR_NO_TARGET_M    = 9999.0

POLL_INTERVAL_S = 0.25


# â”€â”€ Lidar helpers (from your existing code) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
def open_lidar(port=LIDAR_PORT, baud=LIDAR_BAUD_RATE):
    ser = serial.Serial(
        port=port,
        baudrate=baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.1,
    )
    ser.reset_input_buffer()
    print(f"TF-Nova lidar opened on {port}")
    return ser


def read_lidar_m(ser):
    """Read one fresh frame from the TF-Nova and return distance in metres."""
    ser.reset_input_buffer()

    for _ in range(18):
        b1 = ser.read(1)
        if not b1 or b1[0] != FRAME_HEADER:
            continue

        b2 = ser.read(1)
        if not b2 or b2[0] != FRAME_HEADER:
            continue

        payload = ser.read(7)
        if len(payload) < 7:
            return LIDAR_NO_TARGET_M

        dist_l, dist_h, peak_l, peak_h, temp, confidence, checksum = payload

        raw = [FRAME_HEADER, FRAME_HEADER,
               dist_l, dist_h, peak_l, peak_h, temp, confidence]
        if (sum(raw) & 0xFF) != checksum:
            continue

        distance_cm = (dist_h << 8) | dist_l
        if confidence < LIDAR_MIN_CONFIDENCE or distance_cm == 0:
            return LIDAR_NO_TARGET_M

        return distance_cm / 100.0

    return LIDAR_NO_TARGET_M


# â”€â”€ Ultrasonic helpers â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
def open_ultrasonic(trig, echo, label="US"):
    kwargs = {"pin_factory": _pin_factory} if _pin_factory else {}
    sensor = DistanceSensor(
        echo=echo,
        trigger=trig,
        max_distance=ULTRASONIC_MAX_DISTANCE_M,
        **kwargs,
    )
    print(f"{label} ultrasonic opened  trig={trig}  echo={echo}")
    return sensor


# â”€â”€ Main loop â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
def main():
    print("=" * 50)
    print("  3-SENSOR OBSTACLE DETECTION TEST")
    print("=" * 50)
    print(f"  Ultrasonic threshold : {ULTRASONIC_THRESHOLD_FT} ft ({ULTRASONIC_THRESHOLD_M:.3f} m)")
    print(f"  Lidar threshold      : {LIDAR_THRESHOLD_FT} ft ({LIDAR_THRESHOLD_M:.3f} m)")
    print()

    # Open sensors
    us_right  = open_ultrasonic(RIGHT_TRIG, RIGHT_ECHO, label="RIGHT")
    us_left   = open_ultrasonic(LEFT_TRIG,  LEFT_ECHO,  label="LEFT")
    lidar_ser = open_lidar()

    # Let lidar warm up
    print("Warming up lidar (1 s)...")
    time.sleep(1.0)

    print()
    print("Reading sensors: press Ctrl-C to stop")
    print("-" * 70)

    try:
        while True:
            # â”€â”€ Read all three sensors â”€â”€
            right_m  = us_right.distance          # metres
            left_m   = us_left.distance           # metres
            center_m = read_lidar_m(lidar_ser)    # metres

            # â”€â”€ Check thresholds â”€â”€
            right_detected  = right_m  <= ULTRASONIC_THRESHOLD_M
            left_detected   = left_m   <= ULTRASONIC_THRESHOLD_M
            center_detected = (center_m < LIDAR_NO_TARGET_M
                               and center_m <= LIDAR_THRESHOLD_M)

            # â”€â”€ Convert to feet for display â”€â”€
            right_ft  = right_m / FT_TO_M
            left_ft   = left_m / FT_TO_M
            center_ft = center_m / FT_TO_M if center_m < LIDAR_NO_TARGET_M else None

            # â”€â”€ Build status strings â”€â”€
            def fmt(label, dist_ft, detected):
                tag = "*** DETECTED ***" if detected else "clear"
                if dist_ft is None:
                    return f"{label}: no target  [{tag}]"
                return f"{label}: {dist_ft:5.2f} ft  [{tag}]"

            r_str = fmt("RIGHT(US)", right_ft,  right_detected)
            c_str = fmt("CENTER(L)", center_ft, center_detected)
            l_str = fmt("LEFT(US)",  left_ft,   left_detected)

            print(f"  {l_str}  |  {c_str}  |  {r_str}")

            time.sleep(POLL_INTERVAL_S)

    except KeyboardInterrupt:
        print("\nStopped by user.")

    finally:
        us_right.close()
        us_left.close()
        lidar_ser.close()
        print("All sensors closed.")


if __name__ == "__main__":
    main()