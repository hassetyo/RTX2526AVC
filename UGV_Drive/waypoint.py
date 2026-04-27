from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

# ==========================================
# UGV GPS WAYPOINT TRACKER
# Behavior:
#   1) Connect to the UGV Pixhawk
#   2) Switch to GUIDED mode
#   3) Keep a waypoint object updated with the UGV's current GPS location
#   4) Print the UGV waypoint location every second
# ==========================================

# ----------------------------
# Pixhawk connection settings
# ----------------------------
UGV_CONTROL_PORT = "/dev/ttyACM0"
UGV_BAUD_RATE = 115200

# If True, the script will arm the UGV after entering GUIDED mode.
# Leave False if you only want to read GPS location without moving.
ARM_UGV = False

# GPS fix type needed before printing valid GPS coordinates.
# 0 = no GPS, 1 = no fix, 2 = 2D fix, 3 = 3D fix
MIN_GPS_FIX_TYPE = 2


print("==========================================")
print("        UGV GPS WAYPOINT TRACKER")
print("==========================================")
print(f"[Waypoint] Connecting to UGV at {UGV_CONTROL_PORT}...")

try:
    vehicle = connect(UGV_CONTROL_PORT, wait_ready=True, baud=UGV_BAUD_RATE)
    print("[Waypoint] Connected to UGV.")
except Exception as e:
    print(f"!!! Error connecting to UGV: {e} !!!")
    raise SystemExit(1)


def wait_for_mode(vehicle, mode_name, timeout_s=10.0):
    deadline = time.time() + timeout_s
    while vehicle.mode.name != mode_name and time.time() < deadline:
        time.sleep(0.1)
    return vehicle.mode.name == mode_name


def wait_for_armed(vehicle, armed_state, timeout_s=10.0):
    deadline = time.time() + timeout_s
    while vehicle.armed != armed_state and time.time() < deadline:
        time.sleep(0.1)
    return vehicle.armed == armed_state


def get_gps_fix_type(vehicle):
    try:
        return int(vehicle.gps_0.fix_type)
    except Exception:
        return 0


def get_current_waypoint(vehicle):
    """
    Creates a waypoint object using the UGV's current GPS position.
    This waypoint is updated every loop so the UGV always has a waypoint
    matching its current location.
    """
    current_location = vehicle.location.global_relative_frame

    if current_location.lat is None or current_location.lon is None:
        return None

    altitude = current_location.alt
    if altitude is None:
        altitude = 0.0

    return LocationGlobalRelative(
        current_location.lat,
        current_location.lon,
        altitude
    )


def print_waypoint(waypoint, gps_fix_type):
    print(
        "[Waypoint] UGV waypoint location: "
        f"lat={waypoint.lat:.7f}, "
        f"lon={waypoint.lon:.7f}, "
        f"alt={waypoint.alt:.2f} m, "
        f"gps_fix={gps_fix_type}"
    )


def enter_guided_mode(vehicle):
    print("[Waypoint] Switching UGV to GUIDED mode...")
    vehicle.mode = VehicleMode("GUIDED")

    if not wait_for_mode(vehicle, "GUIDED"):
        raise RuntimeError(f"Failed to enter GUIDED mode. Current mode: {vehicle.mode.name}")

    print("[Waypoint] GUIDED mode confirmed.")


def arm_vehicle_if_enabled(vehicle):
    if not ARM_UGV:
        print("[Waypoint] ARM_UGV is False. UGV will stay disarmed.")
        return

    print("[Waypoint] Arming UGV...")
    vehicle.armed = True

    if not wait_for_armed(vehicle, True):
        raise RuntimeError("Failed to arm UGV.")

    print("[Waypoint] UGV armed.")


def main():
    try:
        enter_guided_mode(vehicle)
        arm_vehicle_if_enabled(vehicle)

        print("[Waypoint] Waiting for GPS location...")

        while True:
            gps_fix_type = get_gps_fix_type(vehicle)

            if gps_fix_type < MIN_GPS_FIX_TYPE:
                print(
                    "[Waypoint] Waiting for valid GPS fix... "
                    f"current_fix={gps_fix_type}, required_fix={MIN_GPS_FIX_TYPE}"
                )
                time.sleep(1.0)
                continue

            ugv_waypoint = get_current_waypoint(vehicle)

            if ugv_waypoint is None:
                print("[Waypoint] GPS location unavailable. Waiting...")
                time.sleep(1.0)
                continue

            print_waypoint(ugv_waypoint, gps_fix_type)
            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\n[Waypoint] Stopping waypoint tracker...")
    except Exception as e:
        print(f"!!! Waypoint tracker error: {e} !!!")
    finally:
        try:
            vehicle.close()
            print("[Waypoint] Vehicle connection closed.")
        except Exception:
            pass


if __name__ == "__main__":
    main()
