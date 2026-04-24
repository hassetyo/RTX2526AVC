from __future__ import print_function
import time
import argparse
from dronekit import connect, VehicleMode, LocationGlobalRelative

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
DEFAULT_ALTITUDE = 10   # metres
LAND_ALTITUDE_THRESHOLD = 0.5   # metres — considered landed below this
CONNECTION_STRING_LIVE = '/dev/ttyACM0'
CONNECTION_BAUD_LIVE = 115200
SITL_LAT = 32.92019271850586
SITL_LON = -96.94831085205078

# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------
parser = argparse.ArgumentParser(description="Simple GPS takeoff and land script.")
parser.add_argument(
    "--livedrone",
    action="store_true",
    help="Connect to a real drone instead of SITL simulation.",
)
parser.add_argument(
    "--altitude",
    type=float,
    default=DEFAULT_ALTITUDE,
    help=f"Target altitude in metres (default: {DEFAULT_ALTITUDE}).",
)
args = parser.parse_args()

SIMULATE_DRONE = not args.livedrone
TARGET_ALTITUDE = args.altitude


# ---------------------------------------------------------------------------
# Helper functions
# ---------------------------------------------------------------------------

def connect_vehicle():
    """Connect to the vehicle via SITL or a live serial connection."""
    if SIMULATE_DRONE:
        import dronekit_sitl
        print(f"Starting SITL at ({SITL_LAT}, {SITL_LON})...")
        sitl = dronekit_sitl.start_default(SITL_LAT, SITL_LON)
        connection_string = sitl.connection_string()
        print(f"Connecting to SITL on {connection_string}")
    else:
        connection_string = CONNECTION_STRING_LIVE
        print(f"Connecting to live drone on {connection_string}")

    vehicle = connect(connection_string, baud=CONNECTION_BAUD_LIVE, wait_ready=True)
    print("Connected to vehicle.")
    return vehicle


def wait_for_armable(vehicle):
    """Block until the vehicle reports it is armable."""
    print("Waiting for vehicle to become armable...")
    while not vehicle.is_armable:
        print(f"  Not armable yet — GPS fix: {vehicle.gps_0.fix_type}, "
              f"EKF: {vehicle.ekf_ok}, Battery: {vehicle.battery.voltage:.1f}V")
        time.sleep(1)
    print("Vehicle is armable.")


def arm_and_set_guided(vehicle):
    """Set GUIDED mode and arm the vehicle."""
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED":
        print("Waiting for GUIDED mode...")
        time.sleep(1)
    print("GUIDED mode active.")

    vehicle.armed = True
    while not vehicle.armed:
        print(f"Waiting for arming... (status: {vehicle.system_status.state})")
        time.sleep(1)
    print("Vehicle armed.")


def takeoff(vehicle, target_altitude):
    """Command a vertical takeoff and block until the target altitude is reached."""
    print(f"Taking off to {target_altitude} m...")
    vehicle.simple_takeoff(target_altitude)

    while True:
        current_alt = vehicle.location.global_relative_frame.alt
        print(f"  Altitude: {current_alt:.1f} m")
        if current_alt >= target_altitude * 0.95:
            print("Target altitude reached.")
            break
        time.sleep(1)


def land(vehicle):
    """Switch to LAND mode and block until the vehicle has touched down."""
    print("Initiating landing...")
    vehicle.mode = VehicleMode("LAND")

    while vehicle.mode.name != "LAND":
        print("Waiting for LAND mode...")
        time.sleep(1)

    while vehicle.location.global_relative_frame.alt > LAND_ALTITUDE_THRESHOLD:
        print(f"  Altitude: {vehicle.location.global_relative_frame.alt:.1f} m — descending...")
        time.sleep(1)

    print("Vehicle has landed.")


def print_vehicle_state(vehicle):
    """Print a summary of the current vehicle state."""
    loc = vehicle.location.global_relative_frame
    print(
        f"  GPS fix: {vehicle.gps_0.fix_type} | "
        f"Lat: {loc.lat:.7f} | Lon: {loc.lon:.7f} | Alt: {loc.alt:.1f} m | "
        f"Mode: {vehicle.mode.name} | Armed: {vehicle.armed}"
    )


# ---------------------------------------------------------------------------
# Main mission
# ---------------------------------------------------------------------------

def main():
    vehicle = connect_vehicle()

    try:
        print("\n--- Vehicle state at connection ---")
        print_vehicle_state(vehicle)

        wait_for_armable(vehicle)
        arm_and_set_guided(vehicle)
        takeoff(vehicle, TARGET_ALTITUDE)

        print("\n--- Hovering for 10 seconds ---")
        for i in range(10):
            print_vehicle_state(vehicle)
            time.sleep(1)

        land(vehicle)

        print("\n--- Final vehicle state ---")
        print_vehicle_state(vehicle)

    finally:
        print("Closing vehicle connection.")
        vehicle.close()


if __name__ == "__main__":
    main()
