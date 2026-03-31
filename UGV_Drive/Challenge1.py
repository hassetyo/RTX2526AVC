from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import math

# Challenge 1:
# Move exactly 10 feet forward at 1 mile per hour using a displacement command only.

UGV_CONTROL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

DISTANCE_FT = 10.0
SPEED_MPH = 1.0

FT_TO_M = 0.3048
MPH_TO_MPS = 0.44704

DISTANCE_M = DISTANCE_FT * FT_TO_M      # 3.048 m
SPEED_MPS = SPEED_MPH * MPH_TO_MPS      # 0.44704 m/s

POSITION_TOLERANCE_M = 0.03
TIMEOUT_S = 20.0


def wait_for_mode(vehicle, mode_name, timeout_s=5.0):
    deadline = time.time() + timeout_s
    while vehicle.mode.name != mode_name and time.time() < deadline:
        time.sleep(0.1)
    return vehicle.mode.name == mode_name


def wait_for_armed(vehicle, armed_state, timeout_s=3.0):
    deadline = time.time() + timeout_s
    while vehicle.armed != armed_state and time.time() < deadline:
        time.sleep(0.1)
    return vehicle.armed == armed_state


def arm_ugv(vehicle):
    if not vehicle.is_armable:
        print("Warning: vehicle reports not armable; attempting arm anyway.")

    # Same arm/disarm/arm pattern as your current script
    for label, state in (("FIRST ARM", True), ("RESET DISARM", False), ("FINAL ARM", True)):
        print(f"{label} in mode {vehicle.mode.name}...")
        vehicle.armed = state
        if not wait_for_armed(vehicle, state):
            raise RuntimeError(f"Failed to set armed={state}")
        time.sleep(1.0)

    print(f"Switching {vehicle.mode.name} -> GUIDED...")
    vehicle.mode = VehicleMode("GUIDED")
    if not wait_for_mode(vehicle, "GUIDED"):
        raise RuntimeError(f"Failed to enter GUIDED mode, current mode: {vehicle.mode.name}")


def get_local_position(vehicle):
    msg = vehicle._master.messages.get("LOCAL_POSITION_NED")
    if msg is None:
        return None
    return (msg.x, msg.y, msg.z)


def set_speed_limit(vehicle, speed_mps):
    original_wp_speed = None
    if "WP_SPEED" in vehicle.parameters:
        original_wp_speed = float(vehicle.parameters["WP_SPEED"])
        vehicle.parameters["WP_SPEED"] = float(speed_mps)
        time.sleep(0.5)
        print(f"WP_SPEED set to {speed_mps:.3f} m/s")
    else:
        print("Warning: WP_SPEED not available; continuing anyway.")
    return original_wp_speed


def restore_speed_limit(vehicle, original_wp_speed):
    if original_wp_speed is not None:
        vehicle.parameters["WP_SPEED"] = float(original_wp_speed)
        time.sleep(0.5)
        print(f"WP_SPEED restored to {original_wp_speed:.3f} m/s")


def build_displacement_msg(vehicle, forward_m):
    """
    BODY_OFFSET_NED:
      x = forward relative to rover heading
      y = right
      z = down

    This message uses position only and ignores velocity, acceleration, yaw, yaw rate.
    """
    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )

    return vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms
        0,  # target_system
        0,  # target_component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        type_mask,
        forward_m,  # x forward
        0.0,        # y
        0.0,        # z
        0.0, 0.0, 0.0,  # vx, vy, vz ignored
        0.0, 0.0, 0.0,  # afx, afy, afz ignored
        0.0, 0.0        # yaw, yaw_rate ignored
    )


def build_stop_msg(vehicle):
    """
    Hold current target by sending zero forward displacement in BODY_OFFSET_NED.
    """
    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )

    return vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        type_mask,
        0.0,
        0.0,
        0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0
    )


def move_forward_displacement(vehicle, distance_m, speed_mps):
    start_pos = get_local_position(vehicle)
    if start_pos is None:
        raise RuntimeError(
            "LOCAL_POSITION_NED is not available. A displacement command requires a valid local position estimate."
        )

    original_wp_speed = set_speed_limit(vehicle, speed_mps)
    move_msg = build_displacement_msg(vehicle, distance_m)
    stop_msg = build_stop_msg(vehicle)

    try:
        print(f"Starting displacement move: {distance_m:.3f} m forward at {speed_mps:.3f} m/s")
        start_time = time.time()
        last_print = 0.0

        while True:
            vehicle.send_mavlink(move_msg)

            pos = get_local_position(vehicle)
            if pos is not None:
                dx = pos[0] - start_pos[0]
                dy = pos[1] - start_pos[1]
                traveled = math.sqrt(dx * dx + dy * dy)
                remaining = max(0.0, distance_m - traveled)

                elapsed = time.time() - start_time
                if elapsed - last_print >= 1.0:
                    print(f"  traveled={traveled:.3f} m  remaining={remaining:.3f} m")
                    last_print = elapsed

                if remaining <= POSITION_TOLERANCE_M:
                    print("Target distance reached.")
                    break

            if (time.time() - start_time) > TIMEOUT_S:
                print("Timeout reached before finishing movement.")
                break

            time.sleep(0.1)

    finally:
        vehicle.send_mavlink(stop_msg)
        time.sleep(0.5)
        restore_speed_limit(vehicle, original_wp_speed)


def main():
    print("==========================================")
    print("           CHALLENGE 1 TEST")
    print("==========================================")
    print(f"Connecting to UGV at {UGV_CONTROL_PORT}...")
    print(f"Target distance: {DISTANCE_FT:.1f} ft ({DISTANCE_M:.3f} m)")
    print(f"Target speed:    {SPEED_MPH:.1f} mph ({SPEED_MPS:.4f} m/s)")
    print("Command type:    displacement only")

    vehicle = connect(UGV_CONTROL_PORT, wait_ready=True, baud=BAUD_RATE)

    try:
        print(f"Initial state: armed={vehicle.armed} mode={vehicle.mode.name} armable={vehicle.is_armable}")
        arm_ugv(vehicle)
        print("UGV armed in GUIDED mode. Starting Challenge 1 move...")
        move_forward_displacement(vehicle, DISTANCE_M, SPEED_MPS)
        print("Move complete. Disarming...")
        vehicle.armed = False
        wait_for_armed(vehicle, False)
    finally:
        vehicle.close()


if __name__ == "__main__":
    main()