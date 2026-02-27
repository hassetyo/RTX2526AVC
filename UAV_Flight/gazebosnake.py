import argparse
import asyncio
import math
import time

from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw
# python3 gazebo_snake.py --connect udp://:14540

# ==========================
# Mission settings (same as yours)
# ==========================
ALTITUDE_M = 10.0
LEG_DISTANCE_M = 10.0
STEP_DISTANCE_M = 3.0
SPEED_MS = 1.0

# Safety tolerances
POS_TOL_M = 0.25
MOVE_TIMEOUT_S = 90.0
YAW_TOL_DEG = 2.0
TURN_TIMEOUT_S = 20.0
YAW_STABLE_TIME_S = 0.5


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def deg_wrap_180(deg: float) -> float:
    while deg > 180.0:
        deg -= 360.0
    while deg < -180.0:
        deg += 360.0
    return deg


async def wait_connected(drone: System):
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected to PX4")
            return


async def wait_global_position_ok(drone: System):
    # In SITL this usually becomes OK quickly
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("Position OK (global + home)")
            return


async def get_local_ned(drone: System, timeout_s: float = 2.0):
    """
    Returns (north_m, east_m, down_m) in LOCAL NED.
    """
    start = time.time()
    async for pos in drone.telemetry.position_velocity_ned():
        n = float(pos.position.north_m)
        e = float(pos.position.east_m)
        d = float(pos.position.down_m)
        return n, e, d
        # (stream yields continuously; we return first sample)

        if time.time() - start > timeout_s:
            break
    return None


async def get_yaw_deg(drone: System, timeout_s: float = 2.0):
    """
    Returns yaw in degrees (0..360) from telemetry.attitude_euler.
    """
    start = time.time()
    async for att in drone.telemetry.attitude_euler():
        return float(att.yaw_deg)
        if time.time() - start > timeout_s:
            break
    return None


async def start_offboard(drone: System):
    # PX4 requires a setpoint BEFORE starting offboard
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
    try:
        await drone.offboard.start()
        print("Offboard started")
    except OffboardError as e:
        print(f"Offboard start failed: {e._result.result}")
        raise


async def arm_and_takeoff(drone: System, alt_m: float):
    print("Arming...")
    await drone.action.arm()

    # Climb by commanding vertical velocity up (negative down)
    # We'll hold N/E near current by commanding 0,0 and climb until -down ~= alt
    print(f"Taking off to {alt_m:.1f} m (Offboard climb)")
    t_end = time.time() + 30.0

    while time.time() < t_end:
        ned = await get_local_ned(drone, timeout_s=1.0)
        if ned is None:
            continue

        n, e, d = ned
        current_alt = -d  # because down is negative when above origin? In NED, altitude ~ -down
        err = alt_m - current_alt

        if err <= 0.3:
            break

        # gentle climb rate
        vz_down = -0.8  # climb up => negative down velocity
        await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, vz_down, 0.0))
        await asyncio.sleep(0.05)

    # stop vertical motion
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
    await asyncio.sleep(0.5)
    print("Takeoff altitude reached")


async def send_stop(drone: System, duration_s: float = 1.0, yaw_deg: float = 0.0):
    end = time.time() + duration_s
    while time.time() < end:
        await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, yaw_deg))
        await asyncio.sleep(0.1)


async def move_to_local_ne(drone: System, target_n: float, target_e: float, speed_ms: float, pos_tol: float, timeout_s: float, yaw_deg: float):
    """
    Moves in local N/E by streaming velocity setpoints. Holds altitude.
    """
    t_end = time.time() + timeout_s

    while time.time() < t_end:
        ned = await get_local_ned(drone, timeout_s=0.5)
        if ned is None:
            continue

        n, e, d = ned
        en = target_n - n
        ee = target_e - e
        dist = math.hypot(en, ee)

        if dist <= pos_tol:
            break

        # normalize and scale
        vn = (en / dist) * speed_ms
        ve = (ee / dist) * speed_ms

        # hold altitude: vz=0 (down velocity 0)
        await drone.offboard.set_velocity_ned(VelocityNedYaw(vn, ve, 0.0, yaw_deg))
        await asyncio.sleep(0.05)

    await send_stop(drone, duration_s=1.0, yaw_deg=yaw_deg)


async def move_by_offset_local(drone: System, dn: float, de: float, speed_ms: float, yaw_deg: float):
    start = await get_local_ned(drone, timeout_s=2.0)
    if start is None:
        raise RuntimeError("No local position available from PX4 telemetry.")

    n0, e0, d0 = start
    tn, te = n0 + dn, e0 + de
    print(f"Move offset N/E dn={dn:.2f}, de={de:.2f} -> target ({tn:.2f}, {te:.2f})")
    await move_to_local_ne(
        drone,
        target_n=tn,
        target_e=te,
        speed_ms=speed_ms,
        pos_tol=POS_TOL_M,
        timeout_s=MOVE_TIMEOUT_S,
        yaw_deg=yaw_deg,
    )


async def turn_relative_stable(drone: System, delta_deg: float):
    """
    In Offboard, yaw is controlled by the yaw setpoint embedded in VelocityNedYaw.
    We "turn" by holding zero velocity and changing yaw until stable within tolerance.
    Returns new yaw_deg setpoint.
    """
    yaw0 = await get_yaw_deg(drone, timeout_s=2.0)
    if yaw0 is None:
        raise RuntimeError("No yaw telemetry.")

    target = (yaw0 + delta_deg) % 360.0
    print(f"Turn {delta_deg:.1f}° -> target yaw {target:.1f}°")

    t_end = time.time() + TURN_TIMEOUT_S
    stable_start = None

    while time.time() < t_end:
        yaw = await get_yaw_deg(drone, timeout_s=0.5)
        if yaw is None:
            continue

        err = abs(deg_wrap_180(target - yaw))
        # hold position while turning
        await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, target))

        if err <= YAW_TOL_DEG:
            if stable_start is None:
                stable_start = time.time()
            elif (time.time() - stable_start) >= YAW_STABLE_TIME_S:
                print(f"Yaw stable (err {err:.2f}°)")
                return target
        else:
            stable_start = None

        await asyncio.sleep(0.05)

    print("Turn timeout; proceeding anyway")
    return target


async def land_and_disarm(drone: System):
    print("Landing...")
    try:
        await drone.offboard.stop()
    except Exception:
        pass
    await drone.action.land()

    # Wait until disarmed
    async for armed in drone.telemetry.armed():
        if not armed:
            print("Disarmed")
            return


async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--connect", default="udp://:14540", help="MAVSDK system address (PX4 SITL default udp://:14540)")
    args = parser.parse_args()

    drone = System()
    print(f"Connecting to {args.connect} ...")
    await drone.connect(system_address=args.connect)

    await wait_connected(drone)
    await wait_global_position_ok(drone)

    # Start offboard
    await start_offboard(drone)

    # Takeoff
    await arm_and_takeoff(drone, ALTITUDE_M)

    # Home
    home = await get_local_ned(drone, timeout_s=2.0)
    if home is None:
        raise RuntimeError("No position data.")
    home_n, home_e, home_d = home
    print(f"🏁 Home recorded at LOCAL_NED: n={home_n:.2f}, e={home_e:.2f}, d={home_d:.2f}")

    print("\n--- STARTING SNAKE PATTERN (LOCAL N/E OFFSETS) ---")
    yaw_set = await get_yaw_deg(drone, timeout_s=2.0)
    if yaw_set is None:
        yaw_set = 0.0

    # Leg 1: Out (+N)
    await move_by_offset_local(drone, +LEG_DISTANCE_M, 0.0, SPEED_MS, yaw_set)

    # Step 1: Right (+E)
    yaw_set = await turn_relative_stable(drone, 90.0)
    await move_by_offset_local(drone, 0.0, +STEP_DISTANCE_M, SPEED_MS, yaw_set)

    # Leg 2: Back (-N)
    yaw_set = await turn_relative_stable(drone, 90.0)
    await move_by_offset_local(drone, -LEG_DISTANCE_M, 0.0, SPEED_MS, yaw_set)

    # Step 2: Right (+E) (match your pattern)
    yaw_set = await turn_relative_stable(drone, -90.0)
    await move_by_offset_local(drone, 0.0, +STEP_DISTANCE_M, SPEED_MS, yaw_set)

    # Leg 3: Out (+N)
    yaw_set = await turn_relative_stable(drone, -90.0)
    await move_by_offset_local(drone, +LEG_DISTANCE_M, 0.0, SPEED_MS, yaw_set)

    # Return home
    print("\n🏁 Returning Home...")
    await move_to_local_ne(drone, home_n, home_e, SPEED_MS, POS_TOL_M, MOVE_TIMEOUT_S, yaw_set)

    # Print final coordinates
    final = await get_local_ned(drone, timeout_s=2.0)
    if final is not None:
        fn, fe, fd = final
        print(f"Final LOCAL_NED: n={fn:.2f}, e={fe:.2f}, d={fd:.2f}")
        print(f"Δ from home: dn={fn-home_n:.2f} m, de={fe-home_e:.2f} m, dalt={(-fd)-(-home_d):.2f} m")

    await land_and_disarm(drone)


if __name__ == "__main__":
    asyncio.run(main())
