import time
from pymavlink import mavutil


# ============================================================
# CONFIGURATION
# ============================================================

CONNECTION = "udp:127.0.0.1:14540"

TAKEOFF_ALTITUDE = 2.0

SETPOINT_RATE = 20.0
SETPOINT_PERIOD = 1.0 / SETPOINT_RATE


# ============================================================
# CONNECT TO PX4
# ============================================================

def connect_to_px4():
    print("Connecting to PX4...")

    vehicle = mavutil.mavlink_connection(CONNECTION)

    print("Waiting for heartbeat...")
    vehicle.wait_heartbeat()

    print("Connected to PX4!")
    print(f"System ID: {vehicle.target_system}")
    print(f"Component ID: {vehicle.target_component}")

    return vehicle


# ============================================================
# SEND POSITION SETPOINT
# ============================================================

def send_position_setpoint(vehicle, x, y, z):
    """
    Send a LOCAL_NED position setpoint.

    x = North
    y = East
    z = Down

    Therefore:
        z = -2 means 2 meters above the starting position.
    """

    type_mask = 0x0FF8

    vehicle.mav.set_position_target_local_ned_send(
        int(time.time() * 1000) & 0xFFFFFFFF,

        vehicle.target_system,
        vehicle.target_component,

        mavutil.mavlink.MAV_FRAME_LOCAL_NED,

        type_mask,

        x,
        y,
        z,

        0,
        0,
        0,

        0,
        0,
        0,

        0,
        0
    )


# ============================================================
# CONTINUOUS POSITION CONTROL
# ============================================================

def move_to_position(vehicle, x, y, z, duration, description):
    """
    Continuously stream the same position setpoint.

    This is the important part:
    PX4 keeps receiving Offboard setpoints throughout
    the entire movement.
    """

    print(f"\n{description}")

    start_time = time.time()

    while time.time() - start_time < duration:

        send_position_setpoint(vehicle, x, y, z)

        time.sleep(SETPOINT_PERIOD)


# ============================================================
# SET OFFBOARD MODE
# ============================================================

def set_offboard_mode(vehicle):

    print("\nRequesting OFFBOARD mode...")

    # Important:
    # Do NOT use:
    # vehicle.set_mode(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, "OFFBOARD")
    #
    # The correct pymavlink call is simply:

    vehicle.set_mode("OFFBOARD")

    time.sleep(1)

    print("OFFBOARD mode requested.")


# ============================================================
# ARM
# ============================================================

def arm_vehicle(vehicle):

    print("Arming...")

    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,

        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,

        0,

        1,
        0,
        0,
        0,
        0,
        0,
        0
    )

    time.sleep(1)

    print("Arm command sent.")


# ============================================================
# LAND
# ============================================================

def land_vehicle(vehicle):

    print("\nRequesting LAND...")

    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,

        mavutil.mavlink.MAV_CMD_NAV_LAND,

        0,

        0,
        0,
        0,
        0,
        0,
        0,
        0
    )

    print("LAND command sent.")


# ============================================================
# MAIN FLIGHT
# ============================================================

def main():

    vehicle = connect_to_px4()

    print("\nPreparing Offboard setpoints...")

    # --------------------------------------------------------
    # IMPORTANT:
    # PX4 requires setpoints BEFORE entering OFFBOARD.
    # --------------------------------------------------------

    print("Streaming initial setpoints...")

    for _ in range(60):

        send_position_setpoint(
            vehicle,
            0,
            0,
            -TAKEOFF_ALTITUDE
        )

        time.sleep(SETPOINT_PERIOD)

    print("Initial setpoint stream ready.")

    # --------------------------------------------------------
    # OFFBOARD
    # --------------------------------------------------------

    set_offboard_mode(vehicle)

    # Continue streaming immediately after mode change
    for _ in range(20):

        send_position_setpoint(
            vehicle,
            0,
            0,
            -TAKEOFF_ALTITUDE
        )

        time.sleep(SETPOINT_PERIOD)

    # --------------------------------------------------------
    # ARM
    # --------------------------------------------------------

    arm_vehicle(vehicle)

    # Keep streaming after arming
    for _ in range(20):

        send_position_setpoint(
            vehicle,
            0,
            0,
            -TAKEOFF_ALTITUDE
        )

        time.sleep(SETPOINT_PERIOD)

    # --------------------------------------------------------
    # TAKEOFF / HOVER
    # --------------------------------------------------------

    move_to_position(
        vehicle,
        0,
        0,
        -TAKEOFF_ALTITUDE,
        5,
        "Taking off to approximately 2 meters..."
    )

    # --------------------------------------------------------
    # MOVE FORWARD
    # --------------------------------------------------------

    move_to_position(
        vehicle,
        3,
        0,
        -TAKEOFF_ALTITUDE,
        3,
        "Moving FORWARD for 3 seconds..."
    )

    # --------------------------------------------------------
    # HOLD
    # --------------------------------------------------------

    move_to_position(
        vehicle,
        3,
        0,
        -TAKEOFF_ALTITUDE,
        2,
        "Holding position for 2 seconds..."
    )

    # --------------------------------------------------------
    # MOVE RIGHT
    # --------------------------------------------------------

    move_to_position(
        vehicle,
        3,
        3,
        -TAKEOFF_ALTITUDE,
        3,
        "Moving RIGHT for 3 seconds..."
    )

    # --------------------------------------------------------
    # HOLD
    # --------------------------------------------------------

    move_to_position(
        vehicle,
        3,
        3,
        -TAKEOFF_ALTITUDE,
        2,
        "Holding position for 2 seconds..."
    )

    print("\nFlight path complete.")

    # --------------------------------------------------------
    # LAND
    # --------------------------------------------------------

    land_vehicle(vehicle)

    # Give PX4 time to execute landing
    time.sleep(5)

    print("\nFlight test complete.")


# ============================================================
# PROGRAM ENTRY
# ============================================================

if __name__ == "__main__":
    main()
