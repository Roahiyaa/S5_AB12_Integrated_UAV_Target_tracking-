from pymavlink import mavutil
import time


def connect_to_px4():
    print("Connecting to PX4...")

    connection = mavutil.mavlink_connection(
        "udp:127.0.0.1:14540"
    )

    print("Waiting for PX4 heartbeat...")

    connection.wait_heartbeat()

    print("Connected to PX4!")
    print(f"System ID: {connection.target_system}")
    print(f"Component ID: {connection.target_component}")

    return connection


def main():
    connection = connect_to_px4()

    print("\nPX4 connection successful.")
    print("Waiting for telemetry...\n")

    while True:
        message = connection.recv_match(
            type="GLOBAL_POSITION_INT",
            blocking=True
        )

        if message:
            print(
                f"Latitude:  {message.lat / 1e7:.7f}"
            )
            print(
                f"Longitude: {message.lon / 1e7:.7f}"
            )
            print(
                f"Altitude:  {message.relative_alt / 1000:.2f} m"
            )
            print("-" * 40)

        time.sleep(1)


if __name__ == "__main__":
    main()
