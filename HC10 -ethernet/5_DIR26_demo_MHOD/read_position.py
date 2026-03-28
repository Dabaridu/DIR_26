from yrc_high_speed_ethernet import ClientOfYRC
import struct

ROBOT_IP = "172.16.0.1"


def unpack_int32(data_bytes):
    """Pretvori 4 byte v signed 32-bit integer."""
    return struct.unpack("=l", data_bytes)[0]


def main():
    yrc = ClientOfYRC(ROBOT_IP)

    # Preberi status
    status = yrc.status_information_reading()
    print("STATUS:")
    print(status)
    print()

    # Preberi pozicijske podatke
    data = yrc.robot_position_data_read()

    if data is None:
        print("Ni odgovora od robota.")
        return

    # Če robot vrne error tuple namesto podatkov
    if isinstance(data, tuple):
        print("Napaka pri branju pozicije:", data)
        return

    print("RAW POSITION DATA:")
    print(data)
    print()

    # Razpakiraj osi
    x_raw = unpack_int32(bytes(data["first_axis_data"]))
    y_raw = unpack_int32(bytes(data["second_axis_data"]))
    z_raw = unpack_int32(bytes(data["third_axis_data"]))
    tx_raw = unpack_int32(bytes(data["fourth_axis_data"]))
    ty_raw = unpack_int32(bytes(data["fifth_axis_data"]))
    tz_raw = unpack_int32(bytes(data["sixth_axis_data"]))

    print("RAW VALUES:")
    print(f"X  = {x_raw}")
    print(f"Y  = {y_raw}")
    print(f"Z  = {z_raw}")
    print(f"Tx = {tx_raw}")
    print(f"Ty = {ty_raw}")
    print(f"Tz = {tz_raw}")
    print()

    # Pretvorba v človeku berljive enote
    x_mm = x_raw / 1000
    y_mm = y_raw / 1000
    z_mm = z_raw / 1000

    tx_deg = tx_raw / 10000
    ty_deg = ty_raw / 10000
    tz_deg = tz_raw / 10000

    print("HUMAN READABLE POSITION:")
    print(f"X  = {x_mm:.3f} mm")
    print(f"Y  = {y_mm:.3f} mm")
    print(f"Z  = {z_mm:.3f} mm")
    print(f"Tx = {tx_deg:.4f} deg")
    print(f"Ty = {ty_deg:.4f} deg")
    print(f"Tz = {tz_deg:.4f} deg")


if __name__ == "__main__":
    main()