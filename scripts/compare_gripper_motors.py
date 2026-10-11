#!/usr/bin/env python3
"""Compare the control tables of the two RH-P12-RN-A gripper Dynamixels (left vs right).

READ-ONLY. Talks Dynamixel Protocol 2.0 directly over the serial ports, so it does
NOT need ROS running -- in fact the ROS stack must be STOPPED first (otherwise the
serial ports are busy). Stop the `robot_run` tmux session, then run:

    pixi run -e humble python scripts/compare_gripper_motors.py

Prints every register side by side and flags differences, focusing on the ones that
would explain "the right motor won't drive to a commanded position".
"""
import sys

try:
    from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
except ImportError:
    sys.exit("dynamixel_sdk not importable in this env. Install with: pip install dynamixel_sdk")

PROTOCOL = 2.0
BAUD = 2_000_000
DXL_ID = 1

# Stable by-id paths (ttyUSB numbers can re-enumerate). Override via argv if needed.
LEFT_PORT = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT45BL9X-if00-port0"
RIGHT_PORT = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT792A6Q-if00-port0"

# RH-P12-RN-A control table (Protocol 2.0):  (name, address, size_bytes, signed)
REGISTERS = [
    # ----- EEPROM (persistent config -- the prime suspects) -----
    ("Model Number",        0,  2, False),
    ("Firmware Version",    6,  1, False),
    ("ID",                  7,  1, False),
    ("Baud Rate",           8,  1, False),
    ("Drive Mode",          10, 1, False),   # bit0=1 reverse direction; bit2=1 torque-on-by-goal
    ("Operating Mode",      11, 1, False),   # 3=Pos 4=ExtPos 5=Curr-Pos 1=Vel 0=Curr 16=PWM
    ("Homing Offset",       20, 4, True),
    ("Moving Threshold",    24, 4, False),
    ("Temperature Limit",   31, 1, False),
    ("Max Voltage Limit",   32, 2, False),
    ("Min Voltage Limit",   34, 2, False),
    ("PWM Limit",           36, 2, False),
    ("Current Limit",       38, 2, False),
    ("Velocity Limit",      44, 4, False),
    ("Max Position Limit",  48, 4, True),
    ("Min Position Limit",  52, 4, True),
    ("Shutdown",            63, 1, False),
    # ----- RAM (live state) -----
    ("Torque Enable",       512, 1, False),
    ("Hardware Error",      518, 1, False),
    ("Position D Gain",     528, 2, False),
    ("Position I Gain",     530, 2, False),
    ("Position P Gain",     532, 2, False),
    ("Goal PWM",            548, 2, True),
    ("Goal Current",        550, 2, True),
    ("Profile Accel",       556, 4, False),
    ("Profile Velocity",    560, 4, False),
    ("Goal Position",       564, 4, True),
    ("Moving",              570, 1, False),
    ("Moving Status",       571, 1, False),
    ("Present PWM",         572, 2, True),
    ("Present Current",     574, 2, True),
    ("Present Position",    580, 4, True),
    ("Present Input Volt",  592, 2, False),
    ("Present Temp",        594, 1, False),
]

# Registers most relevant to the "won't actuate to goal" diagnosis.
SUSPECTS = {
    "Drive Mode", "Operating Mode", "Homing Offset", "Max Position Limit",
    "Min Position Limit", "Torque Enable", "Hardware Error", "Shutdown",
    "PWM Limit", "Current Limit", "Position P Gain", "Goal Position",
    "Present Position", "Moving Status",
}


def to_signed(value: int, size: int) -> int:
    bits = size * 8
    return value - (1 << bits) if value >= (1 << (bits - 1)) else value


def read_all(port_path: str):
    ph = PortHandler(port_path)
    pk = PacketHandler(PROTOCOL)
    if not ph.openPort():
        return None, f"could not open {port_path} (busy? is the ROS stack still running?)"
    if not ph.setBaudRate(BAUD):
        ph.closePort()
        return None, f"could not set baud {BAUD}"
    model, comm, _ = pk.ping(ph, DXL_ID)
    if comm != COMM_SUCCESS:
        ph.closePort()
        return None, f"ping of id {DXL_ID} failed: {pk.getTxRxResult(comm)}"
    readers = {1: pk.read1ByteTxRx, 2: pk.read2ByteTxRx, 4: pk.read4ByteTxRx}
    out = {"_ping_model": model}
    for name, addr, size, signed in REGISTERS:
        val, comm, err = readers[size](ph, DXL_ID, addr)
        if comm != COMM_SUCCESS or err != 0:
            out[name] = None
        else:
            out[name] = to_signed(val, size) if signed else val
    ph.closePort()
    return out, None


def main() -> None:
    left_port = sys.argv[1] if len(sys.argv) > 1 else LEFT_PORT
    right_port = sys.argv[2] if len(sys.argv) > 2 else RIGHT_PORT

    left, lerr = read_all(left_port)
    right, rerr = read_all(right_port)
    if lerr:
        print(f"LEFT  ({left_port}): ERROR: {lerr}")
    if rerr:
        print(f"RIGHT ({right_port}): ERROR: {rerr}")
    if not left or not right:
        sys.exit(1)

    print(f"\nPing model number:  left={left['_ping_model']}   right={right['_ping_model']}\n")
    hdr = f"  {'Register':<20}{'LEFT':>14}{'RIGHT':>14}   DIFF"
    print(hdr)
    print("  " + "-" * (len(hdr) - 2))
    for name, _addr, _size, _signed in REGISTERS:
        lv, rv = left.get(name), right.get(name)
        diff = "  <-- DIFF" if lv != rv else ""
        star = "*" if name in SUSPECTS else " "
        print(f"{star} {name:<20}{str(lv):>14}{str(rv):>14}{diff}")

    print("\n* = registers most relevant to 'right motor won't drive to goal'.")
    print("Operating Mode: 3=Position 4=ExtendedPosition 5=Current-based-Position 1=Velocity 0=Current 16=PWM")
    print("Drive Mode: bit0=1 -> reversed direction, bit2=1 -> torque on by goal update")


if __name__ == "__main__":
    main()
