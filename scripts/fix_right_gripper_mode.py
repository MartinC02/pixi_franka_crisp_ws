#!/usr/bin/env python3
"""Fix the RIGHT RH-P12-RN-A gripper: set Operating Mode = 5 (Current-based Position),
matching the LEFT, then verify the motor now tracks Goal Position.

Root cause (confirmed by scripts/compare_gripper_motors.py): the right motor's EEPROM
Operating Mode was 0 (Current Control) instead of 5 (Current-based Position). In mode 0
the motor ignores Goal Position and only follows Goal Current (which was 0), so it sat
limp and ignored every position command. The dynamixel_hardware_interface never sets
Operating Mode at init, so the stale value survived every relaunch and reboot.

READ + WRITE + MOVES THE GRIPPER. Run with the ROS stack STOPPED (serial ports free):

    pixi run -e humble python scripts/fix_right_gripper_mode.py

Steps: write Operating Mode=5 and Current Limit=300 to EEPROM (torque off, to match the
left), briefly drive the gripper to confirm it now tracks position, then return it to
open and disable torque so ROS can take over cleanly. Operating Mode persists in EEPROM.
"""
import sys
import time

try:
    from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
except ImportError:
    sys.exit("dynamixel_sdk not importable. Install with: pip install dynamixel_sdk")

PROTOCOL = 2.0
BAUD = 2_000_000
DXL_ID = 1
RIGHT_PORT = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT792A6Q-if00-port0"

ADDR_OPERATING_MODE = 11    # 1B EEPROM
ADDR_CURRENT_LIMIT = 38     # 2B EEPROM
ADDR_TORQUE_ENABLE = 512    # 1B
ADDR_GOAL_CURRENT = 550     # 2B
ADDR_GOAL_POSITION = 564    # 4B
ADDR_PRESENT_POSITION = 580  # 4B

MODE_CURRENT_BASED_POSITION = 5
TARGET_CURRENT = 300  # match the LEFT motor's Goal Current / Current Limit


def chk(pk, comm, err, what):
    if comm != COMM_SUCCESS:
        sys.exit(f"FAILED {what}: {pk.getTxRxResult(comm)}")
    if err != 0:
        sys.exit(f"FAILED {what}: {pk.getRxPacketError(err)}")


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else RIGHT_PORT
    ph = PortHandler(port)
    pk = PacketHandler(PROTOCOL)
    if not ph.openPort():
        sys.exit(f"Could not open {port} (busy? is the ROS stack still running?)")
    if not ph.setBaudRate(BAUD):
        sys.exit(f"Could not set baud {BAUD}")

    model, comm, err = pk.ping(ph, DXL_ID)
    chk(pk, comm, err, "ping")
    print(f"Connected to id {DXL_ID}, model {model} on {port}")

    mode, comm, err = pk.read1ByteTxRx(ph, DXL_ID, ADDR_OPERATING_MODE)
    chk(pk, comm, err, "read Operating Mode")
    print(f"Operating Mode before = {mode}")

    if mode != MODE_CURRENT_BASED_POSITION:
        comm, err = pk.write1ByteTxRx(ph, DXL_ID, ADDR_TORQUE_ENABLE, 0)  # EEPROM needs torque off
        chk(pk, comm, err, "disable torque")
        comm, err = pk.write1ByteTxRx(ph, DXL_ID, ADDR_OPERATING_MODE, MODE_CURRENT_BASED_POSITION)
        chk(pk, comm, err, "write Operating Mode = 5")
        comm, err = pk.write2ByteTxRx(ph, DXL_ID, ADDR_CURRENT_LIMIT, TARGET_CURRENT)
        chk(pk, comm, err, "write Current Limit = 300")
        rb, comm, err = pk.read1ByteTxRx(ph, DXL_ID, ADDR_OPERATING_MODE)
        chk(pk, comm, err, "read back Operating Mode")
        print(f"Operating Mode after  = {rb} (expected 5); Current Limit set to {TARGET_CURRENT}")
    else:
        print("Already in mode 5 (Current-based Position); leaving EEPROM as-is.")

    def present():
        v, c, e = pk.read4ByteTxRx(ph, DXL_ID, ADDR_PRESENT_POSITION)
        return v - (1 << 32) if v >= (1 << 31) else v

    print("\nVerifying position tracking (the RIGHT gripper will move)...")
    comm, err = pk.write1ByteTxRx(ph, DXL_ID, ADDR_TORQUE_ENABLE, 1)
    chk(pk, comm, err, "enable torque")
    comm, err = pk.write2ByteTxRx(ph, DXL_ID, ADDR_GOAL_CURRENT, TARGET_CURRENT)
    chk(pk, comm, err, "set Goal Current")

    ok = True
    for goal in (150, 500, 50):
        comm, err = pk.write4ByteTxRx(ph, DXL_ID, ADDR_GOAL_POSITION, goal)
        chk(pk, comm, err, f"write Goal Position {goal}")
        time.sleep(1.3)
        pos = present()
        moved = abs(pos - goal) < 60
        ok = ok and moved
        print(f"  goal={goal:4d}  present={pos:5d}  {'OK' if moved else 'NOT TRACKING'}")

    pk.write4ByteTxRx(ph, DXL_ID, ADDR_GOAL_POSITION, 0)  # return to open
    time.sleep(1.0)
    print(f"  returned to open, present={present()}")
    pk.write1ByteTxRx(ph, DXL_ID, ADDR_TORQUE_ENABLE, 0)  # release for ROS
    ph.closePort()

    print("\n" + ("SUCCESS: right gripper now tracks position commands."
                  if ok else "WARNING: motor still not tracking — investigate further."))
    print("Torque disabled; Operating Mode 5 persisted in EEPROM.")


if __name__ == "__main__":
    main()
