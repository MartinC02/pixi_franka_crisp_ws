#!/usr/bin/env python3
"""Probe the RIGHT RH-P12-RN-A after the DC-socket repair. READ-ONLY.

1. Try ping at the expected baud (2M, id 1).
2. If that fails, scan common baud rates and all IDs 0..252 (broadcast ping).
3. If the motor answers, read voltage / hardware error / mode / position.

With --blink, instead ping id 1 continuously (~20/s) so you can watch the
adapter LEDs: the LED that flickers = TXD (host transmitting); a second LED
flickering with it = RXD (the motor actually answering). Ctrl-C to stop.
"""
import sys
import time

from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

PROTOCOL = 2.0
RIGHT_PORT = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT792A6Q-if00-port0"
EXPECTED_BAUD = 2_000_000
EXPECTED_ID = 1
BAUDS = [2_000_000, 57_600, 1_000_000, 115_200, 3_000_000, 4_000_000, 9_600]

CHECKS = [  # (name, addr, size, signed, scale, unit)
    ("Model Number",        0,   2, False, 1,    ""),
    ("Firmware Version",    6,   1, False, 1,    ""),
    ("Operating Mode",      11,  1, False, 1,    ""),
    ("Homing Offset",       20,  4, True,  1,    ""),
    ("Current Limit",       38,  2, False, 1,    ""),
    ("Max Position Limit",  48,  4, True,  1,    ""),
    ("Min Position Limit",  52,  4, True,  1,    ""),
    ("Shutdown",            63,  1, False, 1,    ""),
    ("Torque Enable",       512, 1, False, 1,    ""),
    ("Hardware Error",      518, 1, False, 1,    ""),
    ("Present Position",    580, 4, True,  1,    ""),
    ("Present Input Volt",  592, 2, False, 0.1,  "V"),
    ("Present Temp",        594, 1, False, 1,    "C"),
]

HW_ERR_BITS = {
    0: "Input Voltage Error",
    2: "Overheating",
    3: "Motor Encoder Error",
    4: "Electrical Shock Error",
    5: "Overload Error",
}


def to_signed(v, size):
    bits = size * 8
    return v - (1 << bits) if v >= (1 << (bits - 1)) else v


def read_checks(ph, pk, dxl_id):
    readers = {1: pk.read1ByteTxRx, 2: pk.read2ByteTxRx, 4: pk.read4ByteTxRx}
    for name, addr, size, signed, scale, unit in CHECKS:
        val, comm, err = readers[size](ph, dxl_id, addr)
        if comm != COMM_SUCCESS:
            print(f"  {name:<22} <comm fail: {pk.getTxRxResult(comm)}>")
            continue
        v = to_signed(val, size) if signed else val
        shown = f"{v * scale:.1f}{unit}" if scale != 1 else f"{v}{unit}"
        extra = ""
        if name == "Hardware Error" and v:
            errs = [d for b, d in HW_ERR_BITS.items() if v & (1 << b)]
            extra = "  <-- " + ", ".join(errs)
        if err:
            extra += f"  (status err byte: {pk.getRxPacketError(err)})"
        print(f"  {name:<22} {shown}{extra}")


def blink_loop(ph, pk):
    print(f"Pinging id {EXPECTED_ID} @ {EXPECTED_BAUD} continuously -- watch the adapter LEDs.")
    print("Flickering LED = TXD (host transmitting). Second LED = RXD (motor answering).")
    print("Ctrl-C to stop.")
    n = ok = 0
    try:
        while True:
            _, comm, _ = pk.ping(ph, EXPECTED_ID)
            n += 1
            ok += comm == COMM_SUCCESS
            if n % 100 == 0:
                print(f"  {n} pings sent, {ok} answered")
            time.sleep(0.05)
    except KeyboardInterrupt:
        print(f"\nstopped: {n} pings sent, {ok} answered")


def main():
    args = [a for a in sys.argv[1:] if a != "--blink"]
    blink = "--blink" in sys.argv[1:]
    port = args[0] if args else RIGHT_PORT
    ph = PortHandler(port)
    pk = PacketHandler(PROTOCOL)
    if not ph.openPort():
        sys.exit(f"could not open {port} (busy?)")

    ph.setBaudRate(EXPECTED_BAUD)
    if blink:
        blink_loop(ph, pk)
        ph.closePort()
        return
    model, comm, err = pk.ping(ph, EXPECTED_ID)
    if comm == COMM_SUCCESS:
        print(f"PING OK at {EXPECTED_BAUD} baud, id {EXPECTED_ID}, model {model}")
        read_checks(ph, pk, EXPECTED_ID)
        ph.closePort()
        return

    print(f"ping id {EXPECTED_ID} @ {EXPECTED_BAUD}: FAILED ({pk.getTxRxResult(comm)})")
    print("scanning other bauds/ids...")
    for baud in BAUDS:
        if not ph.setBaudRate(baud):
            print(f"  baud {baud}: not settable on this adapter")
            continue
        found = pk.broadcastPing(ph)
        devs = found[0] if isinstance(found, tuple) else found
        if devs:
            for dxl_id, (model, fw) in devs.items():
                print(f"  baud {baud}: FOUND id {dxl_id} model {model} fw {fw}")
                read_checks(ph, pk, dxl_id)
            ph.closePort()
            return
        print(f"  baud {baud}: nothing")
    ph.closePort()
    print("\nNO DEVICE FOUND on the right bus at any baud/id.")
    print("=> motor electronics not powered or not talking (power/polarity/board damage).")


if __name__ == "__main__":
    main()
