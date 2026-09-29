from dynamixel_sdk import *

PORT = "/dev/ttyUSB0"
BAUD = 57600
PROTOCOL = 2.0

ADDR_TORQUE_ENABLE = 64
ADDR_HW_ERROR = 70
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_VOLTAGE = 144
ADDR_PRESENT_TEMP = 146

port = PortHandler(PORT)
pkt = PacketHandler(PROTOCOL)

if not port.openPort():
    print("port open failed")
    raise SystemExit

if not port.setBaudRate(BAUD):
    print("baudrate set failed")
    raise SystemExit

print("ID | torque | hw_error | current | position | voltage | temp")
print("-" * 70)

for dxl_id in range(16):
    torque, r1, e1 = pkt.read1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE)
    hw, r2, e2 = pkt.read1ByteTxRx(port, dxl_id, ADDR_HW_ERROR)
    cur, r3, e3 = pkt.read2ByteTxRx(port, dxl_id, ADDR_PRESENT_CURRENT)
    pos, r4, e4 = pkt.read4ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)
    volt, r5, e5 = pkt.read2ByteTxRx(port, dxl_id, ADDR_PRESENT_VOLTAGE)
    temp, r6, e6 = pkt.read1ByteTxRx(port, dxl_id, ADDR_PRESENT_TEMP)

    if r1 != COMM_SUCCESS:
        print(f"ID {dxl_id}: NO RESPONSE")
        continue

    if cur > 32767:
        cur -= 65536

    print(f"{dxl_id:2d} | {torque:6d} | {hw:8d} | {cur:7d} | {pos:8d} | {volt/10:6.1f}V | {temp:3d}C")

port.closePort()
