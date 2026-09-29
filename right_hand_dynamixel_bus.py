"""Real Dynamixel bus for the right LEAP Hand (XC330, Protocol 2.0).

Same control table addresses as left_hand_finger_ratio_node_safe.py.
dynamixel_sdk is imported lazily so dry-run never needs or touches it.
"""

from right_hand_controller_core import tick_to_deg


ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_HARDWARE_ERROR = 70
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
LEN_GOAL_POSITION = 4


def to_signed32(value):
    # Present Position is a signed 32-bit register; never wrap it with % 4096,
    # a multi-turn value must show up as out of range instead.
    value = int(value) & 0xFFFFFFFF
    return value - (1 << 32) if value & 0x80000000 else value


class DynamixelBus:
    def __init__(self, port, baudrate, ids, protocol_version=2.0):
        import dynamixel_sdk as sdk

        self.sdk = sdk
        self.port_name = port
        self.baudrate = int(baudrate)
        self.ids = list(ids)
        self.port = sdk.PortHandler(port)
        self.packet = sdk.PacketHandler(protocol_version)
        self.opened = False

    def open(self):
        if not self.port.openPort():
            raise RuntimeError(f"port open failed: {self.port_name}")
        self.opened = True
        if not self.port.setBaudRate(self.baudrate):
            self.close()
            raise RuntimeError(f"baudrate set failed: {self.baudrate}")

    def _check(self, dxl_id, result, error, label):
        if result != self.sdk.COMM_SUCCESS:
            raise RuntimeError(f"ID {dxl_id} {label}: {self.packet.getTxRxResult(result)}")
        if error != 0:
            raise RuntimeError(f"ID {dxl_id} {label}: {self.packet.getRxPacketError(error)}")

    def _read1(self, address, label):
        values = {}
        for dxl_id in self.ids:
            value, result, error = self.packet.read1ByteTxRx(self.port, dxl_id, address)
            self._check(dxl_id, result, error, label)
            values[dxl_id] = int(value)
        return values

    def _write_each(self, address, value, size, label):
        write = self.packet.write1ByteTxRx if size == 1 else self.packet.write4ByteTxRx
        for dxl_id in self.ids:
            result, error = write(self.port, dxl_id, address, int(value))
            self._check(dxl_id, result, error, label)

    def ping_all(self):
        models = {}
        for dxl_id in self.ids:
            model, result, error = self.packet.ping(self.port, dxl_id)
            self._check(dxl_id, result, error, "ping")
            models[dxl_id] = int(model)
        return models

    def read_operating_modes(self):
        return self._read1(ADDR_OPERATING_MODE, "operating mode")

    def read_torque(self):
        return self._read1(ADDR_TORQUE_ENABLE, "torque enable")

    def read_hardware_errors(self):
        return self._read1(ADDR_HARDWARE_ERROR, "hardware error")

    def read_present_raw(self):
        raw = {}
        for dxl_id in self.ids:
            value, result, error = self.packet.read4ByteTxRx(self.port, dxl_id, ADDR_PRESENT_POSITION)
            self._check(dxl_id, result, error, "present position")
            raw[dxl_id] = to_signed32(value)
        return raw

    def read_present_deg(self):
        return {dxl_id: tick_to_deg(raw) for dxl_id, raw in self.read_present_raw().items()}

    def set_profile_velocity(self, value):
        self._write_each(ADDR_PROFILE_VELOCITY, value, 4, "profile velocity")

    def torque_on(self):
        self._write_each(ADDR_TORQUE_ENABLE, 1, 1, "torque on")

    def torque_off(self):
        # Try every motor even if one fails, then report.
        failures = []
        for dxl_id in self.ids:
            try:
                result, error = self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, 0)
                self._check(dxl_id, result, error, "torque off")
            except Exception as e:
                failures.append(str(e))
        if failures:
            raise RuntimeError("torque off failed: " + "; ".join(failures))

    def write_goal_ticks(self, goal_ticks):
        sdk = self.sdk
        group = sdk.GroupSyncWrite(self.port, self.packet, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
        for dxl_id, tick in goal_ticks.items():
            tick = int(tick)
            param = [
                sdk.DXL_LOBYTE(sdk.DXL_LOWORD(tick)),
                sdk.DXL_HIBYTE(sdk.DXL_LOWORD(tick)),
                sdk.DXL_LOBYTE(sdk.DXL_HIWORD(tick)),
                sdk.DXL_HIBYTE(sdk.DXL_HIWORD(tick)),
            ]
            if not group.addParam(int(dxl_id), param):
                raise RuntimeError(f"ID {dxl_id} groupSyncWrite addParam failed")
        result = group.txPacket()
        group.clearParam()
        if result != sdk.COMM_SUCCESS:
            raise RuntimeError(f"groupSyncWrite failed: {self.packet.getTxRxResult(result)}")

    def close(self):
        if self.opened:
            self.port.closePort()
            self.opened = False
