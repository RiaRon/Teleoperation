import argparse
import json
import time
from pathlib import Path

from dynamixel_sdk import (
    PortHandler,
    PacketHandler,
    GroupSyncWrite,
    COMM_SUCCESS,
    DXL_LOBYTE,
    DXL_HIBYTE,
    DXL_LOWORD,
    DXL_HIWORD,
)


MAP_FILE = Path("left_hand_motor_map.json")

PROTOCOL_VERSION = 2.0

# XC330-M288 / Protocol 2.0 control table
ADDR_TORQUE_ENABLE = 64
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116

LEN_GOAL_POSITION = 4

TORQUE_OFF = 0
TORQUE_ON = 1

FINGER_IDS = {
    "thumb": [0, 1, 2, 3],
    "index": [4, 5, 6, 7],
    "middle": [8, 9, 10, 11],
    "ring": [12, 13, 14, 15],
    "all": list(range(16)),
}


def shortest_angle_delta(open_deg, close_deg):
    """
    360도 경계에서 가장 짧은 방향으로 이동하게 만드는 함수.
    예: 360.53 -> 32.87 은 -327.66도가 아니라 +32.34도로 처리.
    """
    delta = close_deg - open_deg

    if delta > 180:
        delta -= 360
    elif delta < -180:
        delta += 360

    return delta


def deg_to_tick(deg):
    """
    0~360 deg를 Dynamixel position tick 0~4095로 변환.
    """
    deg = deg % 360.0
    return int(round(deg / 360.0 * 4095))


def make_target_pose(data, ratio, selected_ids):
    open_pose = data["open_pose_deg"]
    close_pose = data["close_pose_deg"]

    targets = {}

    for motor_id in selected_ids:
        key = str(motor_id)

        open_deg = float(open_pose[key])
        close_deg = float(close_pose[key])

        delta = shortest_angle_delta(open_deg, close_deg)
        target_deg = (open_deg + delta * ratio) % 360.0
        target_tick = deg_to_tick(target_deg)

        targets[motor_id] = {
            "deg": round(target_deg, 2),
            "tick": target_tick,
        }

    return targets


def parse_ids(args):
    """
    --ids 0,1,2,3 이 있으면 그걸 우선 사용.
    없으면 --finger 기준 사용.
    """
    if args.ids:
        return [int(x.strip()) for x in args.ids.split(",") if x.strip() != ""]

    return FINGER_IDS[args.finger]


def check_comm(packet_handler, dxl_id, result, error, action):
    if result != COMM_SUCCESS:
        print(f"[ERROR] ID {dxl_id} {action}: {packet_handler.getTxRxResult(result)}")
        return False

    if error != 0:
        print(f"[ERROR] ID {dxl_id} {action}: {packet_handler.getRxPacketError(error)}")
        return False

    return True


def set_torque(packet_handler, port_handler, ids, enable):
    for dxl_id in ids:
        result, error = packet_handler.write1ByteTxRx(
            port_handler,
            dxl_id,
            ADDR_TORQUE_ENABLE,
            TORQUE_ON if enable else TORQUE_OFF,
        )

        ok = check_comm(
            packet_handler,
            dxl_id,
            result,
            error,
            "torque on" if enable else "torque off",
        )

        if not ok:
            return False

    return True


def set_profile_velocity(packet_handler, port_handler, ids, velocity):
    for dxl_id in ids:
        result, error = packet_handler.write4ByteTxRx(
            port_handler,
            dxl_id,
            ADDR_PROFILE_VELOCITY,
            velocity,
        )

        ok = check_comm(
            packet_handler,
            dxl_id,
            result,
            error,
            "profile velocity",
        )

        if not ok:
            return False

    return True


def sync_write_goal_position(packet_handler, port_handler, targets):
    group = GroupSyncWrite(
        port_handler,
        packet_handler,
        ADDR_GOAL_POSITION,
        LEN_GOAL_POSITION,
    )

    for dxl_id, target in targets.items():
        tick = int(target["tick"])

        param_goal_position = [
            DXL_LOBYTE(DXL_LOWORD(tick)),
            DXL_HIBYTE(DXL_LOWORD(tick)),
            DXL_LOBYTE(DXL_HIWORD(tick)),
            DXL_HIBYTE(DXL_HIWORD(tick)),
        ]

        if not group.addParam(dxl_id, param_goal_position):
            print(f"[ERROR] ID {dxl_id} groupSyncWrite addParam failed")
            group.clearParam()
            return False

    result = group.txPacket()

    if result != COMM_SUCCESS:
        print(f"[ERROR] groupSyncWrite txPacket: {packet_handler.getTxRxResult(result)}")
        group.clearParam()
        return False

    group.clearParam()
    return True


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--port", default="COM6")
    parser.add_argument("--baudrate", type=int, default=None)
    parser.add_argument("--ratio", type=float, default=0.0)
    parser.add_argument("--finger", choices=list(FINGER_IDS.keys()), default="thumb")
    parser.add_argument("--ids", default=None, help="예: 0 또는 0,1,2,3")
    parser.add_argument("--profile-velocity", type=int, default=30)
    parser.add_argument("--hold", type=float, default=1.0)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--keep-torque", action="store_true")

    args = parser.parse_args()

    if not MAP_FILE.exists():
        raise FileNotFoundError("left_hand_motor_map.json 파일이 같은 폴더에 없음")

    with open(MAP_FILE, "r", encoding="utf-8") as f:
        data = json.load(f)

    baudrate = args.baudrate if args.baudrate is not None else int(data["baudrate"])
    ratio = max(0.0, min(1.0, args.ratio))
    selected_ids = parse_ids(args)

    targets = make_target_pose(data, ratio, selected_ids)

    print()
    print("LEFT HAND DXL RUN")
    print("=================")
    print(f"port      : {args.port}")
    print(f"baudrate  : {baudrate}")
    print(f"ratio     : {ratio}")
    print(f"ids       : {selected_ids}")
    print(f"execute   : {args.execute}")
    print()

    for dxl_id in selected_ids:
        deg = targets[dxl_id]["deg"]
        tick = targets[dxl_id]["tick"]
        print(f"ID {dxl_id:>2}: {deg:>7} deg -> tick {tick:>4}")

    if not args.execute:
        print()
        print("[DRY RUN] 실제 모터에는 명령 안 보냄.")
        print("실제로 움직이려면 명령 뒤에 --execute 붙여.")
        return

    port_handler = PortHandler(args.port)
    packet_handler = PacketHandler(PROTOCOL_VERSION)

    if not port_handler.openPort():
        print(f"[ERROR] 포트 열기 실패: {args.port}")
        return

    if not port_handler.setBaudRate(baudrate):
        print(f"[ERROR] baudrate 설정 실패: {baudrate}")
        port_handler.closePort()
        return

    print()
    print("[OK] port open")

    try:
        print("[STEP] torque off")
        if not set_torque(packet_handler, port_handler, selected_ids, False):
            return

        print("[STEP] set profile velocity")
        if not set_profile_velocity(packet_handler, port_handler, selected_ids, args.profile_velocity):
            return

        print("[STEP] torque on")
        if not set_torque(packet_handler, port_handler, selected_ids, True):
            return

        print("[STEP] send goal position")
        if not sync_write_goal_position(packet_handler, port_handler, targets):
            return

        print(f"[STEP] hold {args.hold} sec")
        time.sleep(args.hold)

    finally:
        if not args.keep_torque:
            print("[STEP] torque off")
            set_torque(packet_handler, port_handler, selected_ids, False)

        port_handler.closePort()
        print("[DONE]")


if __name__ == "__main__":
    main()