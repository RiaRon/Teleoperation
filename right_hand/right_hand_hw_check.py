"""READ-ONLY check of the right LEAP Hand: never writes to any motor.

Use it to fill the TBD values in config/right_hand.json:

  # which baud rate answers, and are IDs 0-15 all present?
  python3 right_hand_hw_check.py --port /dev/serial/by-id/... --baudrates 57600,4000000

  # with torque OFF, pose the hand by hand and record it
  python3 right_hand_hw_check.py --port ... --baudrates 57600 --record open  --output right_open.json
  python3 right_hand_hw_check.py --port ... --baudrates 57600 --record close --output right_close.json
"""

import argparse
import json
from datetime import datetime

from right_hand_controller_core import ALL_IDS, load_hand_config, tick_to_deg


def check_baudrate(port, baudrate):
    from right_hand_dynamixel_bus import DynamixelBus

    bus = DynamixelBus(port, baudrate, ALL_IDS)
    found = {}
    try:
        bus.open()
        for dxl_id in ALL_IDS:
            model, result, error = bus.packet.ping(bus.port, dxl_id)
            if result == bus.sdk.COMM_SUCCESS and error == 0:
                found[dxl_id] = int(model)
    finally:
        bus.close()
    return found


def read_state(port, baudrate):
    from right_hand_dynamixel_bus import DynamixelBus

    bus = DynamixelBus(port, baudrate, ALL_IDS)
    try:
        bus.open()
        return {
            "models": bus.ping_all(),
            "operating_modes": bus.read_operating_modes(),
            "torque": bus.read_torque(),
            "hardware_errors": bus.read_hardware_errors(),
            "present_raw": bus.read_present_raw(),
        }
    finally:
        bus.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--baudrates", default="57600,4000000")
    parser.add_argument("--config", default="config/right_hand.json")
    parser.add_argument("--record", choices=["open", "close"])
    parser.add_argument("--output")
    args = parser.parse_args()

    config = load_hand_config(args.config)
    baudrates = [int(b) for b in args.baudrates.split(",") if b.strip()]

    working = None
    for baudrate in baudrates:
        found = check_baudrate(args.port, baudrate)
        print(f"baud {baudrate}: responding IDs {sorted(found)}")
        if len(found) == len(ALL_IDS):
            working = baudrate
            break

    if working is None:
        print("[STOP] no baud rate found where all IDs 0-15 respond. Check power, port, IDs.")
        return 1

    state = read_state(args.port, working)
    print(f"\nbaud {working}: all 16 IDs respond (models {sorted(set(state['models'].values()))})")
    print("ID | joint              | mode | torque | hw_err | raw   | deg")
    for dxl_id in ALL_IDS:
        raw = state["present_raw"][dxl_id]
        print(
            f"{dxl_id:2d} | {config['joint_names'][str(dxl_id)]:<18} | "
            f"{state['operating_modes'][dxl_id]:4d} | {state['torque'][dxl_id]:6d} | "
            f"{state['hardware_errors'][dxl_id]:6d} | {raw:5d} | {tick_to_deg(raw):7.2f}"
        )

    if args.record:
        if any(state["torque"].values()):
            print("[STOP] torque is ON for some motors; record only a hand posed with torque OFF.")
            return 1
        pose = {str(i): round(tick_to_deg(state["present_raw"][i]), 2) for i in ALL_IDS}
        record = {
            "pose": args.record,
            "recorded_at": datetime.now().isoformat(timespec="seconds"),
            "port": args.port,
            "baudrate": working,
            f"{args.record}_pose_deg": pose,
        }
        text = json.dumps(record, indent=2)
        print(f"\n{text}")
        if args.output:
            with open(args.output, "w", encoding="utf-8") as f:
                f.write(text + "\n")
            print(f"[INFO] saved {args.output}. Review it, then copy into config/right_hand.json by hand.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
