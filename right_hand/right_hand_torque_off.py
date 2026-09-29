"""Emergency torque OFF for the right LEAP Hand, directly over the serial port.

Used by LEAP-Right-Disarm-Jazzy.sh only when no right controller is listening
on /right_hand/arm. It writes nothing but Torque Enable = 0 to IDs 0-15.
Do not run it while the right controller is alive: two processes on one
serial bus corrupt packets.
"""

import argparse

from right_hand_controller_core import ALL_IDS, load_hand_config


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="config/right_hand.json")
    parser.add_argument("--port")
    parser.add_argument("--baudrate", type=int)
    args = parser.parse_args()

    config = load_hand_config(args.config)
    port = args.port or config.get("port")
    baudrate = args.baudrate or config.get("baudrate")

    if not port or not baudrate:
        print("[ERROR] right hand port/baudrate are TBD in the config; cannot torque off directly.")
        print("        Cut the right hand power if it is holding torque.")
        return 1

    from right_hand_dynamixel_bus import DynamixelBus

    bus = DynamixelBus(port, baudrate, ALL_IDS, config["protocol_version"])
    try:
        bus.open()
        bus.torque_off()
        print(f"[OK] torque OFF sent to IDs 0-15 on {port} @ {baudrate}")
        return 0
    except Exception as e:
        print(f"[ERROR] direct torque off failed: {e}")
        print("        Cut the right hand power.")
        return 1
    finally:
        bus.close()


if __name__ == "__main__":
    raise SystemExit(main())
