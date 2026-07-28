#!/usr/bin/env python3
"""
sender.py
- 使用 cantools 加载 DBC，编码信号
- 使用 python-can 将 CAN 帧发送到 vcan0
"""
import time
import cantools
import can

DB_PATH = "dbc/example.dbc"
MSG_NAME = "SignalMessage"
INTERFACE = "socketcan"
CHANNEL = "vcan0"

def main():
    db = cantools.database.load_file(DB_PATH)
    msg = db.get_message_by_name(MSG_NAME)
    if msg is None:
        raise SystemExit("找不到消息: " + MSG_NAME)

    bus = can.Bus(interface=INTERFACE, channel=CHANNEL, bitrate=500000)

    try:
        seq = 0
        while True:
            # 构造信号值（示例：改变转速与车速）
            engine_rpm = 800 + (seq % 5000)  # rpm
            vehicle_kph = 30 + ((seq // 10) % 120)  # km/h

            data = msg.encode({
                "EngineSpeed": engine_rpm,
                "VehicleSpeed": vehicle_kph
            })

            can_msg = can.Message(arbitration_id=msg.frame_id, data=data, is_extended_id=False)
            bus.send(can_msg)
            print(f"Sent id=0x{can_msg.arbitration_id:X} EngineSpeed={engine_rpm} VehicleSpeed={vehicle_kph}")
            seq += 1
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("Sender terminated")
    finally:
        bus.shutdown()

if __name__ == "__main__":
    main()
