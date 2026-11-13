#!/usr/bin/env python3
"""
receiver.py
- 在 vcan0 上监听 CAN 帧，使用 cantools 解码并打印信号
"""
import cantools
import can

DB_PATH = "dbc/example.dbc"
INTERFACE = "socketcan"
CHANNEL = "vcan0"

def main():
    db = cantools.database.load_file(DB_PATH)
    # 建立 SocketCAN 总线
    bus = can.Bus(interface=INTERFACE, channel=CHANNEL, bitrate=500000)

    print("Listening on vcan0... (Ctrl-C to exit)")
    try:
        while True:
            msg = bus.recv(timeout=1.0)
            if msg is None:
                continue
            try:
                decoded = db.decode_message(msg.arbitration_id, msg.data)
                print(f"Recv id=0x{msg.arbitration_id:X} raw={msg.data.hex()} decoded={decoded}")
            except Exception:
                # 不是我们 DBC 中的消息，忽略或打印
                print(f"Recv unknown id=0x{msg.arbitration_id:X} raw={msg.data.hex()}")
    except KeyboardInterrupt:
        print("Receiver terminated")
    finally:
        bus.shutdown()

if __name__ == "__main__":
    main()
