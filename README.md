# AUTOSAR Classic 快速原型（Linux + vcan）

这个示例演示如何在 Linux 上使用 vcan (SocketCAN) + cantools + python-can 快速做一个 AUTOSAR Classic 风格的原型：
- 用 DBC 定义消息与信号
- 用 Python 将信号编码为 CAN 帧并发送到 vcan0
- 另一个 Python 进程接收并解码消息

依赖（在 Ubuntu/Debian 下）：
- python3, pip
- can-utils
- python 包：cantools, python-can

安装（示例）：
```bash
sudo apt update
sudo apt install -y python3 python3-venv python3-pip can-utils
python3 -m venv venv
. venv/bin/activate
pip install --upgrade pip
pip install cantools python-can
```

启动 vcan 接口：
```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
# 检查
ip -details link show vcan0
```

运行示例：
- 打开一个终端，启动接收端：
  ```bash
  . venv/bin/activate
  python3 receiver.py
  ```
- 打开另一个终端，发送几次消息：
  ```bash
  . venv/bin/activate
  python3 sender.py
  ```

你会在接收端看到解码后的信号值。这个最小化原型可用于验证信号定义、打包/解析及定时逻辑。下一步可选：用 `python-autosar` 解析 ARXML 并自动生成 DBC / stub 信息，或把发送端的逻辑用 C 移植到 ERIKA/RTOS 上。
