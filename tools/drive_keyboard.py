"""Hold arrow keys to drive an OpenCTR H60 through its USB-COM serial port.

Linux input events include both press and release, unlike terminal escape codes.
The controller has no command-loss watchdog: test with wheels raised and keep
its power switch within reach. A disconnected serial cable cannot be stopped
by this script.
"""

import argparse
import glob
import os
import select
import struct
import sys
import termios
import time
import tty

from drive_serial import BAUDRATE, build_velocity_frame, wait_for_telemetry


EVENT = struct.Struct("@llHHi")  # Linux struct input_event on this 64-bit host
KEY_CODES = {103: "up", 108: "down", 105: "left", 106: "right", 57: "stop", 16: "quit"}
INTERVAL = 0.05


def decode_input_events(pending: bytes, chunk: bytes):
    """Decode complete Linux input events, retaining an incomplete tail."""
    data = pending + chunk
    complete = len(data) // EVENT.size * EVENT.size
    events = []
    for offset in range(0, complete, EVENT.size):
        _, _, event_type, code, value = EVENT.unpack_from(data, offset)
        if event_type == 1 and code in KEY_CODES and value in (0, 1, 2):
            events.append((KEY_CODES[code], value))
    return events, data[complete:]


class DriveState:
    def __init__(self, speed_mm_s: int, turn_mrad_s: int):
        self.speed_mm_s = speed_mm_s
        self.turn_mrad_s = turn_mrad_s
        self.pressed = set()

    def accept(self, key: str, value: int) -> None:
        if value == 2:  # Kernel key repeat does not change the held state.
            return
        if key == "stop" and value == 1:
            self.pressed.clear()
        elif key in ("up", "down", "left", "right"):
            if value == 1:
                self.pressed.add(key)
            elif value == 0:
                self.pressed.discard(key)

    def target(self):
        vx = self.speed_mm_s * (int("up" in self.pressed) - int("down" in self.pressed))
        yaw = self.turn_mrad_s * (int("left" in self.pressed) - int("right" in self.pressed))
        return vx, yaw


def run_drive_loop(read_events, write, state: DriveState, on_target=None) -> None:
    """Transmit every 50 ms; a key-up makes the next frame zero immediately."""
    stop = build_velocity_frame(0)
    stop_error = None
    last_target = None
    try:
        write(stop)
        while True:
            events = read_events(INTERVAL)
            if any(key == "quit" and value == 1 for key, value in events):
                break
            for key, value in events:
                state.accept(key, value)
            target = state.target()
            if on_target is not None and target != last_target:
                on_target(target)
            last_target = target
            write(build_velocity_frame(target[0], yaw_mrad_s=target[1]))
    finally:
        for _ in range(3):
            try:
                write(stop)
            except Exception as exc:
                stop_error = exc
        if stop_error is not None:
            raise RuntimeError("停车帧未能可靠发送；请关闭小车电源") from stop_error


def keyboard_paths(explicit_paths):
    if explicit_paths:
        return explicit_paths
    candidates = glob.glob("/dev/input/by-id/*-event-kbd")
    candidates += glob.glob("/dev/input/by-path/*-event-kbd")
    # by-id and by-path can point to the same device.
    return list(dict((os.path.realpath(path), path) for path in candidates).values())


class KeyboardInput:
    def __init__(self, paths):
        self.paths = paths
        self.fds = []
        self.pending = {}
        self.tty_fd = None
        self.tty_settings = None

    def __enter__(self):
        if not sys.stdin.isatty():
            raise RuntimeError("请在本机交互式终端运行，以便按 Ctrl+C 退出")
        errors = []
        try:
            for path in self.paths:
                try:
                    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
                except OSError as exc:
                    errors.append(f"{path}: {exc.strerror}")
                else:
                    self.fds.append(fd)
                    self.pending[fd] = b""
            if not self.fds:
                detail = "；".join(errors) if errors else "未发现 /dev/input/*-event-kbd"
                raise RuntimeError(
                    f"无法读取键盘按下/松开事件：{detail}。可用 --keyboard-device 指定键盘设备；"
                    "若权限不足，请为当前用户配置该设备的读取权限"
                )
            self.tty_fd = sys.stdin.fileno()
            self.tty_settings = termios.tcgetattr(self.tty_fd)
            tty.setcbreak(self.tty_fd)
            return self
        except BaseException:
            self.__exit__(None, None, None)
            raise

    def __exit__(self, _type, _value, _traceback):
        if self.tty_settings is not None:
            termios.tcsetattr(self.tty_fd, termios.TCSADRAIN, self.tty_settings)
            self.tty_settings = None
        for fd in self.fds:
            os.close(fd)
        self.fds.clear()

    def read_events(self, timeout: float):
        ready, _, _ = select.select(self.fds + [self.tty_fd], [], [], timeout)
        if self.tty_fd in ready:
            # Terminal receives the same keys; discard its escape codes.
            os.read(self.tty_fd, 1024)
        events = []
        for fd in self.fds:
            if fd not in ready:
                continue
            try:
                chunk = os.read(fd, EVENT.size * 64)
            except BlockingIOError:
                continue
            if not chunk:
                raise RuntimeError("键盘输入设备已断开；请关闭小车电源")
            decoded, self.pending[fd] = decode_input_events(self.pending[fd], chunk)
            events.extend(decoded)
        return events


def describe_target(target):
    vx, yaw = target
    parts = []
    if vx:
        parts.append("前进" if vx > 0 else "后退")
    if yaw:
        parts.append("左转" if yaw > 0 else "右转")
    return "+".join(parts) if parts else "停止"


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0", help="USB-COM 串口，默认 /dev/ttyACM0")
    parser.add_argument("--speed", type=int, default=100, help="前后速度 mm/s，默认 100")
    parser.add_argument("--turn", type=int, default=300, help="左右角速度 mrad/s，默认 300")
    parser.add_argument("--keyboard-device", action="append", help="Linux 键盘 event 设备，可重复指定")
    parser.add_argument("--dry-run", action="store_true", help="读取按键并显示状态，不打开串口")
    args = parser.parse_args(argv)
    if not 20 <= args.speed <= 200:
        parser.error("--speed 只能在 20 到 200 mm/s 之间")
    if not 100 <= args.turn <= 600:
        parser.error("--turn 只能在 100 到 600 mrad/s 之间")

    port = None
    try:
        with KeyboardInput(keyboard_paths(args.keyboard_device)) as keyboard:
            print("键盘设备:", ", ".join(keyboard.paths))
            if not args.dry_run:
                try:
                    import serial
                except ImportError as exc:
                    raise RuntimeError("缺少 pyserial；请安装 python3-serial 或 pip install pyserial") from exc
                port = serial.Serial(
                    port=None, baudrate=BAUDRATE, timeout=0.1, write_timeout=0.2,
                    exclusive=True, rtscts=False, dsrdtr=False,
                )
                port.dtr = False
                port.rts = False
                port.port = args.port
                port.open()
                if not wait_for_telemetry(port.read, time.monotonic):
                    raise RuntimeError("未收到有效小车遥测；请确认使用 USB-COM 而非 USB-DEBUG")

            def write(frame):
                if port is not None and port.write(frame) != len(frame):
                    raise RuntimeError("串口发送不完整；请关闭小车电源")

            def read_events(timeout):
                events = keyboard.read_events(timeout)
                if port is not None and port.in_waiting:
                    port.read(port.in_waiting)  # Keep incoming telemetry from accumulating.
                return events

            print("按住方向键移动；松开即发送零速度。空格停车，Q 退出，Ctrl+C 中断。")
            run_drive_loop(
                read_events, write, DriveState(args.speed, args.turn),
                lambda target: print("状态:", describe_target(target)),
            )
        return 0
    except KeyboardInterrupt:
        print("已中断；程序已尝试发送停车指令", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"键盘控制失败：{exc}", file=sys.stderr)
        return 1
    finally:
        if port is not None and port.is_open:
            port.close()


if __name__ == "__main__":
    raise SystemExit(main())
