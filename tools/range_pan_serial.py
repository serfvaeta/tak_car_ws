"""Read range, servo, and avoidance diagnostics through H60 USB-COM."""

import argparse
import struct
import sys
import time


BAUDRATE = 230400
STATUS_NAMES = {3: "READY", 4: "TIMEOUT", 5: "OUT_OF_RANGE"}
AVOID_PHASES = ("ALIGN_TARGET", "FORWARD", "SCAN_SIDE", "TURN_OUT",
                "ADVANCE", "HOLD", "RESCAN_WAIT")
AVOID_REASONS = ("NONE", "AOA_STALE", "FRONT_STALE", "WAIT_LEFT",
                 "WAIT_RIGHT", "NO_SIDE", "TURN_CENTER", "TURN_TIMEOUT",
                 "TURN_BLOCKED", "ADVANCE_STALL", "DISTANCE_LIMIT",
                 "TIME_LIMIT", "REJOIN_CLEAR", "REJOIN_TIMEOUT",
                 "REJOIN_TURN_LIMIT", "LOOP_LATE", "CHASSIS", "BAD_CONFIG",
                 "FOLLOW_STOP", "FRONT_CLOSE", "HOLD_UNKNOWN", "WAIT_RESCAN")
AVOID_REASON_HINTS = (
    "正常", "AOA 无新数据", "正前方测距无效或超时", "等待左侧测距",
    "等待右侧测距", "左右都没有可走通路", "等待舵机回中并取得前方测距",
    "转向超时", "转向后前方距离不足", "前进位移不足",
    "绕行距离达到上限", "绕行时间达到上限", "等待两次清晰前向测距",
    "重新对准目标超时", "重新对准目标转角过大", "控制循环间隔过长",
    "底盘不是四轮差速", "避障配置无效", "跟随目标要求停车",
    "前方距离不足", "未知锁定原因", "等待下一轮扫描",
)
AVOID2_PHASES = ("WAIT_TARGET", "COARSE_SCAN", "FINE_SCAN", "TURN",
                 "VERIFY_FRONT", "MOVE", "WAIT_RESCAN")
AVOID2_REASONS = ("NONE", "AOA_INVALID", "TARGET_NEAR", "COARSE_TIMEOUT",
                  "NO_COARSE_CANDIDATE", "FINE_TIMEOUT", "NO_SAFE_FINE",
                  "TURN_TIMEOUT", "FRONT_EMERGENCY", "VERIFY_TIMEOUT",
                  "VERIFY_BLOCKED", "FRONT_INVALID", "MOVE_STALL",
                  "BAD_CONFIG", "CHASSIS", "INVALID_STATE")


def build_pan_frame(angle_degrees: int) -> bytes:
    if not -90 <= angle_degrees <= 90:
        raise ValueError("舵机角度必须在 -90 到 90 度之间")
    frame = b"\xAA\x55\x07\x61" + struct.pack(">h", angle_degrees)
    return frame + bytes((sum(frame) & 0xFF,))


def extract_frames(buffer: bytearray):
    """Remove complete, checksum-valid X-Protocol frames from a byte buffer."""
    while len(buffer) >= 5:
        if buffer[:2] != b"\xAA\x55":
            del buffer[0]
            continue
        length = buffer[2]
        if not 5 <= length <= 40:
            del buffer[0]
            continue
        if len(buffer) < length:
            return
        frame = bytes(buffer[:length])
        if (sum(frame[:-1]) & 0xFF) != frame[-1]:
            del buffer[0]
            continue
        del buffer[:length]
        yield frame[3], frame[4:-1]


def describe_frame(frame_id: int, payload: bytes) -> str | None:
    if frame_id == 0x17 and len(payload) == 30:
        (phase, reason, flags, pan, azimuth, aoa_cm, range_angle,
         status, range_mm, target_mm_s, actual_mm_s, target_yaw,
         actual_yaw, elapsed_ms, progress, target, scan_index,
         candidate_index, candidate_count, fine_index) = struct.unpack(
             ">BBBbhHbBHhhhhHhHBBBB", payload)
        phase_name = AVOID2_PHASES[phase] if phase < len(AVOID2_PHASES) else f"UNKNOWN({phase})"
        reason_name = AVOID2_REASONS[reason] if reason < len(AVOID2_REASONS) else f"UNKNOWN({reason})"
        aoa_text = f"{azimuth}°/{aoa_cm}cm" if flags & 1 else "无效"
        range_text = (f"{range_angle}°/{range_mm}mm/{STATUS_NAMES.get(status, status)}"
                      if flags & 2 else "无效")
        progress_unit = "mrad" if phase == 3 else "mm" if phase == 5 else ""
        return (f"AVOID2 {phase_name} 原因={reason_name} 舵机={pan}° "
                f"AOA={aoa_text} 超声波={range_text} "
                f"目标={target_mm_s}mm/s,{target_yaw}mrad/s "
                f"实际={actual_mm_s}mm/s,{actual_yaw}mrad/s "
                f"阶段={elapsed_ms}ms 进度={progress}/{target}{progress_unit} "
                f"扫描={scan_index} 候选={candidate_index}/{candidate_count} "
                f"细扫={fine_index}")
    if frame_id == 0x16 and len(payload) == 26:
        (phase, reason, flags, pan, azimuth, aoa_cm, range_angle,
         status, range_mm, target_mm_s, actual_mm_s, target_yaw,
         actual_yaw, elapsed_ms, progress, total_mm) = struct.unpack(
             ">BBBbhHbBHhhhhHHH", payload)
        phase_name = AVOID_PHASES[phase] if phase < len(AVOID_PHASES) else f"UNKNOWN({phase})"
        reason_name = AVOID_REASONS[reason] if reason < len(AVOID_REASONS) else f"UNKNOWN({reason})"
        reason_hint = AVOID_REASON_HINTS[reason] if reason < len(AVOID_REASON_HINTS) else "未知原因"
        aoa_text = f"{azimuth}°/{aoa_cm}cm" if flags & 1 else "无效"
        range_text = (f"{range_angle}°/{range_mm}mm/{STATUS_NAMES.get(status, status)}"
                      if flags & 2 else "无效")
        progress_unit = "mrad" if phase == 3 else "mm"
        return (f"AVOID {phase_name} 原因={reason_name}({reason_hint}) "
                f"底盘={'FWD' if flags & 4 else 'MEC'} "
                f"舵机={pan}° AOA={aoa_text} 超声波={range_text} "
                f"目标={target_mm_s}mm/s,{target_yaw}mrad/s "
                f"实际={actual_mm_s}mm/s,{actual_yaw}mrad/s "
                f"阶段={elapsed_ms}ms 进度={progress}{progress_unit} "
                f"总前进={total_mm}mm")
    if frame_id == 0x15 and len(payload) == 5:
        angle = struct.unpack(">h", payload[:2])[0]
        status, distance_mm = payload[2], int.from_bytes(payload[3:], "big")
        if status == 3:
            return f"SCAN {angle} 度: {distance_mm} mm"
        return f"SCAN {angle} 度: {STATUS_NAMES.get(status, f'UNKNOWN({status})')}"
    if frame_id == 0x11 and len(payload) == 3:
        status, distance_mm = payload[0], int.from_bytes(payload[1:], "big")
        if status == 3:
            return f"XURSC 距离: {distance_mm} mm"
        return f"XURSC 状态: {STATUS_NAMES.get(status, f'UNKNOWN({status})')}"
    if frame_id == 0x12 and len(payload) == 2:
        angle = struct.unpack(">h", payload)[0]
        return f"SG90 已执行: {angle} 度"
    return None


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="USB-COM 串口，例如 /dev/ttyACM0")
    parser.add_argument("--angle", type=int, help="发送 SG90 角度，-90 到 90 度")
    parser.add_argument("--seconds", type=float, default=8.0, help="读取时长，默认 8 秒")
    args = parser.parse_args(argv)
    if args.angle is not None and not -90 <= args.angle <= 90:
        parser.error("--angle 必须在 -90 到 90 度之间")
    if args.seconds <= 0:
        parser.error("--seconds 必须大于 0")

    try:
        import serial
    except ImportError:
        print("缺少 pyserial；请安装 python3-serial", file=sys.stderr)
        return 1

    port = serial.Serial(
        port=None, baudrate=BAUDRATE, timeout=0.1, write_timeout=0.2,
        exclusive=True, rtscts=False, dsrdtr=False,
    )
    port.dtr = False
    port.rts = False
    port.port = args.port
    try:
        port.open()
        buffer = bytearray()
        deadline = time.monotonic() + args.seconds
        angle_sent = args.angle is None
        telemetry_seen = False
        range_seen = False
        avoid_seen = False
        avoid2_seen = False
        angle_ack_seen = args.angle is None
        while time.monotonic() < deadline:
            buffer.extend(port.read(128))
            for frame_id, payload in extract_frames(buffer):
                if frame_id == 0x10 and len(payload) == 20:
                    telemetry_seen = True
                    if not angle_sent:
                        port.write(build_pan_frame(args.angle))
                        angle_sent = True
                        print(f"已发送 SG90 指令: {args.angle} 度")
                if frame_id == 0x11 and len(payload) == 3:
                    range_seen = True
                if frame_id == 0x16 and len(payload) == 26:
                    avoid_seen = True
                if frame_id == 0x17 and len(payload) == 30:
                    avoid2_seen = True
                if angle_sent and frame_id == 0x12 and len(payload) == 2:
                    angle_ack_seen = struct.unpack(">h", payload)[0] == args.angle
                description = describe_frame(frame_id, payload)
                if description is not None:
                    print(description)
        if not telemetry_seen:
            print("没有收到有效底盘遥测；请检查 USB-COM 端口和供电", file=sys.stderr)
            return 1
        if not angle_sent:
            print("未能确认串口通信，舵机指令未发送", file=sys.stderr)
            return 1
        if not range_seen and not avoid_seen and not avoid2_seen:
            print("未收到 XURSC 或避障诊断帧；请确认模式与固件", file=sys.stderr)
            return 1
        if not angle_ack_seen:
            print("未收到 SG90 角度回执；请检查固件和 PB8 舵机连接", file=sys.stderr)
            return 1
        return 0
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f"串口读取失败：{exc}", file=sys.stderr)
        return 1
    finally:
        if port.is_open:
            port.close()


if __name__ == "__main__":
    raise SystemExit(main())
