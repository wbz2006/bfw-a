"""Run the retained gimbal sine test through the existing OpenOCD TCL port."""

import argparse
import csv
import json
import math
from pathlib import Path
import socket
import struct
import subprocess
import time


ROOT = Path(__file__).resolve().parents[2]
ELF = ROOT / "build/basic_framework_ATYPE.elf"
AXES = {"yaw": 1, "pitch": 2, "both": 3}


class OpenOCD:
    def __init__(self, port=50001):
        self.socket = socket.create_connection(("127.0.0.1", port), timeout=3)
        self.socket.settimeout(45)
        output = subprocess.check_output(["arm-none-eabi-nm", "-an", str(ELF)], text=True)
        self.symbols = {
            fields[2]: int(fields[0], 16)
            for line in output.splitlines()
            if len(fields := line.split()) == 3
        }

    def call(self, command):
        self.socket.sendall(
            ("if {[catch {" + command + "} result]} {return \"ERROR:$result\"}; return $result").encode()
            + b"\x1a"
        )
        data = b""
        while not data.endswith(b"\x1a"):
            chunk = self.socket.recv(65536)
            if not chunk:
                raise ConnectionError("OpenOCD closed the connection")
            data += chunk
        result = data[:-1].decode()
        if result.startswith("ERROR:"):
            raise RuntimeError(result)
        return result

    def read(self, address, size):
        if address % 4 == 0 and size % 4 == 0:
            values = self.call(f"read_memory {address} 32 {size // 4}").split()
            return struct.pack("<" + "I" * len(values), *(int(v, 0) for v in values))
        return bytes(int(v, 0) for v in self.call(f"read_memory {address} 8 {size}").split())

    def word(self, address, value):
        self.call(f"write_memory {address} 32 {{{value}}}")

    def float(self, address, value):
        self.word(address, struct.unpack("<I", struct.pack("<f", value))[0])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--axis", choices=AXES, required=True)
    parser.add_argument("--name", required=True)
    parser.add_argument("--amplitude", type=float, default=60.0)
    parser.add_argument("--frequency", type=float, default=0.25)
    parser.add_argument("--duration", type=float, default=52.0)
    parser.add_argument("--warmup", type=float, default=8.0)
    parser.add_argument("--record-period", type=int, default=50, help="Board sample interval in milliseconds")
    parser.add_argument("--angle-kp", type=float)
    parser.add_argument("--angle-ki", type=float)
    parser.add_argument("--angle-kd", type=float)
    parser.add_argument("--angle-deadband", type=float)
    parser.add_argument("--speed-kp", type=float)
    parser.add_argument("--speed-ki", type=float)
    parser.add_argument("--speed-kd", type=float)
    parser.add_argument("--integral-limit", type=float)
    args = parser.parse_args()
    if not (0 < args.amplitude <= 60 and 0 < args.frequency <= 1 and
            0 <= args.warmup < args.duration <= 60):
        parser.error("Use amplitude (0,60], frequency (0,1], and warmup < duration <= 60")
    if args.record_period < 5 or math.ceil(args.duration*1000/args.record_period) > 1280:
        parser.error("Record period must be >=5ms and fit the 1280-sample board buffer")

    ocd = OpenOCD()
    sine = ocd.symbols["gimbal_sine_test"]
    imu = struct.unpack("<I", ocd.read(ocd.symbols["gimbal_IMU_data"], 4))[0]
    pointers = {
        axis: struct.unpack("<I", ocd.read(ocd.symbols[f"{axis}_motor"], 4))[0]
        for axis in ("yaw", "pitch")
    }
    if not all(0x20000000 <= pointer < 0x20020000 for pointer in pointers.values()):
        raise RuntimeError("Motor pointers are invalid; check that firmware matches ELF")
    rc = struct.unpack("<5h2B", ocd.read(ocd.symbols["rc_ctrl"], 12))
    chassis = ocd.read(ocd.symbols["chassis_cmd_recv"], 21)
    if any(rc[:5]) or rc[6] != 1 or chassis[16] not in (0, 2):
        raise RuntimeError("Keep both sticks/dial neutral and right switch UP for sine tuning")
    targets = ("yaw", "pitch") if args.axis == "both" else (args.axis,)
    previous = {axis: ocd.read(pointer, 440) for axis, pointer in pointers.items()}

    ocd.word(sine, 0)
    ocd.word(ocd.symbols["gimbal_trace_active"], 0)
    ocd.word(ocd.symbols["gimbal_trace_period_ms"], args.record_period)
    time.sleep(0.1)
    for axis in targets:
        pointer = pointers[axis]
        for base, key, offset in (
            (288, "angle_kp", 0), (288, "angle_ki", 4), (288, "angle_kd", 8),
            (288, "angle_deadband", 16),
            (168, "speed_kp", 0), (168, "speed_ki", 4), (168, "speed_kd", 8),
            (168, "integral_limit", 24),
        ):
            value = getattr(args, key)
            if value is not None:
                ocd.float(pointer + base + offset, value)
        ocd.float(pointer + 356, 0.0)
        ocd.float(pointer + 348, 0.0)
    for offset, value in ((4, args.amplitude), (8, args.frequency), (12, args.duration)):
        ocd.float(sine + offset, value)

    rows = []
    started = time.monotonic()
    aborted = None
    ocd.float(sine + 16, 0.0)
    ocd.word(sine, AXES[args.axis])
    try:
        while time.monotonic() - started < args.duration + 5:
            expressions = [f"[read_memory {sine} 32 7]"]
            expressions.extend(f"[read_memory {p} 32 110]" for p in pointers.values())
            expressions.append(f"[read_memory {ocd.symbols['rc_ctrl']} 32 3]")
            expressions.append(f"[read_memory {imu} 32 10]")
            raw = ocd.call("join [list " + " ".join(expressions) + "] {;}")
            blocks = [struct.pack("<" + "I" * len(values), *values)
                      for group in raw.split(";")
                      for values in [[int(value, 0) for value in group.split()]]]
            axes, amplitude, frequency, duration, elapsed, yaw_center, pitch_center = struct.unpack("<I6f", blocks[0])
            row = {"time_s": elapsed, "axes": axes, "yaw_center": yaw_center, "pitch_center": pitch_center}
            for key, offset in (("gyro_x", 0), ("gyro_y", 4), ("gyro_z", 8), ("imu_pitch", 28)):
                row[key] = struct.unpack_from("<f", blocks[4], offset)[0]
            for axis, block in zip(("yaw", "pitch"), blocks[1:3]):
                for key, offset in (("ref", 380), ("measure", 332), ("error", 340),
                                    ("speed_ref", 260), ("speed_measure", 212),
                                    ("output", 248), ("max_output", 180),
                                    ("angle_kp", 288), ("angle_ki", 292), ("angle_kd", 296),
                                    ("angle_deadband", 304),
                                    ("speed_kp", 168), ("speed_ki", 172), ("speed_kd", 176),
                                    ("integral", 236), ("integral_limit", 192)):
                    row[f"{axis}_{key}"] = struct.unpack_from("<f", block, offset)[0]
            rows.append(row)
            rc = struct.unpack("<5h2B", blocks[3])
            if any(rc[:5]) or rc[6] != 1:
                aborted = "Remote input changed during test"
                break
            if any(abs(row[f"{axis}_error"]) > max(15.0, args.amplitude * 0.5) or
                   abs(row[f"{axis}_output"]) >= row[f"{axis}_max_output"] * 0.98
                   for axis in ("yaw", "pitch")):
                aborted = "Tracking error exceeded motion envelope or speed output reached 98% of limit"
                # 关闭内环输出, 防止失败的试验在退出后继续驱动电机。
                for pointer in pointers.values():
                    ocd.float(pointer + 180, 0.0)
                break
            if elapsed >= args.duration - 0.01:
                break
            time.sleep(0.01)
    finally:
        ocd.word(sine, 0)
        ocd.word(ocd.symbols["gimbal_trace_period_ms"], 50)
        if aborted:
            for axis, pointer in pointers.items():
                for base in (168, 288):
                    for offset in (0, 4, 8, 16, 24):
                        ocd.float(pointer + base + offset,
                                  struct.unpack_from("<f", previous[axis], base + offset)[0])
                ocd.float(pointer + 356, 0.0)
                ocd.float(pointer + 236, 0.0)
                ocd.float(pointer + 180, struct.unpack_from("<f", previous[axis], 180)[0])
        count = struct.unpack("<I", ocd.read(ocd.symbols["gimbal_sine_sample_count"], 4))[0]
        if not 0 < count <= 1280:
            raise RuntimeError("No valid board samples; sine start was rejected or interrupted")
        recorded = ocd.read(ocd.symbols["gimbal_sine_samples"], count * 36)
        ocd.socket.close()

    directory = ROOT / "build/gimbal_pid"
    directory.mkdir(exist_ok=True)
    with (directory / f"{args.name}_live.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0])
        writer.writeheader()
        writer.writerows(rows)
    constant_keys = ["yaw_center", "pitch_center"]
    for axis in ("yaw", "pitch"):
        constant_keys.extend(f"{axis}_{key}" for key in (
            "max_output", "angle_kp", "angle_ki", "angle_kd", "angle_deadband",
            "speed_kp", "speed_ki", "speed_kd", "integral_limit"))
    metadata = {key: rows[-1][key] for key in constant_keys}
    fields = ("time_s", "yaw_ref", "yaw_measure", "yaw_error", "yaw_output",
              "pitch_ref", "pitch_measure", "pitch_error", "pitch_output")
    rows = [dict(metadata, **dict(zip(fields, values)))
            for values in struct.iter_unpack("<9f", recorded)]
    with (directory / f"{args.name}.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0])
        writer.writeheader()
        writer.writerows(rows)
    analysis_end = args.duration - 1.0 / args.frequency
    selected = [row for row in rows if args.warmup <= row["time_s"] <= analysis_end]
    result = {"axis": args.axis, "amplitude_deg": args.amplitude,
              "frequency_hz": args.frequency, "duration_s": args.duration,
              "warmup_s": args.warmup, "analysis_end_s": analysis_end,
              "record_period_ms": args.record_period,
              "samples": len(selected), "aborted": aborted,
              "last_time_s": rows[-1]["time_s"], "metrics": {}}
    for axis in ("yaw", "pitch"):
        errors = [row[f"{axis}_error"] for row in selected]
        refs = [row[f"{axis}_ref"] for row in selected]
        metrics = {
            "rms_error_deg": math.sqrt(sum(v*v for v in errors) / len(errors)) if errors else None,
            "peak_error_deg": max(map(abs, errors)) if errors else None,
            "reference_span_deg": max(refs)-min(refs) if refs else 0,
            "peak_output": max(abs(row[f"{axis}_output"]) for row in selected) if selected else None,
        }
        for key in ("angle_kp", "angle_ki", "angle_kd", "angle_deadband", "speed_kp", "speed_ki", "speed_kd", "integral_limit"):
            metrics[key] = rows[-1][f"{axis}_{key}"]
        metrics["passed"] = bool(selected and not aborted and
            rows[-1]["time_s"] >= args.duration - 0.15 and
            metrics["rms_error_deg"] <= 0.5 and metrics["peak_error_deg"] <= 1.0 and
            metrics["peak_output"] < rows[-1][f"{axis}_max_output"] * 0.98 and
            (axis not in targets or metrics["reference_span_deg"] >= args.amplitude * 1.9))
        result["metrics"][axis] = metrics
    result["passed"] = all(result["metrics"][axis]["passed"] for axis in targets)
    (directory / f"{args.name}.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2), flush=True)


if __name__ == "__main__":
    main()
