"""Check Pitch stability while slowly moving between fixed angles."""

import argparse
import csv
import json
import math
import struct
import time

from run_sine import OpenOCD, ROOT


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name", required=True)
    parser.add_argument("--angles", type=float, nargs="+", default=[-25, -15, 0, 15, 25, 0])
    parser.add_argument("--hold", type=float, default=6)
    parser.add_argument("--trace", action="store_true", help="Record arrival at the RobotTask rate")
    args = parser.parse_args()
    if any(abs(angle) > 25 for angle in args.angles) or not 2 < args.hold <= 10:
        parser.error("Use Pitch angles within +/-25 degrees and hold time (2,10] seconds")

    ocd = OpenOCD()
    rc = struct.unpack("<5h2B", ocd.read(ocd.symbols["rc_ctrl"], 12))
    chassis = ocd.read(ocd.symbols["chassis_cmd_recv"], 21)
    if any(rc[:5]) or rc[6] != 1 or chassis[16] not in (0, 2):
        raise RuntimeError("Keep sticks/dial neutral and right switch UP")
    ocd.word(ocd.symbols["gimbal_sine_test"], 0)
    pointer = struct.unpack("<I", ocd.read(ocd.symbols["pitch_motor"], 4))[0]
    if not 0x20000000 <= pointer < 0x20020000:
        raise RuntimeError("Check that firmware matches ELF and initialization is complete")
    controller = ocd.read(pointer, 440)
    gains = {key: struct.unpack_from("<f", controller, offset)[0]
             for key, offset in (("angle_kp", 288), ("angle_ki", 292), ("angle_kd", 296),
                                 ("speed_kp", 168), ("speed_ki", 172), ("speed_kd", 176),
                                 ("speed_integral_limit", 192))}
    command = ocd.symbols["gimbal_cmd_send"] + 4
    ref = struct.unpack("<f", ocd.read(command, 4))[0]
    rows = []
    directory = ROOT / "build/gimbal_pid"
    started = time.monotonic()
    aborted = None
    for angle in args.angles:
        arrived = None
        tracing = False
        previous = time.monotonic()
        while arrived is None or time.monotonic() - arrived < args.hold:
            now = time.monotonic()
            step = 15 * (now - previous)
            previous = now
            ref += max(-step, min(step, angle - ref))
            ocd.float(command, ref)
            if args.trace and not tracing and abs(angle-ref) <= 3:
                ocd.word(ocd.symbols["gimbal_sine_sample_count"], 0)
                ocd.word(ocd.symbols["gimbal_trace_period_ms"], 5)
                ocd.word(ocd.symbols["gimbal_trace_active"], 1)
                tracing = True
            if ref == angle and arrived is None:
                arrived = now
            raw = ocd.call(
                f"join [list [read_memory {pointer+212} 32 14] "
                f"[read_memory {pointer+332} 32 13] "
                f"[read_memory {ocd.symbols['rc_ctrl']} 32 3]] {{;}}"
            )
            blocks = [struct.pack("<" + "I" * len(values), *values)
                      for group in raw.split(";")
                      for values in [[int(value, 0) for value in group.split()]]]
            row = {"time_s": now-started, "pose_deg": angle,
                   "hold_s": now-arrived if arrived is not None else -1}
            for key, offset in (("speed", 0), ("speed_error", 8), ("integral", 24), ("output", 36)):
                row[key] = struct.unpack_from("<f", blocks[0], offset)[0]
            for key, offset in (("measure", 0), ("error", 8), ("ref", 48)):
                row[key] = struct.unpack_from("<f", blocks[1], offset)[0]
            rows.append(row)
            rc = struct.unpack("<5h2B", blocks[2])
            if any(rc[:5]) or rc[6] != 1:
                aborted = "Remote input changed"
                break
            if abs(row["error"]) > 10 or abs(row["output"]) >= 19600:
                ocd.float(command, row["measure"])
                aborted = "Tracking error or output exceeded test envelope"
                break
        if tracing:
            ocd.word(ocd.symbols["gimbal_trace_active"], 0)
            count = struct.unpack("<I", ocd.read(ocd.symbols["gimbal_sine_sample_count"], 4))[0]
            recorded = ocd.read(ocd.symbols["gimbal_sine_samples"], count*36)
            fields = ("time_s", "yaw_ref", "yaw_measure", "yaw_error", "yaw_output",
                      "pitch_ref", "pitch_measure", "pitch_error", "pitch_output")
            with (directory / f"{args.name}_{len(rows)}_{angle:g}_trace.csv").open("w", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(fields)
                writer.writerows(struct.iter_unpack("<9f", recorded))
            ocd.word(ocd.symbols["gimbal_trace_period_ms"], 50)
        if aborted:
            break
    ocd.socket.close()
    with (directory / f"{args.name}.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0])
        writer.writeheader()
        writer.writerows(rows)
    result = {"aborted": aborted, "gains": gains, "poses": []}
    for angle in dict.fromkeys(args.angles):
        selected = [row for row in rows if row["pose_deg"] == angle and row["hold_s"] >= 2]
        if not selected:
            continue
        result["poses"].append({
            "pose_deg": angle, "samples": len(selected),
            "rms_error_deg": math.sqrt(sum(row["error"]**2 for row in selected)/len(selected)),
            "peak_error_deg": max(abs(row["error"]) for row in selected),
            "angle_span_deg": max(row["measure"] for row in selected)-min(row["measure"] for row in selected),
            "rms_speed_deg_s": math.sqrt(sum(row["speed"]**2 for row in selected)/len(selected)),
            "peak_speed_deg_s": max(abs(row["speed"]) for row in selected),
            "arrival_peak_error_deg": max(abs(row["error"]) for row in rows
                                          if row["pose_deg"] == angle and 0 <= row["hold_s"] < 2),
        })
    result["settled_passed"] = bool(not aborted and len(result["poses"]) == len(set(args.angles)) and
                            all(pose["peak_error_deg"] <= 0.2 and pose["angle_span_deg"] <= 0.2 and
                                pose["rms_speed_deg_s"] <= 1 and pose["peak_speed_deg_s"] <= 5
                                for pose in result["poses"]))
    (directory / f"{args.name}.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2), flush=True)


if __name__ == "__main__":
    main()
