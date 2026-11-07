#!/usr/bin/env python3
"""OD-0 stationary RPLIDAR obstacle-type characterization.

Uses ROS 2 /scan only. No Arduino, motor, or command topics.

Each placement produces N fresh scans. A detected scan must contain a plausible
return in a sector centered on the indicated target bearing. The nearest valid
return in that sector is used for range error; background clutter must be
kept clear for the measurement to be meaningful.
"""
import argparse
import csv
import math
import time
from collections import defaultdict
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


TYPES = [
    ("Cardboard box", "box"),
    ("Human legs", "legs"),
    ("Thin pole", "pole"),
    ("Dark/black object", "dark"),
    ("Reflective surface", "reflective"),
]
DISTANCES = [0.5, 1.0, 1.5]
ANGLES = [-30.0, 0.0, 30.0]
RAW_FIELDS = [
    "obstacle_type", "position", "distance_label_m", "ground_truth_m",
    "target_angle_deg", "target_width_m", "sector_half_width_deg",
    "scan_number", "measured_min_range_m", "detected", "abs_error_m",
]
SUM_FIELDS = [
    "Obstacle Type", "Detection Rate (%)", "Mean Abs. Range Error (m)",
    "Max. Range Error (m)", "Undetected Positions", "Partial Positions",
    "Detected Scans", "Total Scans", "Positions Tested",
]
POSITION_FIELDS = [
    "obstacle_type", "position", "ground_truth_m", "target_angle_deg",
    "detected_scans", "total_scans", "detection_rate_pct",
    "mean_abs_error_m", "max_abs_error_m",
]


def ask_float(question, default=None, minimum=0.0):
    while True:
        default_label = f" [{default}]" if default is not None else ""
        answer = input(f"{question}{default_label}: ").strip()
        if not answer and default is not None:
            value = float(default)
        else:
            try:
                value = float(answer)
            except ValueError:
                print("  Please enter a number (in meters).")
                continue
        if not math.isfinite(value) or value <= minimum:
            print(f"  Value must be greater than {minimum}.")
            continue
        return value


def wrapped_difference_rad(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def measure_scan(scan, target_angle_deg, half_width_deg, actual_m, tolerance_m):
    """Return (nearest valid ROI distance or None, association pass).

    Angle and distance gating is a predeclared target-association rule, NOT an
    object classifier. Empty/invalid rays (including Inf/NaN) are ignored.
    """
    bearing = math.radians(target_angle_deg)
    sector = math.radians(half_width_deg)
    candidate_ranges = []
    for index, measured in enumerate(scan.ranges):
        if not math.isfinite(measured):
            continue
        if not (scan.range_min <= measured <= scan.range_max):
            continue
        beam_angle = scan.angle_min + index * scan.angle_increment
        if abs(wrapped_difference_rad(beam_angle, bearing)) <= sector:
            candidate_ranges.append(float(measured))
    if not candidate_ranges:
        return None, False
    # The closest return is selected by a fixed rule, not by whichever value
    # happens to agree best with ground truth.
    measured = min(candidate_ranges)
    detected = abs(measured - actual_m) <= tolerance_m
    return measured, detected


class ScanReader(Node):
    def __init__(self):
        super().__init__("od0_lidar_characterization")
        self.queue = []
        self.subscription = self.create_subscription(
            LaserScan, "/scan", self.receive, qos_profile_sensor_data
        )

    def receive(self, scan):
        self.queue.append(scan)

    def collect(self, n, timeout_sec=25.0):
        # Drain a few old messages after the user finishes repositioning.
        self.queue.clear()
        deadline = time.monotonic() + timeout_sec
        for _ in range(3):
            if time.monotonic() > deadline:
                raise TimeoutError("LiDAR did not provide a fresh scan")
            rclpy.spin_once(self, timeout_sec=1.0)
            self.queue.clear()
        scans = []
        while len(scans) < n:
            if time.monotonic() > deadline:
                raise TimeoutError(
                    f"Received {len(scans)} of {n} scans. Check /scan."
                )
            if not self.queue:
                rclpy.spin_once(self, timeout_sec=1.0)
            if self.queue:
                scans.append(self.queue.pop(0))
        return scans


def save_reports(raw_rows, output_dir):
    groups = defaultdict(list)
    pos_groups = defaultdict(list)
    for row in raw_rows:
        groups[row["obstacle_type"]].append(row)
        pos_groups[(row["obstacle_type"], row["position"])].append(row)
    summary = []
    per_position = []
    for name, _ in TYPES:
        scans = groups[name]
        if not scans:
            continue
        hits = [r for r in scans if r["detected"] == 1]
        errors = [r["abs_error_m"] for r in hits]
        missed = []
        partial = []
        for (typ, pos), rows in pos_groups.items():
            if typ != name:
                continue
            hits_here = [r for r in rows if r["detected"] == 1]
            errs_here = [r["abs_error_m"] for r in hits_here]
            if not hits_here:
                missed.append(pos)
            elif len(hits_here) < len(rows):
                partial.append(f"{pos} ({len(hits_here)}/{len(rows)})")
            per_position.append({
                "obstacle_type": typ, "position": pos,
                "ground_truth_m": f"{rows[0]['ground_truth_m']:.3f}",
                "target_angle_deg": f"{rows[0]['target_angle_deg']:.1f}",
                "detected_scans": len(hits_here), "total_scans": len(rows),
                "detection_rate_pct": f"{100 * len(hits_here) / len(rows):.1f}",
                "mean_abs_error_m": f"{sum(errs_here) / len(errs_here):.4f}" if errs_here else "",
                "max_abs_error_m": f"{max(errs_here):.4f}" if errs_here else "",
            })
        summary.append({
            "Obstacle Type": name,
            "Detection Rate (%)": f"{100 * len(hits) / len(scans):.1f}",
            "Mean Abs. Range Error (m)": f"{sum(errors) / len(errors):.4f}" if errors else "",
            "Max. Range Error (m)": f"{max(errors):.4f}" if errors else "",
            "Undetected Positions": "; ".join(missed) if missed else "None",
            "Partial Positions": "; ".join(partial) if partial else "None",
            "Detected Scans": len(hits),
            "Total Scans": len(scans),
            "Positions Tested": len([1 for (typ, pos) in pos_groups if typ == name]),
        })
    for filename, rows, fields in [
        ("od0_summary.csv", summary, SUM_FIELDS),
        ("od0_by_position.csv", per_position, POSITION_FIELDS),
    ]:
        with (output_dir / filename).open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scans", type=int, default=10,
                        help="Fresh scans per position (default 10)")
    parser.add_argument("--tolerance", type=float, default=0.25,
                        help="Predefined target association tolerance in meters (default 0.25)")
    parser.add_argument("--margin-deg", type=float, default=2.5,
                        help="Angular margin beyond target half-width in degrees (default 2.5)")
    parser.add_argument("--output-dir", default="~/thesis_cart/od0_results")
    args = parser.parse_args()
    if args.scans < 1 or args.tolerance <= 0 or args.margin_deg < 0:
        parser.error("Use scans>=1, tolerance>0, and margin-deg>=0")
    outdir = Path(args.output_dir).expanduser().resolve()
    outdir.mkdir(parents=True, exist_ok=True)
    raw_path = outdir / "od0_raw_scans.csv"
    if raw_path.exists():
        parser.error(f"Raw file already exists: {raw_path}. Rename it or use another --output-dir.")

    print("\nOD-0 STATIONARY LIDAR TEST")
    print("LiDAR ONLY. Keep Arduino, motor drivers and motor battery disconnected.")
    print("Place ONE obstacle at a time; clear other objects from the viewing sector.")
    print("Distances are measured from the LiDAR ranging center to the nearest target surface.")
    print("The positive scan angle is counterclockwise about +Z (verify physical orientation).")
    print(f"Using {args.scans} scans per position; target tolerance +/-{args.tolerance:.2f} m.")
    print(f"Files will be saved at: {outdir}\n")
    rclpy.init()
    reader = ScanReader()
    raw_rows = []
    try:
        with raw_path.open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=RAW_FIELDS)
            writer.writeheader()
            for name, slug in TYPES:
                print("\n" + "=" * 60)
                print(f"OBJECT: {name}")
                print("=" * 60)
                width_m = ask_float(
                    "Measured approximate target width across the scanning plane (m)",
                    minimum=0.0,
                )
                for dist in DISTANCES:
                    for angle in ANGLES:
                        label = f"{dist:.1f}m/{angle:+.0f}deg"
                        print(f"\nPosition {label}, obstacle: {name}")
                        print("Place and keep the target STILL. Avoid other objects in the target sector.")
                        command = input("Press Enter to measure, or type 'skip' / 'quit': ").strip().lower()
                        if command in ("q", "quit", "exit"):
                            print("Session ended by user.")
                            return
                        if command in ("s", "skip"):
                            print("Skipped; not counted as a tested position.")
                            continue
                        actual_m = ask_float("Measured actual center-ray target distance (m)", default=dist)
                        # approximate target angular half-width; plus a margin for alignment,
                        # with a maximum of 35 degrees to limit background contamination.
                        half_width_deg = min(35.0, math.degrees(
                            math.atan2(width_m / 2.0, actual_m)) + args.margin_deg)
                        print(f"Using +/-{half_width_deg:.1f} degrees around the target bearing")
                        try:
                            scans = reader.collect(args.scans)
                        except TimeoutError as exc:
                            print(f"ERROR: {exc}. This position was NOT recorded. Please check LiDAR.")
                            continue
                        placement_rows = []
                        for k, scan in enumerate(scans, start=1):
                            measured, detected = measure_scan(
                                scan, angle, half_width_deg, actual_m, args.tolerance
                            )
                            error = abs(measured - actual_m) if detected else None
                            row = {
                                "obstacle_type": name,
                                "position": label,
                                "distance_label_m": dist,
                                "ground_truth_m": actual_m,
                                "target_angle_deg": angle,
                                "target_width_m": width_m,
                                "sector_half_width_deg": round(half_width_deg, 3),
                                "scan_number": k,
                                "measured_min_range_m": "" if measured is None else round(measured, 5),
                                "detected": int(detected),
                                "abs_error_m": "" if error is None else round(error, 5),
                            }
                            writer.writerow(row)
                            placement_rows.append(row)
                            raw_rows.append(row)
                        file.flush()
                        hits = [r for r in placement_rows if r["detected"]]
                        print(f"RESULT: {len(hits)}/{len(placement_rows)} detected "
                              f"({100*len(hits)/len(placement_rows):.1f}%).")
                        summary = save_reports(raw_rows, outdir)
                        for s in summary:
                            if s["Obstacle Type"] == name:
                                print("Cumulative type detection rate:", s["Detection Rate (%)"], "%")
                                print("MAE:", s["Mean Abs. Range Error (m)"],
                                      "m; max:", s["Max. Range Error (m)"], "m")
                        print("Saved CSV files.")
    except (KeyboardInterrupt, EOFError):
        print("\nSession interrupted; previously captured results remain saved.")
    finally:
        # If completed, interrupted, or intentionally stopped, still output summary.
        if raw_rows:
            summary = save_reports(raw_rows, outdir)
            print("\nFINAL/PARTIAL RESULTS:")
            for row in summary:
                print(f"{row['Obstacle Type']}: detection {row['Detection Rate (%)']}%, "
                      f"MAE {row['Mean Abs. Range Error (m)']} m, "
                      f"max {row['Max. Range Error (m)']} m, "
                      f"undetected: {row['Undetected Positions']}")
        reader.destroy_node()
        rclpy.shutdown()
        print(f"\nOutput directory: {outdir}")


if __name__ == "__main__":
    main()
