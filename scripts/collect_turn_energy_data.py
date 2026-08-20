#!/usr/bin/env python3
"""Collect PX4 SITL energy/time data for single-turn UAV paths.

The script connects to one PX4/Gazebo UAV through MAVSDK, takes off once, then
flies a sequence of synthetic paths. Each path contains a straight entry
segment, a circular arc approximated by short waypoints, and a straight exit
segment. Battery telemetry is integrated during the mission only.
"""

from __future__ import annotations

import argparse
import asyncio
import csv
import math
import time
import uuid
from dataclasses import dataclass
from pathlib import Path
from statistics import mean
from typing import Iterable, List, Optional, Sequence, Tuple

from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan


EARTH_RADIUS_M = 6378137.0


LatLon = Tuple[float, float]
LocalXY = Tuple[float, float]


@dataclass(frozen=True)
class TurnMissionGeometry:
    angle_deg: float
    angle_rad: float
    speed_m_s: float
    turn_radius_m: float
    pre_length_m: float
    post_length_m: float
    arc_length_m: float
    total_distance_m: float
    local_points_xy: List[LocalXY]


@dataclass
class BatteryEnergyResult:
    measured_time_s: float
    measured_energy_wh: float
    mean_power_w: float
    battery_voltage_mean_v: float
    battery_current_mean_a: float
    valid_energy_intervals: int
    sample_count: int

    @property
    def valid(self) -> bool:
        return self.valid_energy_intervals > 0 and self.sample_count > 0


def parse_angles(raw: str) -> List[float]:
    return [float(part.strip()) for part in raw.split(",") if part.strip()]


def local_xy_to_lat_lon(origin: LatLon, point: LocalXY) -> LatLon:
    lat0, lon0 = origin
    x_east_m, y_north_m = point
    lat = lat0 + math.degrees(y_north_m / EARTH_RADIUS_M)
    lon = lon0 + math.degrees(x_east_m / (EARTH_RADIUS_M * math.cos(math.radians(lat0))))
    return lat, lon


def build_turn_geometry(
    angle_deg: float,
    speed_m_s: float,
    turn_radius_m: float,
    pre_length_m: float,
    post_length_m: float,
    arc_step_m: float,
) -> TurnMissionGeometry:
    if speed_m_s <= 0:
        raise ValueError("speed_m_s must be > 0")
    if turn_radius_m <= 0:
        raise ValueError("turn_radius_m must be > 0")
    if pre_length_m < 0 or post_length_m < 0:
        raise ValueError("pre_length_m and post_length_m must be >= 0")
    if arc_step_m <= 0:
        raise ValueError("arc_step_m must be > 0")

    angle_rad = math.radians(angle_deg)
    sign = 1.0 if angle_rad >= 0 else -1.0
    theta_abs = abs(angle_rad)
    points: List[LocalXY] = [(0.0, 0.0), (pre_length_m, 0.0)]

    if theta_abs > 1e-9:
        steps = max(2, int(math.ceil(turn_radius_m * theta_abs / arc_step_m)))
        for idx in range(1, steps + 1):
            phi = theta_abs * idx / steps
            x = pre_length_m + turn_radius_m * math.sin(phi)
            y = sign * turn_radius_m * (1.0 - math.cos(phi))
            points.append((x, y))

        exit_heading = sign * theta_abs
        end_x, end_y = points[-1]
        points.append(
            (
                end_x + post_length_m * math.cos(exit_heading),
                end_y + post_length_m * math.sin(exit_heading),
            )
        )
    else:
        points.append((pre_length_m + post_length_m, 0.0))

    arc_length_m = turn_radius_m * theta_abs
    total_distance_m = pre_length_m + arc_length_m + post_length_m
    return TurnMissionGeometry(
        angle_deg=angle_deg,
        angle_rad=angle_rad,
        speed_m_s=speed_m_s,
        turn_radius_m=turn_radius_m,
        pre_length_m=pre_length_m,
        post_length_m=post_length_m,
        arc_length_m=arc_length_m,
        total_distance_m=total_distance_m,
        local_points_xy=points,
    )


def to_mission_plan(origin: LatLon, geometry: TurnMissionGeometry, altitude_m: float) -> MissionPlan:
    items = []
    for point in geometry.local_points_xy:
        lat, lon = local_xy_to_lat_lon(origin, point)
        items.append(
            MissionItem(
                latitude_deg=lat,
                longitude_deg=lon,
                relative_altitude_m=altitude_m,
                speed_m_s=geometry.speed_m_s,
                is_fly_through=True,
                gimbal_pitch_deg=float("nan"),
                gimbal_yaw_deg=float("nan"),
                loiter_time_s=0.0,
                acceptance_radius_m=1.0,
                yaw_deg=float("nan"),
                camera_action=MissionItem.CameraAction.NONE,
                camera_photo_distance_m=float("nan"),
                camera_photo_interval_s=float("nan"),
                vehicle_action=MissionItem.VehicleAction.NONE,
            )
        )
    return MissionPlan(items)


async def wait_connected(drone: System, timeout_s: float) -> None:
    async def wait() -> None:
        async for state in drone.core.connection_state():
            if state.is_connected:
                return

    await asyncio.wait_for(wait(), timeout=timeout_s)


async def first_position(drone: System, timeout_s: float) -> LatLon:
    async def wait() -> LatLon:
        async for position in drone.telemetry.position():
            if math.isfinite(position.latitude_deg) and math.isfinite(position.longitude_deg):
                return position.latitude_deg, position.longitude_deg
        raise TimeoutError("Position telemetry stream ended")

    return await asyncio.wait_for(wait(), timeout=timeout_s)


async def wait_for_mission_mode(drone: System, timeout_s: float):
    async def wait():
        async for flight_mode in drone.telemetry.flight_mode():
            if "MISSION" in str(flight_mode).upper():
                return flight_mode
        raise RuntimeError("Flight-mode telemetry stream ended")

    return await asyncio.wait_for(wait(), timeout=timeout_s)


async def first_flight_mode_value(drone: System, timeout_s: float):
    async def wait():
        async for flight_mode in drone.telemetry.flight_mode():
            return flight_mode
        raise RuntimeError("Flight-mode telemetry stream ended")

    return await asyncio.wait_for(wait(), timeout=timeout_s)


async def first_mission_progress(drone: System, timeout_s: float):
    async def wait():
        async for progress in drone.mission.mission_progress():
            return progress
        raise RuntimeError("Mission-progress stream ended")

    return await asyncio.wait_for(wait(), timeout=timeout_s)


async def wait_relative_altitude(drone: System, target_altitude_m: float, timeout_s: float) -> None:
    async def wait() -> None:
        last_report_time = 0.0
        async for position in drone.telemetry.position():
            now = time.monotonic()
            if now - last_report_time >= 2.0:
                print(
                    f"[collect] Relative altitude: {position.relative_altitude_m:.1f} m "
                    f"(target >= {target_altitude_m - 1.0:.1f} m)"
                )
                last_report_time = now
            if position.relative_altitude_m >= target_altitude_m - 1.0:
                return

    await asyncio.wait_for(wait(), timeout=timeout_s)


async def wait_mission_finished(drone: System, timeout_s: float) -> None:
    async def wait() -> None:
        while True:
            if await drone.mission.is_mission_finished():
                return
            await asyncio.sleep(0.25)

    await asyncio.wait_for(wait(), timeout=timeout_s)


async def integrate_battery_during(drone: System, task: asyncio.Task) -> BatteryEnergyResult:
    energy_wh = 0.0
    valid_intervals = 0
    voltage_samples: List[float] = []
    current_samples: List[float] = []
    power_samples: List[float] = []
    last_sample_time: Optional[float] = None
    start_time = time.monotonic()

    async def collect() -> None:
        nonlocal energy_wh, valid_intervals, last_sample_time
        async for battery in drone.telemetry.battery():
            now = time.monotonic()
            voltage_v = getattr(battery, "voltage_v", float("nan"))
            current_a = getattr(battery, "current_battery_a", float("nan"))
            if math.isfinite(voltage_v) and math.isfinite(current_a) and voltage_v > 0.0 and current_a >= 0.0:
                voltage_samples.append(voltage_v)
                current_samples.append(current_a)
                power_samples.append(voltage_v * current_a)
                if last_sample_time is not None:
                    dt_s = min(max(now - last_sample_time, 0.0), 2.0)
                    energy_wh += voltage_v * current_a * dt_s / 3600.0
                    valid_intervals += 1
                last_sample_time = now
            else:
                last_sample_time = None

    battery_task = asyncio.create_task(collect())
    try:
        await task
    finally:
        battery_task.cancel()
        await asyncio.gather(battery_task, return_exceptions=True)

    measured_time_s = time.monotonic() - start_time
    return BatteryEnergyResult(
        measured_time_s=measured_time_s,
        measured_energy_wh=energy_wh,
        mean_power_w=mean(power_samples) if power_samples else float("nan"),
        battery_voltage_mean_v=mean(voltage_samples) if voltage_samples else float("nan"),
        battery_current_mean_a=mean(current_samples) if current_samples else float("nan"),
        valid_energy_intervals=valid_intervals,
        sample_count=len(power_samples),
    )


async def start_mission_and_wait_mode(drone: System, timeout_s: float = 10.0):
    """Start mission with item index reset and retry if PX4 momentarily stays in HOLD."""
    for attempt in range(1, 4):
        try:
            await drone.mission.set_current_mission_item_index(0)
        except Exception:
            pass
        await asyncio.sleep(0.2)
        try:
            await drone.mission.start_mission()
        except Exception as e:
            print(f"[collect] (attempt {attempt}/3) start_mission: {e}")
        
        try:
            flight_mode = await wait_for_mission_mode(drone, timeout_s=2.5)
            return flight_mode
        except asyncio.TimeoutError:
            pass
        await asyncio.sleep(0.5)

    # Final attempt
    try:
        await drone.mission.set_current_mission_item_index(0)
    except Exception:
        pass
    await drone.mission.start_mission()
    return await wait_for_mission_mode(drone, timeout_s=timeout_s)


async def run_single_mission(
    drone: System,
    geometry: TurnMissionGeometry,
    altitude_m: float,
    mission_timeout_s: float,
) -> BatteryEnergyResult:
    print("[collect] Reading current position...")
    origin = await first_position(drone, timeout_s=10.0)
    print(f"[collect] Mission origin: lat={origin[0]:.7f}, lon={origin[1]:.7f}")
    mission_plan = to_mission_plan(origin, geometry, altitude_m)
    print(f"[collect] Uploading mission with {len(mission_plan.mission_items)} waypoints...")
    await drone.mission.clear_mission()
    await drone.mission.set_return_to_launch_after_mission(False)
    await drone.action.set_current_speed(geometry.speed_m_s)
    await drone.mission.upload_mission(mission_plan)
    await asyncio.sleep(0.5)
    print("[collect] Starting mission...")
    try:
        flight_mode = await start_mission_and_wait_mode(drone, timeout_s=10.0)
    except asyncio.TimeoutError as exc:
        current_mode = await first_flight_mode_value(drone, timeout_s=2.0)
        raise RuntimeError(
            f"PX4 did not enter MISSION mode within 10 seconds (mode={current_mode}). "
            "Check the PX4 console for a mode rejection or failsafe message."
        ) from exc
    progress = await first_mission_progress(drone, timeout_s=5.0)
    print(
        f"[collect] Flight mode={flight_mode}; "
        f"mission progress={progress.current}/{progress.total}"
    )
    print("[collect] Mission active; collecting battery telemetry...")

    async def wait_for_finish() -> None:
        await wait_mission_finished(drone, mission_timeout_s)

    wait_task = asyncio.create_task(wait_for_finish())
    return await integrate_battery_during(drone, wait_task)


def build_rows_with_baseline(rows: List[dict]) -> List[dict]:
    valid_baselines = [
        row for row in rows
        if row["valid"] and abs(float(row["angle_deg"])) < 1e-9 and float(row["total_distance_m"]) > 0.0
    ]
    if not valid_baselines:
        for row in rows:
            row["turn_time_s"] = ""
            row["turn_energy_wh"] = ""
        return rows

    baseline_time_per_m = mean(
        float(row["measured_time_s"]) / float(row["total_distance_m"])
        for row in valid_baselines
    )
    baseline_energy_per_m = mean(
        float(row["measured_energy_wh"]) / float(row["total_distance_m"])
        for row in valid_baselines
    )

    for row in rows:
        total_distance_m = float(row["total_distance_m"])
        row["turn_time_s"] = float(row["measured_time_s"]) - baseline_time_per_m * total_distance_m
        row["turn_energy_wh"] = float(row["measured_energy_wh"]) - baseline_energy_per_m * total_distance_m
    return rows


def write_rows(csv_path: Path, rows: List[dict]) -> None:
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "run_id",
        "repeat_index",
        "angle_deg",
        "angle_rad",
        "speed_m_s",
        "turn_radius_m",
        "arc_length_m",
        "pre_length_m",
        "post_length_m",
        "total_distance_m",
        "measured_time_s",
        "measured_energy_wh",
        "turn_time_s",
        "turn_energy_wh",
        "mean_power_w",
        "battery_voltage_mean_v",
        "battery_current_mean_a",
        "valid_energy_intervals",
        "sample_count",
        "valid",
    ]
    with csv_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


async def collect(args: argparse.Namespace) -> None:
    drone = System()
    await drone.connect(system_address=args.system_address)
    print(f"[collect] Waiting for {args.system_address}...")
    await wait_connected(drone, timeout_s=args.connect_timeout_s)
    print("[collect] Connected.")

    await drone.telemetry.set_rate_battery(args.battery_rate_hz)
    await drone.action.set_takeoff_altitude(args.altitude_m)
    await drone.action.set_current_speed(args.speed_m_s)

    if not args.skip_takeoff:
        print(f"[collect] Taking off to {args.altitude_m:.1f} m...")
        await drone.action.arm()
        await drone.action.takeoff()
        await wait_relative_altitude(drone, args.altitude_m, timeout_s=args.takeoff_timeout_s)
        await asyncio.sleep(args.settle_s)

    rows: List[dict] = []
    run_group = uuid.uuid4().hex[:8]
    angles = parse_angles(args.angles)

    for repeat_index in range(1, args.repeats + 1):
        for angle_deg in angles:
            geometry = build_turn_geometry(
                angle_deg=angle_deg,
                speed_m_s=args.speed_m_s,
                turn_radius_m=args.turn_radius_m,
                pre_length_m=args.pre_length_m,
                post_length_m=args.post_length_m,
                arc_step_m=args.arc_step_m,
            )
            run_id = f"{run_group}_{repeat_index:02d}_{angle_deg:g}"
            print(
                f"[collect] Run {run_id}: angle={angle_deg:g} deg, "
                f"distance={geometry.total_distance_m:.2f} m, waypoints={len(geometry.local_points_xy)}"
            )
            result = await run_single_mission(
                drone=drone,
                geometry=geometry,
                altitude_m=args.altitude_m,
                mission_timeout_s=args.mission_timeout_s,
            )
            rows.append(
                {
                    "run_id": run_id,
                    "repeat_index": repeat_index,
                    "angle_deg": geometry.angle_deg,
                    "angle_rad": geometry.angle_rad,
                    "speed_m_s": geometry.speed_m_s,
                    "turn_radius_m": geometry.turn_radius_m,
                    "arc_length_m": geometry.arc_length_m,
                    "pre_length_m": geometry.pre_length_m,
                    "post_length_m": geometry.post_length_m,
                    "total_distance_m": geometry.total_distance_m,
                    "measured_time_s": result.measured_time_s,
                    "measured_energy_wh": result.measured_energy_wh,
                    "turn_time_s": "",
                    "turn_energy_wh": "",
                    "mean_power_w": result.mean_power_w,
                    "battery_voltage_mean_v": result.battery_voltage_mean_v,
                    "battery_current_mean_a": result.battery_current_mean_a,
                    "valid_energy_intervals": result.valid_energy_intervals,
                    "sample_count": result.sample_count,
                    "valid": result.valid,
                }
            )
            rows = build_rows_with_baseline(rows)
            write_rows(args.output, rows)
            print(
                f"[collect] Saved {args.output} "
                f"(energy={result.measured_energy_wh:.4f} Wh, time={result.measured_time_s:.2f}s, valid={result.valid})"
            )
            await asyncio.sleep(args.cooldown_s)

    if args.rtl_after:
        print("[collect] Returning to launch...")
        await drone.action.return_to_launch()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--system-address", default="udpin://127.0.0.1:14540")
    parser.add_argument("--output", type=Path, default=Path("logs/turn_energy/turn_energy_dataset.csv"))
    parser.add_argument("--angles", default="0,15,30,45,60,90,120,150,180")
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument("--speed-m-s", type=float, default=2.5)
    parser.add_argument("--turn-radius-m", type=float, default=5.0)
    parser.add_argument("--altitude-m", type=float, default=13.0)
    parser.add_argument("--pre-length-m", type=float, default=30.0)
    parser.add_argument("--post-length-m", type=float, default=30.0)
    parser.add_argument("--arc-step-m", type=float, default=1.5)
    parser.add_argument("--battery-rate-hz", type=float, default=10.0)
    parser.add_argument("--connect-timeout-s", type=float, default=30.0)
    parser.add_argument("--takeoff-timeout-s", type=float, default=60.0)
    parser.add_argument("--mission-timeout-s", type=float, default=180.0)
    parser.add_argument("--settle-s", type=float, default=2.0)
    parser.add_argument("--cooldown-s", type=float, default=2.0)
    parser.add_argument("--skip-takeoff", action="store_true")
    parser.add_argument("--rtl-after", action="store_true")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> None:
    args = build_parser().parse_args(argv)
    asyncio.run(collect(args))


if __name__ == "__main__":
    main()
