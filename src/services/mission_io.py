"""Mission file I/O and conversion helpers."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Union

from mavsdk.mission import MissionItem as MavsdkMissionItem
from mavsdk.mission import MissionPlan

from domain.mission import Mission, MissionItem

NAV_WAYPOINT_COMMAND = 16


def load_points_file(
    points_file: Union[str, Path], default_altitude: float = 10.0, name: str = "mission"
) -> Mission:
    """Load a Mission from a plain points file."""
    mission = Mission(name=name)
    with open(points_file, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            latitude, longitude = map(float, line.split(", "))
            mission.add_item(MissionItem.from_lat_lon(latitude, longitude, default_altitude))
    return mission


def load_plan_file(plan_file: Union[str, Path], name: str = "mission") -> Mission:
    """Load flyable waypoints from a QGroundControl ``.plan`` JSON file."""
    plan_data = json.loads(Path(plan_file).read_text())
    mission_list = plan_data.get("mission", {}).get("items", [])

    mission = Mission(name=name)
    for item in mission_list:
        if item.get("command") != NAV_WAYPOINT_COMMAND:
            continue
        latitude = item["params"][4]
        longitude = item["params"][5]
        altitude = item["params"][6]
        mission.add_item(MissionItem.from_lat_lon(latitude, longitude, altitude))
    return mission


def load_mission_file(
    mission_file: Union[str, Path], default_altitude: float = 10.0, name: str = "mission"
) -> Mission:
    """Load a Mission from a ``.plan`` file or a plain coordinate file."""
    mission_file = Path(mission_file)
    if not mission_file.exists():
        raise FileNotFoundError(f"Mission file {mission_file} not found")

    if mission_file.suffix == ".plan":
        return load_plan_file(mission_file, name=name)

    mission = Mission(name=name)
    with open(mission_file, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split(",")
            if len(parts) == 2:
                lat, lon = map(float, parts)
                alt = default_altitude
            elif len(parts) == 3:
                lat, lon, alt = map(float, parts)
            else:
                raise ValueError(f"Invalid mission line: {line!r}")
            mission.add_item(MissionItem.from_lat_lon(lat, lon, alt))
    return mission


def to_qgc_plan_dict(mission: Mission, templates_dir: Union[str, Path]) -> dict:
    """Serialize a Mission into the QGroundControl-style ``.plan`` dict."""
    templates_dir = Path(templates_dir)
    item_template = json.loads((templates_dir / "single_item_obj.json").read_text())
    mission_template = json.loads((templates_dir / "mission_template.json").read_text())
    plan_template = json.loads((templates_dir / "plan_template.json").read_text())

    mission_template["items"] = []
    for item in mission:
        entry = json.loads(json.dumps(item_template))  # deep copy per item
        entry["command"] = item.command
        entry["params"][4] = item.latitude
        entry["params"][5] = item.longitude
        entry["params"][6] = item.altitude
        mission_template["items"].append(entry)

    plan_template["mission"] = mission_template
    return plan_template


def save_as_plan(
    mission: Mission, output_path: Union[str, Path], templates_dir: Union[str, Path]
) -> None:
    """Write a Mission to a ``.plan`` JSON file."""
    plan_dict = to_qgc_plan_dict(mission, templates_dir)
    Path(output_path).write_text(json.dumps(plan_dict, indent=4))


def to_mavsdk_mission_plan(mission: Mission, cruise_speed_ms: float = 2.5) -> MissionPlan:
    """Convert a domain Mission into a mavsdk MissionPlan."""
    mavsdk_items = [
        MavsdkMissionItem(
            latitude_deg=item.latitude,
            longitude_deg=item.longitude,
            relative_altitude_m=item.altitude,
            speed_m_s=cruise_speed_ms,
            is_fly_through=False,
            gimbal_pitch_deg=float("nan"),
            gimbal_yaw_deg=float("nan"),
            loiter_time_s=1,
            acceptance_radius_m=float("nan"),
            yaw_deg=float("nan"),
            camera_action=MavsdkMissionItem.CameraAction.NONE,
            camera_photo_distance_m=float("nan"),
            camera_photo_interval_s=float("nan"),
            vehicle_action=MavsdkMissionItem.VehicleAction.NONE,
        )
        for item in mission
    ]
    return MissionPlan(mavsdk_items)
