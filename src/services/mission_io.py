"""Mission file I/O and format conversion.

Everything that touches a disk, JSON, or mavsdk's mission types lives
here — deliberately kept out of ``domain/mission.py``, which only holds
plain data (``Mission``, ``MissionItem``) with no I/O and no external
type dependencies.

Replaces:
- the file-reading half of
  ``drone_utils.convert_pointsFile_to_missionPlan`` (points ``.txt`` -> Mission)
- the ``.plan`` / ``.txt`` parsing branch inside
  ``drone_utils.uav_fn_upload_mission`` (mission file -> mavsdk MissionPlan)
- the JSON template-patching in ``convert_pointsFile_to_missionPlan``
  (Mission -> QGC ``.plan`` dict) — see the bug note below

Bug fixed here (also noted in the domain/mission.py history): the
original ``convert_pointsFile_to_missionPlan`` loaded ``item_template``
once, mutated it in a loop, and appended the *same* dict object on every
iteration. Since dicts are stored by reference, every appended item ended
up identical — the exported ``.plan`` file only ever contained N copies
of the *last* waypoint. ``to_qgc_plan_dict`` below deep-copies the
template per item to avoid this.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Union

from mavsdk.mission import MissionItem as MavsdkMissionItem
from mavsdk.mission import MissionPlan

from domain.mission import Mission, MissionItem

# MAVLink command id for a simple waypoint (MAV_CMD_NAV_WAYPOINT). Only
# items with this command are treated as flyable waypoints when reading a
# QGC .plan file — mirrors the ``if item.get("command") != 16: continue``
# check in the original uav_fn_upload_mission.
NAV_WAYPOINT_COMMAND = 16


# ----------------------------------------------------------------------
# Reading missions from disk
# ----------------------------------------------------------------------
def load_points_file(
    points_file: Union[str, Path], default_altitude: float = 10.0, name: str = "mission"
) -> Mission:
    """Load a Mission from a plain points file: one "latitude, longitude"
    pair per line. Replaces the file-reading half of
    ``convert_pointsFile_to_missionPlan``.
    """
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
    """Load a Mission from a QGroundControl ``.plan`` JSON file, keeping
    only NAV_WAYPOINT items. Replaces the ``.plan``-parsing branch of
    ``uav_fn_upload_mission``.
    """
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
    """Load a Mission from either a ``.plan`` file or a simple ``.txt`` /
    ``.csv``-style points file, dispatching on extension exactly like
    ``uav_fn_upload_mission`` did — but returning a ``Mission`` instead of
    a raw list of ``(lat, lon, alt)`` tuples.

    The ``.txt`` branch also accepts an optional third "altitude" column
    per line (``lat,lon,alt``), matching the original's ``len(parts) == 3``
    handling, falling back to ``default_altitude`` for 2-column lines.
    """
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


# ----------------------------------------------------------------------
# Writing missions to disk (QGC .plan format)
# ----------------------------------------------------------------------
def to_qgc_plan_dict(mission: Mission, templates_dir: Union[str, Path]) -> dict:
    """Serialize a Mission into the QGroundControl-style ``.plan`` dict,
    reusing the project's existing template files (``plan_template.json``
    / ``mission_template.json`` / ``single_item_obj.json``) — deep-copying
    the item template per waypoint (see module docstring for the bug this
    avoids).

    Args:
        mission: the Mission to serialize.
        templates_dir: directory containing the three template JSON files
            (previously hardcoded as ``"./mission/"``).
    """
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
    """Write a Mission to a ``.plan`` JSON file. Replaces the hardcoded
    ``./mission/mission_plan.json`` output path in the original
    ``convert_pointsFile_to_missionPlan`` with an explicit argument.
    """
    plan_dict = to_qgc_plan_dict(mission, templates_dir)
    Path(output_path).write_text(json.dumps(plan_dict, indent=4))


# ----------------------------------------------------------------------
# Converting to mavsdk's MissionPlan (for DroneService.upload_mission)
# ----------------------------------------------------------------------
def to_mavsdk_mission_plan(mission: Mission, cruise_speed_ms: float = 2.5) -> MissionPlan:
    """Convert a domain Mission into a ``mavsdk.mission.MissionPlan``,
    ready to hand to ``System.mission.upload_mission(...)``.

    Field defaults below mirror exactly what ``uav_fn_upload_mission``
    used to hardcode per item (fly-through disabled, no gimbal control,
    no camera action, 1s loiter time). ``cruise_speed_ms`` is the one
    value that used to be a bare literal (``speed_m_s=2.5``); it's now a
    parameter so callers can override it per-mission instead of editing
    this function.
    """
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