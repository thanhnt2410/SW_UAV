"""Domain model for UAV missions (flight plans).

A ``Mission`` is an ordered list of ``MissionItem`` waypoints.

This module is intentionally pure data: no file I/O, no mavsdk types, no
JSON. Reading/writing mission files and converting to/from external
formats (points ``.txt``, QGroundControl ``.plan``, mavsdk's
``MissionPlan``) lives in ``services/mission_io.py`` instead — see that
module's docstring for why the split matters.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Iterator, List

# MAVLink command id for a simple waypoint (MAV_CMD_NAV_WAYPOINT), used as
# a named constant instead of an unexplained magic index.
DEFAULT_WAYPOINT_COMMAND = 16


@dataclass
class MissionItem:
    """A single waypoint in a mission."""

    latitude: float
    longitude: float
    altitude: float = 10.0

    command: int = DEFAULT_WAYPOINT_COMMAND
    """MAVLink command id. Defaults to a plain waypoint, matching current
    behaviour, but left configurable for future item types (loiter, land,
    RTL, ...) instead of being baked into a single hardcoded template."""

    extra_params: Dict[str, Any] = field(default_factory=dict)
    """Escape hatch for mavlink/QGC params not modeled explicitly yet, so
    this class doesn't need to change for every new field discovered
    later (e.g. speed, loiter time, gimbal angles — see
    ``services/mission_io.to_mavsdk_mission_plan`` for where those are
    actually consumed)."""

    @classmethod
    def from_lat_lon(cls, latitude: float, longitude: float, altitude: float = 10.0) -> "MissionItem":
        return cls(latitude=latitude, longitude=longitude, altitude=altitude)


@dataclass
class Mission:
    """An ordered collection of mission items for one UAV."""

    name: str = "mission"
    items: List[MissionItem] = field(default_factory=list)

    def add_item(self, item: MissionItem) -> None:
        self.items.append(item)

    def __len__(self) -> int:
        return len(self.items)

    def __iter__(self) -> Iterator[MissionItem]:
        return iter(self.items)

    @classmethod
    def from_waypoints(
        cls, waypoints: List[tuple], default_altitude: float = 10.0, name: str = "mission"
    ) -> "Mission":
        """Build a Mission from a plain list of ``(lat, lon)`` or
        ``(lat, lon, alt)`` tuples already in memory — no file access.
        For loading from a points/.plan file on disk, use
        ``services/mission_io.py`` instead.
        """
        mission = cls(name=name)
        for wp in waypoints:
            if len(wp) == 2:
                lat, lon = wp
                alt = default_altitude
            else:
                lat, lon, alt = wp
            mission.add_item(MissionItem.from_lat_lon(lat, lon, alt))
        return mission