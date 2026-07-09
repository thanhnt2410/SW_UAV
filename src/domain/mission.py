"""Domain model for UAV missions."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Iterator, List

DEFAULT_WAYPOINT_COMMAND = 16


@dataclass
class MissionItem:
    """A single waypoint in a mission."""

    latitude: float
    longitude: float
    altitude: float = 10.0

    command: int = DEFAULT_WAYPOINT_COMMAND
    """MAVLink command id."""

    extra_params: Dict[str, Any] = field(default_factory=dict)
    """Extra MAVLink/QGC params not modeled explicitly yet."""

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
        """Build a Mission from ``(lat, lon)`` or ``(lat, lon, alt)`` tuples."""
        mission = cls(name=name)
        for wp in waypoints:
            if len(wp) == 2:
                lat, lon = wp
                alt = default_altitude
            else:
                lat, lon, alt = wp
            mission.add_item(MissionItem.from_lat_lon(lat, lon, alt))
        return mission
