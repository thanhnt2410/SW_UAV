"""Domain model for a single UAV (drone).

This module consolidates state that today is spread across a global
``UAVs`` dict-of-dicts (e.g. ``UAVs[i]["system"]``, ``UAVs[i]["status"]``)
plus several parallel lists indexed by ``uav_index`` in
``interface_wrapper.py`` (``uav_stream_frame_cnt``, ``uav_stream_threads``,
``uav_detection_models``, ...). Keeping N separate lists in sync by index
is fragile — this module replaces all of that with one ``UAV`` object per
drone.

Design notes
------------
- ``UAVConfig`` holds values set once at startup and rarely changed
  (id, connection address, param file path, detection model path).
- ``UAVTelemetry`` holds values refreshed continuously from mavsdk
  telemetry streams (position, battery, gps, arm status, ...).
- ``UAV`` ties both together plus the live mavsdk connection.

This module has no runtime dependency on PyQt, and only a type-checking
dependency on mavsdk, so it can be unit tested without a GUI or a real
drone connection.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
from typing import Optional, Tuple, TYPE_CHECKING

if TYPE_CHECKING:
    # Only needed for type checkers — avoids a hard runtime dependency on
    # mavsdk for anything that just needs the UAV/UAVConfig/UAVTelemetry
    # shapes (e.g. unit tests, the planning package).
    from mavsdk import System


@dataclass
class UAVConfig:
    """Static configuration for a UAV, set once at startup.

    Replaces scattered globals such as ``UAVs[i]["ID"]``,
    ``UAVs[i]["system_address"]``, ``UAVs[i]["init_params"]`` and the
    per-index ``uav_detection_models`` list.
    """

    index: int
    """1-based UAV index, matching the numbering already used throughout
    the UI and mavsdk-related functions as ``uav_index``."""

    uav_id: str
    """Human readable / mavlink ID for this UAV (was ``drone['ID']``)."""

    system_address: str
    """mavsdk connection string, e.g. "udpin://127.0.0.1:14541"."""

    init_params_path: Optional[str] = None
    """Path to the parameter file loaded on connect (was
    ``drone['init_params']``, consumed by ``uav_fn_import_params``)."""

    detection_enabled: bool = False
    """Whether object detection (YOLO) is enabled for this UAV's stream
    (was ``drone["detection_enable"]``)."""

    detection_model_path: Optional[str] = None
    """Path to the YOLO weights file for this UAV (was
    ``config.model_uav_paths[uav_index]``)."""


@dataclass
class UAVTelemetry:
    """Live telemetry snapshot for a UAV, refreshed on every poll cycle.

    Replaces the nested ``UAVs[i]["status"]`` dict that used to hold keys
    like ``connection_status``, ``position_status``, ``altitude_status``,
    ``mode_status``, ``battery_status``, ``arming_status``, ``gps_status``.
    """

    connected: bool = False
    armed: bool = False
    flight_mode: Optional[str] = None

    latitude: Optional[float] = None
    longitude: Optional[float] = None
    altitude_relative_m: Optional[float] = None
    altitude_msl_m: Optional[float] = None

    battery_percent: Optional[float] = None
    gps_fix_type: Optional[str] = None

    on_mission: bool = False
    mission_start_time: Optional[datetime] = None

    def as_position(self) -> Optional[Tuple[float, float]]:
        """Return (latitude, longitude), or None if position is unknown yet."""
        if self.latitude is None or self.longitude is None:
            return None
        return (self.latitude, self.longitude)

    def has_healthy_gps_fix(self) -> bool:
        """Best-effort check mirroring the old inline
        ``gps_status.value < 3`` warning in ``uav_fn_get_gps``.

        Kept as a string comparison (rather than importing mavsdk's enum
        directly) so this module stays free of a hard mavsdk dependency.
        The service layer that talks to mavsdk is responsible for mapping
        the real enum value to one of these strings.
        """
        return self.gps_fix_type in {"FIX_3D", "FIX_3D_DGPS", "RTK_FIXED", "RTK_FLOAT"}


@dataclass
class UAV:
    """A single UAV: its static config, live telemetry, and connection.

    One instance of this class replaces one "slot" that today is spread
    across ``UAVs[uav_index]`` plus several parallel lists such as
    ``self.uav_stream_threads[i]`` and ``self.uav_stream_frame_cnt[i]`` in
    ``interface_wrapper.py``.

    A fleet of UAVs should be represented as ``list[UAV]`` (or
    ``dict[int, UAV]`` keyed by index) inside ``DroneService`` — not as
    several parallel lists that must be kept in sync by hand.
    """

    config: UAVConfig
    telemetry: UAVTelemetry = field(default_factory=UAVTelemetry)

    system: Optional["System"] = None
    """The live mavsdk.System connection, once connected. None until
    DroneService establishes a connection for this UAV."""

    stream_frame_count: int = 0
    """Replaces ``self.uav_stream_frame_cnt[i]``."""

    stream_thread: Optional[object] = None
    """Replaces ``self.uav_stream_threads[i]``. Typed as ``object`` here
    (instead of the real StreamQtThread) so this module has no PyQt
    dependency; the streaming service can narrow the type on its side."""

    @property
    def index(self) -> int:
        return self.config.index

    @property
    def is_connected(self) -> bool:
        return self.telemetry.connected and self.system is not None

    def __repr__(self) -> str:  # pragma: no cover - convenience only
        return (
            f"UAV(index={self.config.index}, id={self.config.uav_id!r}, "
            f"connected={self.is_connected}, armed={self.telemetry.armed})"
        )