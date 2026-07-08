"""Domain model for a single UAV (drone).

This module consolidates state into a UAV object.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
from typing import Optional, Tuple, TYPE_CHECKING, Dict, Any

if TYPE_CHECKING:
    from mavsdk import System


@dataclass
class UAVConfig:
    index: int
    uav_id: str
    system_address: str
    streaming_address: str = ""
    init_params_path: Optional[str] = None
    
    connection_allow: bool = False
    streaming_enable: bool = False
    detection_enabled: bool = False
    recording_enable: bool = False
    
    detection_model_path: Optional[str] = None
    init_params: Dict[str, Any] = field(default_factory=dict)
    overwrite_params: bool = False


@dataclass
class UAVTelemetry:
    connected: bool = False
    armed: bool = False
    flight_mode: str = "N/A"
    
    latitude: Optional[float] = None
    longitude: Optional[float] = None
    altitude_relative_m: Optional[float] = None
    altitude_msl_m: Optional[float] = None
    
    battery_percent: str = "N/A"
    gps_fix_type: str = "N/A"
    
    on_mission: bool = False
    mission_start_time: str = ""
    
    streaming_status: bool = False
    actuator_status: bool = False

    def as_position(self) -> Optional[Tuple[float, float]]:
        if self.latitude is None or self.longitude is None:
            return None
        return (self.latitude, self.longitude)

    def has_healthy_gps_fix(self) -> bool:
        return self.gps_fix_type in {"FIX_3D", "FIX_3D_DGPS", "RTK_FIXED", "RTK_FLOAT"}


@dataclass
class UAV:
    config: UAVConfig
    telemetry: UAVTelemetry = field(default_factory=UAVTelemetry)

    system: Optional["System"] = None
    server: Dict[str, Any] = field(default_factory=lambda: {"shell": None, "start": False})
    rescue_first_time: bool = True

    stream_frame_count: int = 0
    stream_thread: Optional[object] = None
    detection_model: Optional[object] = None

    @property
    def index(self) -> int:
        return self.config.index

    @property
    def is_connected(self) -> bool:
        return self.telemetry.connected and self.system is not None

    def __repr__(self) -> str:
        return (
            f"UAV(index={self.config.index}, id={self.config.uav_id!r}, "
            f"connected={self.is_connected})"
        )
