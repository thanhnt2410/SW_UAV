"""
Service layer for managing and interacting with the UAV fleet.

This module encapsulates all MAVSDK-related logic, separating it from the
UI layer (interface_wrapper.py). It is responsible for creating, connecting,
commanding, and monitoring the status of all UAVs.
"""

import asyncio
import os
from datetime import datetime

from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan

from utils.mavsdk_server_utils import MAVSDKServer
from utils.drone_utils import uav_fn_overwrite_params, uav_fn_export_params, uav_fn_upload_mission


class DroneService:
    """
    Owns and manages a fleet of UAVs end-to-end: registration,
    connect/disconnect, telemetry refresh, and basic flight actions.
    """

    def __init__(self, config):
        self.config = config
        self.uavs = self._initialize_uavs()

    def _initialize_uavs(self):
        """Initializes the UAVs dictionary based on the loaded configuration."""
        try:
            return {
                uav_conf["id"]: {
                    "ID": uav_conf["id"],
                    "server": {
                        "shell": MAVSDKServer(
                            id=uav_conf["id"],
                            protocol=self.config.PROTOCOLS[uav_conf["id"] - 1],
                            server_host=self.config.SERVER_HOSTS[uav_conf["id"] - 1],
                            port=self.config.CLIENT_PORTS[uav_conf["id"] - 1],
                            bind_port=self.config.SERVER_PORTS[uav_conf["id"] - 1],
                        ),
                        "start": False,
                    },
                    "system": System(mavsdk_server_address="localhost", port=self.config.CLIENT_PORTS[uav_conf["id"] - 1]),
                    "system_address": self.config.SYSTEMS_ADDRESSES[uav_conf["id"] - 1],
                    "streaming_address": self.config.DEFAULT_STREAM_VIDEO_PATHS[uav_conf["id"] - 1],
                    "connection_allow": uav_conf["connection_allow"],
                    "streaming_enable": uav_conf["streaming_enable"],
                    "detection_enable": uav_conf["detection_enable"],
                    "recording_enable": uav_conf["recording_enable"],
                    "init_params": {
                        "longitude": self.config.init_pos[f"uav_{uav_conf['id']}"]["longitude"],
                        "latitude": self.config.init_pos[f"uav_{uav_conf['id']}"]["latitude"],
                        "altitude": uav_conf["init_alt"],
                    },
                    "overwrite_params": uav_conf["overwrite_params"],
                    "status": {
                        "connection_status": False, "streaming_status": False, "on_mission": False,
                        "mission_start_time": "", "arming_status": "N/A", "battery_status": "N/A",
                        "gps_status": "N/A", "mode_status": "N/A", "actuator_status": False,
                        "altitude_status": ["N/A", "N/A"], "position_status": ["N/A", "N/A"],
                    },
                    "rescue_first_time": True,
                }
                for uav_conf in self.config.uav['uavs']
            }
        except Exception as e:
            print(f"[FATAL] Error initializing UAVs in DroneService: {repr(e)}")
            raise

    def get_uav(self, uav_index):
        """Retrieves the data dictionary for a single UAV."""
        return self.uavs.get(uav_index)

    def get_all_uavs(self):
        """Retrieves the entire dictionary of UAVs."""
        return self.uavs

    # --- Connection Lifecycle ---
    async def connect(self, uav_index):
        uav = self.get_uav(uav_index)
        if not uav: raise ValueError(f"UAV {uav_index} not found.")
        if not uav["connection_allow"]: raise PermissionError(f"Connection not allowed for UAV {uav_index}")

        uav["status"]["connection_status"] = False

        # 1. Initialize server
        uav["server"]["shell"].stop()
        await asyncio.sleep(1)
        uav["server"]["shell"].start()
        uav["server"]["start"] = True
        await asyncio.sleep(5)

        # 2. Connect to system
        await uav["system"].connect(system_address=uav["system_address"])
        is_connected = False
        async for state in uav["system"].core.connection_state():
            is_connected = state.is_connected
            break
        if not is_connected: raise ConnectionError(f"Failed to connect to UAV {uav['ID']}")
        uav["status"]["connection_status"] = True

        # 3. Configure parameters
        await self._configure_uav_parameters(uav_index)

    async def _configure_uav_parameters(self, uav_index: int):
        """Configure UAV parameters after connection."""
        
        # Overwrite parameters from configuration
        await uav_fn_overwrite_params(
            self.uavs[uav_index], parameters=self.uavs[uav_index]["overwrite_params"]
        )
        
        # Set additional parameters manually
        await self.uavs[uav_index]["system"].action.set_takeoff_altitude(
            altitude=self.uavs[uav_index]["init_params"]["altitude"]
        )
        await self.uavs[uav_index]["system"].action.set_current_speed(3)
        
        try:
            await self.uavs[uav_index]["system"].param.set_param_float("RTL_RETURN_ALT", 5.0)
            print(f"[INFO] UAV-{uav_index}: Đặt độ cao RTL thành 5m thành công")
        except Exception as e:
            print(f"[ERROR] UAV-{uav_index}: Lỗi khi đặt RTL_RETURN_ALT - {e}")
        # Export parameters to file
        await uav_fn_export_params(
            drone=self.uavs[uav_index], save_path=self.config.parameter_data_files[uav_index - 1]
        )

    # --- Basic Flight Actions ---
    async def arm(self, uav_index):
        await self.get_uav(uav_index)["system"].action.arm()

    async def disarm(self, uav_index):
        await self.get_uav(uav_index)["system"].action.disarm()

    async def takeoff(self, uav_index):
        uav = self.get_uav(uav_index)
        await uav["system"].action.arm()
        await uav["system"].action.takeoff()

    async def land(self, uav_index):
        await self.get_uav(uav_index)["system"].action.land()

    async def return_to_launch(self, uav_index):
        await self.get_uav(uav_index)["system"].action.return_to_launch()

    async def goto_location(self, uav_index, latitude, longitude):
        uav = self.get_uav(uav_index)
        await uav["system"].action.goto_location(latitude, longitude, uav["init_params"]["altitude"], 0)

    # --- Mission Control ---

    async def start_mission(self, uav_index):
        await self.get_uav(uav_index)["system"].mission.start_mission()

    async def pause_mission(self, uav_index):
        await self.get_uav(uav_index)["system"].mission.pause_mission()

    async def uav_fn_do_mission(drone, mission_plan_file) -> None:
        """
        Execute a complete UAV mission from takeoff to landing.
        
        Args:
            drone (dict): UAV system dictionary
            mission_plan_file (str): Path to mission file with coordinates
        """
        print_mission_progress_task = None
        termination_task = None
        try:
            # Health check before mission
            # print(f"UAV-{drone['ID']} checking health before mission")
            await _check_uav_health(drone)
            
            # Clear any existing mission
            await drone["system"].mission.clear_mission()
            
            # Set up monitoring tasks
            print_mission_progress_task = asyncio.ensure_future(print_mission_progress(drone))
            running_tasks = [print_mission_progress_task]
            termination_task = asyncio.ensure_future(observe_is_in_air(drone, running_tasks))
            
            # Upload the mission
            await uav_fn_upload_mission(drone, mission_plan_file)
            await asyncio.sleep(1)
            
            # Connect to the UAV
            # Bỏ connect lại vì UAV đã connect từ lúc ấn nút trên giao diện rồi, gọi lại dễ gây crash
            # await drone["system"].connect(drone["system_address"])
            
            # Arm and take off
            # print(f"UAV-{drone['ID']} arming")
            await drone["system"].action.arm()
            await asyncio.sleep(2)
            
            # print(f"UAV-{drone['ID']} taking off")
            await drone["system"].action.takeoff()
            await asyncio.sleep(3)
            
            # Start the mission
            # print(f"UAV-{drone['ID']} starting mission")
            await drone["system"].mission.start_mission()
            await asyncio.sleep(3)
            await drone["system"].action.set_current_speed(3)
            
            # Wait for termination (landing)
            await termination_task
            
        except Exception as e:
            print(f"Error executing mission: {repr(e)}")
            
            # Try to cancel any running tasks
            try:
                cleanup_tasks = [
                    task for task in [print_mission_progress_task, termination_task]
                    if task is not None
                ]
                for task in cleanup_tasks:
                    if task is not None and not task.done():
                        task.cancel()
                if cleanup_tasks:
                    await asyncio.gather(*cleanup_tasks, return_exceptions=True)
            except Exception:
                pass
            raise e  # Ném lỗi ra để giao diện UI nhận được và hiển thị Popup

    # --- Telemetry ---
    async def get_status(self, uav_index):
        """Gathers all telemetry data for a UAV in parallel."""
        uav = self.get_uav(uav_index)
        if not uav or not uav["status"]["connection_status"]: return

        await asyncio.gather(
            self._get_position(uav), self._get_mode(uav), self._get_battery(uav),
            self._get_arm_status(uav), self._get_gps(uav)
        )

    async def _get_position(self, uav):
        async for p in uav["system"].telemetry.position():
            uav["status"]["altitude_status"] = [round(p.relative_altitude_m, 12), round(p.absolute_altitude_m, 12)]
            uav["status"]["position_status"] = [round(p.latitude_deg, 12), round(p.longitude_deg, 12)]
            break

    async def _get_mode(self, uav):
        async for m in uav["system"].telemetry.flight_mode():
            uav["status"]["mode_status"] = str(m)
            break

    async def _get_battery(self, uav):
        async for b in uav["system"].telemetry.battery():
            uav["status"]["battery_status"] = f"{round(b.remaining_percent * 100, 1)}%"
            break

    async def _get_arm_status(self, uav):
        async for a in uav["system"].telemetry.armed():
            uav["status"]["arming_status"] = "ARMED" if a else "DISARMED"
            break

    async def _get_gps(self, uav):
        async for g in uav["system"].telemetry.gps_info():
            uav["status"]["gps_status"] = str(g.fix_type)
            break

    # --- Parameters ---
    async def get_params(self, uav_index, param_list):
        uav = self.get_uav(uav_index)
        params = {}
        for p_name in param_list:
            try:
                val = await uav["system"].param.get_param_float(p_name)
                params[p_name] = val
            except Exception:
                params[p_name] = 'N/A'
        return params

    async def set_params(self, uav_index, params):
        uav = self.get_uav(uav_index)
        for p_name, p_val in params.items():
            await uav["system"].param.set_param_float(p_name, float(p_val))

    # --- Actuator ---
    async def toggle_actuator(self, uav_index):
        uav = self.get_uav(uav_index)
        current_state = uav["status"]["actuator_status"]
        new_state = not current_state
        if new_state: # Open
            await uav["system"].action.set_actuator(4, -1)
        else: # Close
            await uav["system"].action.set_actuator(4, 1)
        uav["status"]["actuator_status"] = new_state
        await asyncio.sleep(3)