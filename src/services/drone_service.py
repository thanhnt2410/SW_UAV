"""
Service layer for managing and interacting with the UAV fleet.

This module encapsulates all MAVSDK-related logic, separating it from the
UI layer (main_controller.py). It is responsible for creating, connecting,
commanding, and monitoring the status of all UAVs.
"""

import asyncio
import json
import math
import os
import time
from pathlib import Path

from mavsdk import System
from mavsdk.gimbal import ControlMode, GimbalMode
from mavsdk.mission import MissionItem, MissionPlan
from mavsdk.offboard import ActuatorControl, ActuatorControlGroup, OffboardError

from mavsdk_server.mavsdk_server_utils import MAVSDKServer
from utils.drone_utils import calculate_distance



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
            from domain.uav import UAV, UAVConfig, UAVTelemetry
            uavs = {}
            for uav_conf in self.config.uav['uavs']:
                idx = uav_conf["id"]
                px4_parameters = dict(self.config.PX4_PARAMETERS)
                px4_parameters.update(uav_conf.get("px4_parameters", {}) or {})
                cfg = UAVConfig(
                    index=idx,
                    uav_id=idx,
                    system_address=self.config.SYSTEMS_ADDRESSES[idx - 1],
                    streaming_address=self.config.DEFAULT_STREAM_VIDEO_PATHS[idx - 1],
                    connection_allow=uav_conf["connection_allow"],
                    streaming_enable=uav_conf["streaming_enable"],
                    detection_enabled=uav_conf["detection_enable"],
                    recording_enable=uav_conf["recording_enable"],
                    init_params={
                        "longitude": self.config.init_pos[f"uav_{idx}"]["longitude"],
                        "latitude": self.config.init_pos[f"uav_{idx}"]["latitude"],
                        "altitude": uav_conf["init_alt"],
                    },
                    overwrite_params=uav_conf.get("overwrite_params", {}),
                    px4_parameters=px4_parameters,
                )
                telemetry = UAVTelemetry()
                uav = UAV(config=cfg, telemetry=telemetry)
                uav.server = {
                    "shell": MAVSDKServer(
                        id=idx,
                        protocol=self.config.PROTOCOLS[idx - 1],
                        server_host=self.config.SERVER_HOSTS[idx - 1],
                        port=self.config.CLIENT_PORTS[idx - 1],
                        bind_port=self.config.SERVER_PORTS[idx - 1],
                    ),
                    "start": False,
                }
                uav.system = System(mavsdk_server_address="localhost", port=self.config.CLIENT_PORTS[idx - 1])
                uav.rescue_first_time = True
                uavs[idx] = uav
            return uavs
        except Exception as e:
            print(f"[FATAL] Error initializing UAVs in DroneService: {repr(e)}")
            raise

    def get_uav(self, uav_index):
        """Retrieves the data dictionary for a single UAV."""
        return self.uavs.get(uav_index)

    def get_all_uavs(self):
        """Retrieves the entire dictionary of UAVs."""
        return self.uavs

    async def connect(self, uav_index):
        uav = self.get_uav(uav_index)
        if not uav: raise ValueError(f"UAV {uav_index} not found.")
        if not uav.config.connection_allow: raise PermissionError(f"Connection not allowed for UAV {uav_index}")

        uav.telemetry.connected = False

        uav.server["shell"].stop()
        await asyncio.sleep(1)
        uav.server["shell"].start()
        uav.server["start"] = True
        await asyncio.sleep(5)

        await uav.system.connect(system_address=uav.config.system_address)
        is_connected = False
        async for state in uav.system.core.connection_state():
            is_connected = state.is_connected
            break
        if not is_connected: raise ConnectionError(f"Failed to connect to UAV {uav.config.uav_id}")
        uav.telemetry.connected = True

        await self._configure_uav_parameters(uav_index)

    async def _configure_uav_parameters(self, uav_index: int):
        """Configure UAV parameters after connection."""

        px4_parameters = self.uavs[uav_index].config.px4_parameters
        if px4_parameters:
            applied_parameters = await self.uav_fn_set_params(
                uav_index=uav_index,
                parameters=px4_parameters,
            )
            print(
                f"[INFO] UAV-{uav_index}: Applied {len(applied_parameters)}/"
                f"{len(px4_parameters)} PX4 parameters from uav_config.yaml"
            )

        await self.uav_fn_overwrite_params(
            uav_index=uav_index, parameters=self.uavs[uav_index].config.overwrite_params
        )
        
        await self.uavs[uav_index].system.action.set_takeoff_altitude(
            altitude=self.uavs[uav_index].config.init_params["altitude"]
        )
        await self.uavs[uav_index].system.action.set_current_speed(3)
        
        try:
            await self.uavs[uav_index].system.param.set_param_float("RTL_RETURN_ALT", 5.0)
            print(f"[INFO] UAV-{uav_index}: Đặt độ cao RTL thành 5m thành công")
        except Exception as e:
            print(f"[ERROR] UAV-{uav_index}: Lỗi khi đặt RTL_RETURN_ALT - {e}")

        await self.uav_fn_export_params(
            uav_index=uav_index, save_path=self.config.parameter_data_files[uav_index - 1]
        )

    async def arm(self, uav_index):
        await self.get_uav(uav_index).system.action.arm()

    async def disarm(self, uav_index):
        await self.get_uav(uav_index).system.action.disarm()

    async def takeoff(self, uav_index):
        uav = self.get_uav(uav_index)
        await uav.system.action.arm()
        await uav.system.action.takeoff()

    async def land(self, uav_index):
        await self.get_uav(uav_index).system.action.land()

    async def return_to_launch(self, uav_index):
        await self.get_uav(uav_index).system.action.return_to_launch()

    async def goto_location(self, uav_index, latitude, longitude):
        uav = self.get_uav(uav_index)
        await uav.system.action.goto_location(latitude, longitude, uav.config.init_params["altitude"], 0)

    async def start_mission(self, uav_index):
        await self.get_uav(uav_index).system.mission.start_mission()

    async def pause_mission(self, uav_index):
        await self.get_uav(uav_index).system.mission.pause_mission()

    async def get_status(self, uav_index):
        """Gathers all telemetry data for a UAV in parallel."""
        uav = self.get_uav(uav_index)
        if not uav or not uav.telemetry.connected: return

        await asyncio.gather(
            self._get_position(uav), self._get_mode(uav), self._get_battery(uav),
            self._get_arm_status(uav), self._get_gps(uav)
        )

    async def _get_position(self, uav):
        async for p in uav.system.telemetry.position():
            uav.telemetry.altitude_relative_m = round(p.relative_altitude_m, 12)
            uav.telemetry.altitude_msl_m = round(p.absolute_altitude_m, 12)
            uav.telemetry.latitude = round(p.latitude_deg, 12)
            uav.telemetry.longitude = round(p.longitude_deg, 12)
            break

    async def _get_mode(self, uav):
        async for m in uav.system.telemetry.flight_mode():
            uav.telemetry.flight_mode = str(m)
            break

    async def _get_battery(self, uav):
        async for b in uav.system.telemetry.battery():
            uav.telemetry.battery_percent = f"{round(b.remaining_percent * 100, 1)}%"
            uav.telemetry.battery_voltage_v = b.voltage_v
            uav.telemetry.battery_current_a = b.current_battery_a
            uav.telemetry.battery_consumed_ah = b.capacity_consumed_ah
            break

    async def _get_arm_status(self, uav):
        async for a in uav.system.telemetry.armed():
            uav.telemetry.armed = "ARMED" if a else "DISARMED"
            break

    async def _get_gps(self, uav):
        async for g in uav.system.telemetry.gps_info():
            uav.telemetry.gps_fix_type = str(g.fix_type)
            break

    async def get_params(self, uav_index, param_list):
        uav = self.get_uav(uav_index)
        params = {}
        for p_name in param_list:
            try:
                val = await uav.system.param.get_param_float(p_name)
                params[p_name] = val
            except Exception:
                params[p_name] = 'N/A'
        return params

    async def set_params(self, uav_index, params):
        uav = self.get_uav(uav_index)
        for p_name, p_val in params.items():
            await uav.system.param.set_param_float(p_name, float(p_val))

    async def toggle_actuator(self, uav_index):
        uav = self.get_uav(uav_index)
        current_state = uav.telemetry.actuator_status
        new_state = not current_state
        if new_state:
            await uav.system.action.set_actuator(4, -1)
        else:
            await uav.system.action.set_actuator(4, 1)
        uav.telemetry.actuator_status = new_state
        await asyncio.sleep(3)

    async def uav_fn_export_params(self, uav_index, save_path) -> None:
        """Export UAV parameters to a file."""
        drone = self.get_uav(uav_index)
        if save_path is None:
            return
    
        try:
            param_plugin = drone.system.param
            params = await param_plugin.get_all_params()
    
            int_params = [(p.name, p.value) for p in params.int_params]
            float_params = [(p.name, p.value) for p in params.float_params]
            custom_params = [(p.name, p.value) for p in params.custom_params]
            
            os.makedirs(os.path.dirname(save_path), exist_ok=True)
    
            with open(save_path, "w") as wf:
                for name, value in int_params + float_params + custom_params:
                    wf.write(f"{name}\t{value}\n")
                    
        except Exception as e:
            print(f"Error exporting parameters: {repr(e)}")

    def uav_fn_import_params(self, load_path) -> dict:
        """Import UAV parameters from a tab-separated parameter file."""
        if load_path is None:
            return None
            
        if not os.path.exists(load_path):
            print(f"Parameter file not found: {load_path}")
            return {}
            
        parameters = {}
    
        try:
            with open(load_path, "r") as rf:
                for line in rf.readlines():
                    if line.startswith("#") or not line.strip():
                        continue
                        
                    columns = line.strip().split("\t")
                    if len(columns) < 3:
                        print(f"Invalid parameter line: {line.strip()}")
                        continue
                        
                    name = columns[2] if len(columns) >= 3 else columns[0]
                    value = columns[3] if len(columns) >= 4 else columns[1]
                    parameters[name] = value
                    
            return parameters
            
        except Exception as e:
            print(f"Error importing parameters: {repr(e)}")
            return {}

    async def uav_fn_get_params(self, uav_index, list_params=None) -> dict:
        """Get all UAV parameters, or only the names in list_params."""
        drone = self.get_uav(uav_index)
        parameters = {}
    
        try:
            param_plugin = drone.system.param
            params = await param_plugin.get_all_params()
    
            int_param_names = [p.name for p in params.int_params]
            float_param_names = [p.name for p in params.float_params]
            custom_param_names = [p.name for p in params.custom_params]
    
            if list_params is None:
                int_param_values = [p.value for p in params.int_params]
                float_param_values = [p.value for p in params.float_params]
                custom_param_values = [p.value for p in params.custom_params]
    
                param_names = int_param_names + float_param_names + custom_param_names
                param_values = int_param_values + float_param_values + custom_param_values
                
            else:
                param_names = list_params
                param_values = []
                
                for param_name in param_names:
                    try:
                        if param_name in int_param_names:
                            param = await param_plugin.get_param_int(param_name)
                        elif param_name in float_param_names:
                            param = await param_plugin.get_param_float(param_name)
                        elif param_name in custom_param_names:
                            param = await param_plugin.get_param_custom(param_name)
                        else:
                            print(f"Parameter not found: {param_name}")
                            param = None
                            
                        param_values.append(param)
                        
                    except Exception as e:
                        print(f"Error retrieving parameter {param_name}: {repr(e)}")
                        param_values.append(None)
    
            for i, name in enumerate(param_names):
                if i < len(param_values):
                    parameters[name] = param_values[i]
    
            return parameters
            
        except Exception as e:
            print(f"Error getting parameters: {repr(e)}")
            return parameters

    async def uav_fn_set_params(self, uav_index, parameters=None, param_file=None):
        """Set UAV parameters from a dict or a parameter file."""
        drone = self.get_uav(uav_index)
        applied_parameters = {}
        if parameters is None and param_file is None:
            print("No parameters or parameter file provided")
            return applied_parameters
    
        try:
            param_plugin = drone.system.param
            params = await param_plugin.get_all_params()
    
            int_param_names = [p.name for p in params.int_params]
            float_param_names = [p.name for p in params.float_params]
            custom_param_names = [p.name for p in params.custom_params]
    
            if parameters is None and param_file is not None:
                parameters = self.uav_fn_import_params(param_file)
                if not parameters:
                    print(f"No valid parameters found in {param_file}")
                    return applied_parameters
    
            for param_name, param_value in parameters.items():
                try:
                    if param_name in int_param_names:
                        await param_plugin.set_param_int(param_name, int(param_value))
                        applied_parameters[param_name] = int(param_value)
                        
                    elif param_name in float_param_names:
                        await param_plugin.set_param_float(param_name, float(param_value))
                        applied_parameters[param_name] = float(param_value)
                        
                    elif param_name in custom_param_names:
                        await param_plugin.set_param_custom(param_name, str(param_value))
                        applied_parameters[param_name] = str(param_value)
                        
                    else:
                        print(f"Unknown parameter: {param_name}, skipping")
                        
                except Exception as e:
                    print(f"Error setting parameter {param_name}: {repr(e)}")
                    
        except Exception as e:
            print(f"Error setting parameters: {repr(e)}")

        return applied_parameters

    async def uav_fn_overwrite_params(self, uav_index, parameters) -> None:
        """Overwrite critical UAV flight parameters."""
        drone = self.get_uav(uav_index)
        try:
            await drone.system.mission.set_return_to_launch_after_mission(
                parameters.get("RTL_AFTER_MS", False)
            )
            
            takeoff_alt = parameters.get("MIS_TAKEOFF_ALT", 10.0)
            await drone.system.action.set_takeoff_altitude(takeoff_alt)
            
            await drone.system.action.set_return_to_launch_altitude(takeoff_alt)
            
            await drone.system.action.set_current_speed(
                parameters.get("CURRENT_SPEED", 3)
            )
            
        except Exception as e:
            print(f"Error overwriting parameters: {repr(e)}")

    async def uav_fn_goto_location(self, uav_index, latitude=None, longitude=None, altitude=None) -> None:
        drone = self.get_uav(uav_index)
        target_lat, target_lon, target_alt = None, None, None
        try:
            async for position in drone.system.telemetry.position():
                target_lat = latitude if latitude is not None else position.latitude_deg
                target_lon = longitude if longitude is not None else position.longitude_deg
                target_alt = altitude if altitude is not None else position.absolute_altitude_m
                
                print(f"[DEBUG] UAV-{drone.config.uav_id} initiating GOTO to: lat={target_lat}, lon={target_lon}, alt={target_alt}m")
                await drone.system.action.set_current_speed(4)
                await drone.system.action.goto_location(target_lat, target_lon, target_alt, float("nan"))
                print(f"[DEBUG] UAV-{drone.config.uav_id} goto_location command successfully sent to MAVSDK.")
                break
                
            await self._wait_for_location_reached(uav_index, target_lat, target_lon, target_alt, timeout=60)
                
        except Exception as e:
            print(f"Error in goto_location: {repr(e)}")

    async def _wait_for_location_reached(self, uav_index, target_lat, target_lon, target_alt, 
                                        tolerance_deg=2e-5, tolerance_alt=1.0, timeout=60):
        """Wait until a UAV reaches the target location or times out."""
        drone = self.get_uav(uav_index)
        start_time = time.time()
        last_print_time = start_time
    
        async for position in drone.system.telemetry.position():
            current_lat = position.latitude_deg
            current_lon = position.longitude_deg
            current_alt = position.absolute_altitude_m             
        
            lat_reached = abs(current_lat - target_lat) < tolerance_deg
            lon_reached = abs(current_lon - target_lon) < tolerance_deg
            alt_reached = abs(current_alt - target_alt) < tolerance_alt
            
            if time.time() - last_print_time >= 2:
                print(f"[DEBUG] UAV-{drone.config.uav_id} GOTO Tracking - Lat Diff: {abs(current_lat - target_lat):.6f}, "
                      f"Lon Diff: {abs(current_lon - target_lon):.6f}, Alt Diff: {abs(current_alt - target_alt):.2f}m")
                last_print_time = time.time()
                
            if lat_reached and lon_reached and alt_reached:
                print(f"[DEBUG] UAV-{drone.config.uav_id} at {[current_lat, current_lon, current_alt]} successfully REACHED target location.")
                return True
            if time.time() - start_time >= timeout:
                print(f"[DEBUG] UAV-{drone.config.uav_id} GOTO timeout reaching location ({timeout}s)")
                return False    
        return False

    async def uav_fn_goto_distance(self, uav_index, distance, direction) -> None:
        """Move UAV a specified distance in a given direction."""
        drone = self.get_uav(uav_index)
        r_earth = 6378137  # Earth radius in meters
        
        try:
            async for position in drone.system.telemetry.position():
                initial_lat = position.latitude_deg
                initial_lon = position.longitude_deg
                initial_alt = position.absolute_altitude_m
                
                if direction == "forward":
                    lat = initial_lat + (distance / r_earth) * (180 / math.pi)
                    lon = initial_lon
                    alt = initial_alt
                    
                elif direction == "backward":
                    lat = initial_lat - (distance / r_earth) * (180 / math.pi)
                    lon = initial_lon
                    alt = initial_alt
                    
                elif direction == "left":
                    lat = initial_lat
                    lon = initial_lon - (distance / (r_earth * math.cos(math.pi * initial_lat / 180))) * (180 / math.pi)
                    alt = initial_alt
                    
                elif direction == "right":
                    lat = initial_lat
                    lon = initial_lon + (distance / (r_earth * math.cos(math.pi * initial_lat / 180))) * (180 / math.pi)
                    alt = initial_alt
                    
                elif direction == "up":
                    lat = initial_lat
                    lon = initial_lon
                    alt = initial_alt + distance
                    
                elif direction == "down":
                    lat = initial_lat
                    lon = initial_lon
                    alt = initial_alt - distance
                    
                else:
                    raise ValueError(f"Invalid direction: {direction}")
                    
                await self.uav_fn_goto_location(uav_index, lat, lon, alt)
                break
                
        except Exception as e:
            print(f"Error in goto_distance: {repr(e)}")

    async def uav_fn_offboard_set_actuator(self, uav_index, group, controls) -> None:
        """Control UAV actuators using offboard mode."""
        drone = self.get_uav(uav_index)
        nan = float("nan")
        offsets1 = [nan] * 8  # 8 control channels for group 0
        offsets2 = [nan] * 8  # 8 control channels for group 1
        
        try:
            await drone.system.action.arm()
            
            await drone.system.offboard.set_actuator_control(
                ActuatorControl([ActuatorControlGroup(offsets1), ActuatorControlGroup(offsets2)])
            )
            
            print(f"UAV-{drone.config.uav_id} starting offboard mode")
            await drone.system.offboard.start()
            
            if group == 0:
                await drone.system.offboard.set_actuator_control(
                    ActuatorControl([ActuatorControlGroup(controls), ActuatorControlGroup(offsets2)])
                )
            elif group == 1:
                await drone.system.offboard.set_actuator_control(
                    ActuatorControl([ActuatorControlGroup(offsets1), ActuatorControlGroup(controls)])
                )
            else:
                print(f"Invalid actuator group: {group}, must be 0 or 1")
                
            await asyncio.sleep(2)
            
            print(f"UAV-{drone.config.uav_id} stopping offboard mode")
            await drone.system.offboard.stop()
            
        except OffboardError as error:
            print(f"Offboard mode error: {error._result.result}")
            print("Disarming UAV")
            await drone.system.action.disarm()
            
        except Exception as e:
            print(f"Error in offboard_set_actuator: {repr(e)}")
            try:
                await drone.system.offboard.stop()
            except:
                pass

    async def uav_fn_control_gimbal(self, uav_index, control_value={"pitch": 0, "yaw": 0}) -> None:
        """Control the UAV gimbal pitch and yaw."""
        drone = self.get_uav(uav_index)
        try:
            await drone.system.gimbal.take_control(
                control_mode=ControlMode.PRIMARY
            )
            
            await drone.system.gimbal.set_mode(
                GimbalMode.YAW_FOLLOW
            )
            
            pitch = control_value.get("pitch", 0)
            yaw = control_value.get("yaw", 0)
            
            print(f"UAV-{drone.config.uav_id} setting gimbal to pitch: {pitch}°, yaw: {yaw}°")
            await drone.system.gimbal.set_pitch_and_yaw(pitch, yaw)
            
            await asyncio.sleep(3)
            await drone.system.gimbal.release_control()
            
        except Exception as e:
            print(f"Error controlling gimbal: {repr(e)}")
            try:
                await drone.system.gimbal.release_control()
            except:
                pass

    async def uav_fn_is_on_mission(self, uav_index) -> bool:
        """Check whether UAV is currently executing a mission."""
        drone = self.get_uav(uav_index)
        try:
            async for mission_progress in drone.system.mission.mission_progress():
                return mission_progress.current < mission_progress.total
        except Exception as e:
            print(f"Error checking mission status: {repr(e)}")
            return False

    async def observe_is_in_air(self, uav_index, running_tasks) -> None:
        """Cancel running tasks after the UAV lands."""
        drone = self.get_uav(uav_index)
        was_in_air = False
    
        try:
            async for is_in_air in drone.system.telemetry.in_air():
                if is_in_air:
                    was_in_air = True
                    
                if was_in_air and not is_in_air:
                    print(f"UAV-{drone.config.uav_id} has landed, canceling tasks")
                    
                    for task in running_tasks:
                        if not task.done():
                            task.cancel()
                            try:
                                await task
                            except asyncio.CancelledError:
                                pass
                                
                    return
                    
        except Exception as e:
            print(f"Error in observe_is_in_air: {repr(e)}")

    async def uav_fn_upload_mission(self, uav_index, mission_plan_file, verbose=False) -> None:
        """Upload a .plan or plain coordinate mission file to a UAV."""
        drone = self.get_uav(uav_index)
        if mission_plan_file is None:
            print("No mission plan file provided")
            return
    
        if not os.path.exists(mission_plan_file):
            raise FileNotFoundError(f"Mission plan file {mission_plan_file} not found")
    
        try:
            mission_data = []
            if Path(mission_plan_file).suffix == ".plan":
                try:
                    with open(mission_plan_file, "r") as f:
                        plan_data = json.load(f)
    
                    mission_list = plan_data.get("mission", {}).get("items", [])
                    if not mission_list:
                        print("No mission items found in .plan file")
                        return
    
                    for item in mission_list:
                        if item.get("command") != 16:
                            continue
    
                        lat = item["params"][4]
                        lon = item["params"][5]
                        alt = item["params"][6]
                        mission_data.append((lat, lon, alt))
    
                except Exception as e:
                    print(f"Error reading .plan file: {repr(e)}")
                    return
            else:
                with open(mission_plan_file, "r") as f:
                    for line in f:
                        line = line.strip()
                        if not line:
                            continue
                        try:
                            parts = line.split(",")
                            if len(parts) == 2:
                                lat, lon = map(float, parts)
                                alt = drone.config.init_params.get("altitude", 10.0)
                            elif len(parts) == 3:
                                lat, lon, alt = map(float, parts)
                            else:
                                print(f"Invalid line: {line}")
                                continue
                            mission_data.append((lat, lon, alt))
                        except ValueError:
                            print(f"Invalid coordinate format: {line}")
    
            if not mission_data:
                print("No valid mission data found")
                return
    
            mission_items = []
            for lat, lon, alt in mission_data:
                mission_items.append(
                    MissionItem(
                        latitude_deg=lat,
                        longitude_deg=lon,
                        relative_altitude_m=alt,
                        speed_m_s=2.5,
                        is_fly_through=False,
                        gimbal_pitch_deg=float("nan"),
                        gimbal_yaw_deg=float("nan"),
                        loiter_time_s=1,
                        acceptance_radius_m=float("nan"),
                        yaw_deg=float("nan"),
                        camera_action=MissionItem.CameraAction.NONE,
                        camera_photo_distance_m=float("nan"),
                        camera_photo_interval_s=float("nan"),
                        vehicle_action=MissionItem.VehicleAction.NONE,
                    )
                )
    
            mission_plan = MissionPlan(mission_items)
            if verbose:
                async for progress in drone.system.mission.upload_mission_with_progress(mission_plan):
                    print(f"Upload progress: {progress}")
            else:
                await drone.system.mission.upload_mission(mission_plan)
    
            print(f"UAV-{drone.config.uav_id} mission upload complete with {len(mission_items)} waypoints")
    
        except Exception as e:
            print(f"Error uploading mission: {repr(e)}")
            raise e

    async def _check_uav_health(self, uav_index):
        """Check if UAV is healthy enough for mission."""
        drone = self.get_uav(uav_index)
        max_checks = 10
        check_count = 0
        
        while check_count < max_checks:
            async for health in drone.system.telemetry.health():
                if health.is_global_position_ok and health.is_home_position_ok:
                    return
                    
                issues = []
                if not health.is_global_position_ok:
                    issues.append("no global position")
                if not health.is_home_position_ok:
                    issues.append("no home position")
                    
                print(f"UAV-{drone.config.uav_id} health check issues: {', '.join(issues)}")
                check_count += 1
                await asyncio.sleep(1)
                break
                
        raise RuntimeError(f"UAV-{drone.config.uav_id} failed health check after {max_checks} attempts")

    async def uav_fn_swarm_goto(self, uav_indices, txt_file_path):
        """Send one or more UAVs to the detected coordinate in a text file."""
        drones = [self.get_uav(idx) for idx in uav_indices]
    
        with open(txt_file_path, "r") as file:
            content = file.read()
            lat_detect, lon_detect = map(float, content.strip().split(", "))
    
        if len(drones) == 1:
            await self.uav_fn_goto_location(drones[0].config.uav_id, lat_detect, lon_detect)
        else:
            await asyncio.gather(
                *[self.uav_fn_goto_location(drone.config.uav_id, lat_detect, lon_detect) for drone in drones]
            )

    async def swarm_algorithm(self, uav_indices, n_swarms, txt_file_path):
        """Send the closest UAVs to the detected coordinate in a text file."""
        drones = [self.get_uav(idx) for idx in uav_indices]
    
        with open(txt_file_path, "r") as file:
            content = file.read()
            lat_detect, lon_detect = map(float, content.strip().split(", "))
    
        distances = []
        latitudes = []
        longitudes = []
    
        for drone in drones:
            async for position in drone.system.telemetry.position():
                latitudes.append(position.latitude_deg)
                longitudes.append(position.longitude_deg)
                break
    
        distances.append(calculate_distance(latitudes[-1], longitudes[-1], lat_detect, lon_detect))
    
        sorted_drones = [drone for _, drone in sorted(zip(distances, drones))]
        await self.uav_fn_swarm_goto([d.config.uav_id for d in sorted_drones[:n_swarms]], txt_file_path)


    async def print_mission_progress(self, uav_index) -> None:
        """Monitor and print mission progress."""
        drone = self.get_uav(uav_index)
        try:
            async for mission_progress in drone.system.mission.mission_progress():
                print(
                    f"Mission UAV-{drone.config.uav_id} progress: "
                    f"{mission_progress.current}/{mission_progress.total}"
                )
        except asyncio.CancelledError:
            pass

    async def uav_fn_do_mission(self, uav_index, mission_plan_file) -> None:
        """Execute a UAV mission from health check through landing."""
        drone = self.get_uav(uav_index)
        print_mission_progress_task = None
        termination_task = None
        try:
            await self._check_uav_health(uav_index)
            
            await drone.system.mission.clear_mission()
            
            print_mission_progress_task = asyncio.ensure_future(self.print_mission_progress(uav_index))
            running_tasks = [print_mission_progress_task]
            termination_task = asyncio.ensure_future(self.observe_is_in_air(uav_index, running_tasks))
            
            await self.uav_fn_upload_mission(uav_index, mission_plan_file)
            await asyncio.sleep(1)
            
            # UAV đã connect từ giao diện; connect lại ở đây dễ gây crash.
            
            await drone.system.action.arm()
            await asyncio.sleep(2)
            
            await drone.system.action.takeoff()
            await asyncio.sleep(3)
            
            await drone.system.mission.start_mission()
            await asyncio.sleep(3)
            await drone.system.action.set_current_speed(3)
            
            await termination_task
            
        except Exception as e:
            print(f"Error executing mission: {repr(e)}")
            
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
