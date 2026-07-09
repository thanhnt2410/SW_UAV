import asyncio
import cv2
from utils.qt_utils import convert_cv2qt

class TelemetryController:
    def __init__(self, app):
        self.app = app

    def set_connection_display(self, uav_index, telemetry):
        
        if telemetry.connected:
            self.app.view.uav_label_params[uav_index - 1].setStyleSheet("background-color: green")
        else:
            self.app.view.uav_label_params[uav_index - 1].setStyleSheet("background-color: red")
    
        self.app.view.uav_information_views[uav_index - 1].setText(
            self.app.view.template_information(uav_index, telemetry)
        )

    def _update_uav_info_display(self, uav_index):
        """Update the information display for a UAV."""
        
        self.app.view.uav_information_views[uav_index - 1].setText(
            self.app.view.template_information(uav_index, self.app.UAVs[uav_index].telemetry)
        )

    async def uav_fn_get_status(self, uav_index, verbose=1) -> None:
        if uav_index not in range(1, self.app.config.MAX_UAV_COUNT + 1):
            status_tasks = [
                self.uav_fn_get_status(i, verbose=verbose) # type: ignore
                for i in range(1, self.app.config.MAX_UAV_COUNT + 1)
                if self.app.UAVs[i].config.connection_allow
            ]
            await asyncio.gather(*status_tasks)
            return
        
        if not (self.app.UAVs[uav_index].telemetry.connected and self.app.UAVs[uav_index].config.connection_allow):
            return
        
        try:
            await self.app.drone_service.get_status(uav_index)
            self._update_uav_info_display(uav_index)
            await self.uav_fn_get_flight_info(uav_index, copy=False) # Keep UI-related param logic here
            if verbose:
                await self.uav_fn_print_status(uav_index)
            
        except Exception as e:
            self.app.logger.log(f"Failed to get status for UAV {uav_index}: {e}", level="error")
            self.app.UAVs[uav_index].telemetry.connected = False
            self.set_connection_display(uav_index, self.app.UAVs[uav_index].telemetry)
            self.app.view.popup_msg(
                f"Error retrieving UAV {uav_index} status: {repr(e)}", 
                src_msg="uav_fn_get_status", 
                type_msg="error"
            )

    async def uav_fn_get_position(self, uav_index) -> None:
        await self.app.drone_service.get_status(uav_index)
        uav_data = self.app.drone_service.get_uav(uav_index)
        self.app._update_position_log(uav_index, uav_data.telemetry.latitude, uav_data.telemetry.longitude, uav_data.telemetry.altitude_msl_m)
        self._update_uav_info_display(uav_index)

    async def uav_fn_get_mode(self, uav_index) -> None:
        """This logic is now in DroneService.get_status()"""
        pass

    async def uav_fn_get_battery(self, uav_index) -> None:
        """This logic is now in DroneService.get_status()"""
        pass

    async def uav_fn_get_arm_status(self, uav_index) -> None:
        """This logic is now in DroneService.get_status()"""
        pass

    async def uav_fn_get_gps(self, uav_index) -> None:
        """This logic is now in DroneService.get_status()"""
        pass

    async def uav_fn_get_flight_info(self, uav_index, copy=False) -> None:
        try:
            parameters = await self.app.drone_service.get_params(uav_index, self.app.config.interface['displayed_parameters'])
            
            for i, (param_name, value) in enumerate(parameters.items()):
                formatted_value = str(round(value, 1))
                self.app.view.uav_param_displays[uav_index - 1].children()[i + 1].setText(formatted_value)
                
                if copy:
                    self.app.view.uav_param_sets[uav_index - 1].children()[i + 1].setText(formatted_value)
                    
        except Exception as e:
            self.app.logger.log(f"Failed to get flight parameters for UAV {uav_index}: {e}", level="error")
            self.app.view.popup_msg(
                f"Error retrieving flight parameters: {repr(e)}", 
                src_msg="uav_fn_get_flight_info", 
                type_msg="error"
            )

    async def uav_fn_set_flight_info(self, uav_index) -> None:
        
        try:
            parameters = {}
            
            input_widgets = self.app.view.uav_param_sets[uav_index - 1].children()[1:-1]
            display_widgets = self.app.view.uav_param_displays[uav_index - 1].children()[1:-1]
            
            for i, (input_widget, display_widget) in enumerate(zip(input_widgets, display_widgets)):
                param_name = self.app.config.interface['displayed_parameters'][i]
                input_text = input_widget.text()
                
                if not input_text:
                    parameters[param_name] = float(display_widget.text())
                else:
                    try:
                        parameters[param_name] = float(input_text)
                    except ValueError:
                        self.app.logger.log(f"Invalid value for parameter {param_name}: {input_text}", level="warning")
                        self.app.view.popup_msg(
                            f"Invalid value for {param_name}: {input_text}", 
                            src_msg="uav_fn_set_flight_info", 
                            type_msg="Warning"
                        )
                        parameters[param_name] = float(display_widget.text())
            
            await self.app.drone_service.set_params(uav_index, parameters)
            
            await self.uav_fn_get_flight_info(uav_index=uav_index, copy=False)
            
            self.app.logger.log(f"Updated flight parameters for UAV {uav_index}", level="info")
            self.app.view.update_terminal(f"[INFO] Updated flight parameters for UAV {uav_index}")
            
        except Exception as e:
            self.app.logger.log(f"Failed to set flight parameters for UAV {uav_index}: {e}", level="error")
            self.app.view.popup_msg(
                f"Error setting flight parameters: {repr(e)}", 
                src_msg="uav_fn_set_flight_info", 
                type_msg="Error"
            )

    async def uav_fn_print_status(self, uav_index) -> None:
        if not self.app._check_uav_connection(uav_index):
            return
        
        try:
            async for status in self.app.drone_service.get_uav(uav_index).system.telemetry.status_text():
                status_text = f"> {status.type} - {status.text}"
                self.app.view.update_terminal(status_text, uav_index)
                
                if status.type.name in ["ERROR", "CRITICAL"]:
                    self.app.logger.log(f"UAV {uav_index}: {status.text}", level="error")
                elif status.type.name == "WARNING":
                    self.app.logger.log(f"UAV {uav_index}: {status.text}", level="warning")
                else:
                    self.app.logger.log(f"UAV {uav_index}: {status.text}", level="debug")
                    
        except Exception as e:
            self.app.logger.log(f"Failed to print status for UAV {uav_index}: {e}", level="error")
