import asyncio
import glob
import os
import sys
import yaml
from datetime import datetime

import cv2
import pyfiglet
from asyncqt import QEventLoop

# mavsdk
from mavsdk import System

# PyQt5
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QFileDialog

# ultralytics
from ultralytics import YOLO

# user-defined configuration
from config.interface_config import *
from config.stream_config import *
from config.uav_config import *

# user-defined interface
from interface_base import *
from interface_map import *

# user-defined utils
from utils.drone_utils import *
from utils.mavsdk_server_utils import *
from utils.qt_utils import *
from utils.serial_utils import *
from utils.stream_utils import *


# gimbal 
GIMBAL_C12_PATH = os.path.join(os.path.dirname(__file__), "GimbalC12.py")

# cspell: ignore UAVs mavsdk asyncqt figlet ndarray offboard pixmap qgroundcontrol rtcm imwrite dsize fourcc imread
__version__ = "3.20.0"
__current_time__ = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
__current_path__ = os.path.dirname(os.path.abspath(__file__))
__system_info__ = get_system_information()
print("*" * 50 + "\n" + "*" * 50)
print("SYSTEM INFO:\n" + __system_info__)
print("APP VERSION:", __version__, "\nWorking directory:", __current_path__, "\n", "*" * 50)
print(pyfiglet.figlet_format("UAV SWARM CONTROL APP"))
print("*" * 50)
print("CURRENT TIME:", __current_time__)

# Load initial UAV positions from YAML config
# Đường dẫn đến file YAML, thư mục config nằm cùng cấp với src
INIT_UAVS_POS_CONFIG_PATH = os.path.join(os.path.dirname(os.path.dirname(__file__)), "config", "init_pos_uavs.yaml")
with open(INIT_UAVS_POS_CONFIG_PATH, "r") as f:
    INITIAL_UAV_POSITIONS = yaml.safe_load(f)

# UAVs object
try:
    UAVs = {
        uav_index: {
            "ID": uav_index,
            "server": {
                "shell": MAVSDKServer(
                    id=uav_index,
                    protocol=PROTOCOLS[uav_index - 1],
                    server_host=SERVER_HOSTS[uav_index - 1],
                    port=CLIENT_PORTS[uav_index - 1],
                    bind_port=SERVER_PORTS[uav_index - 1],
                ),
                "start": False,
            },
            "system": System(mavsdk_server_address="localhost", port=CLIENT_PORTS[uav_index - 1]),
            "system_address": SYSTEMS_ADDRESSES[uav_index - 1],
            "streaming_address": DEFAULT_STREAM_VIDEO_PATHS[uav_index - 1],
            "connection_allow": connection_allows[uav_index - 1],
            "streaming_enable": streaming_enables[uav_index - 1],
            "detection_enable": detection_enables[uav_index - 1],
            "recording_enable": recording_enables[uav_index - 1],
            "init_params": { # Sử dụng tọa độ từ file YAML
                "longitude": INITIAL_UAV_POSITIONS[f"uav_{uav_index}"]["longitude"],
                "latitude": INITIAL_UAV_POSITIONS[f"uav_{uav_index}"]["latitude"],
                "altitude": INIT_ALT[uav_index - 1],
            },
            "status": {
                "connection_status": False,
                "streaming_status": False,
                "on_mission": False,
                "mission_start_time": "",
                "arming_status": "No information",
                "battery_status": "No information",
                "gps_status": "No information",
                "mode_status": "No information",
                "actuator_status": "No information",
                "altitude_status": ["No information", "No information"],
                "position_status": ["No information", "No information"],
            },
            "rescue_first_time": True,
        }
        for uav_index in range(1, MAX_UAV_COUNT + 1)
    }
except Exception as e:
    print(f"[Error]: {repr(e)}")
    sys.exit(1)

logger.log(f"Application initializing...", level="info")


class App(Map, StreamQtThread, Interface, QtWidgets.QWidget):

    def __init__(self) -> None:
        super().__init__()
        # UAVs
        self.uav_stream_threads = [None for _ in range(1, MAX_UAV_COUNT + 1)]
        self.uav_stream_frame_cnt = [0 for _ in range(1, MAX_UAV_COUNT + 1)]
        logger.log(f"Initialize detection model on {DEVICE}...", level="info")
        #
        start_time = time.time()
        self.uav_detection_models = [
            YOLO(model_uav_paths[uav_index]) for uav_index in range(1, MAX_UAV_COUNT + 1)
        ]
        logger.log(
            f"Detection models loaded successfully in {(time.time() - start_time):3f}s!",
            level="info",
        )
        #
        self.init_application()
        logger.log("Application initialized successfully", level="info")

    def _update_action_buttons_state(self, tab_index: int) -> None:
        """Update action buttons based on the selected tab"""
        global UAVs
        if tab_index in range(1, MAX_UAV_COUNT + 1):
            is_on_mission = UAVs[tab_index]["status"].get("on_mission", False)
            if is_on_mission:
                self._set_pause_button_style("Pause")
            else:
                self._set_pause_button_style("Resume")
        else:
            self._set_pause_button_style("Pause/Resume")

    def _set_pause_button_style(self, text_state: str) -> None:
        """Helper to set text and color for Pause/Resume button"""
        self.ui.btn_pause_resume.setText(text_state)
        if text_state == "Pause":
            # Nền màu Vàng/Cam khi UAV đang bay (Nhấn để Pause)
            self.ui.btn_pause_resume.setStyleSheet(
                "QPushButton{background-color: rgb(252, 175, 62);}\n"
                "QPushButton::pressed{background-color: rgb(255, 0, 0);}"
            )
        elif text_state == "Resume":
            # Nền màu Xanh lá khi UAV đang dừng (Nhấn để Resume)
            self.ui.btn_pause_resume.setStyleSheet(
                "QPushButton{background-color: rgb(138, 226, 52);}\n"
                "QPushButton::pressed{background-color: rgb(255, 0, 0);}"
            )
        else:
            # Nền màu Xanh dương mặc định cho Tab All
            self.ui.btn_pause_resume.setStyleSheet(
                "QPushButton{background-color: rgb(114, 159, 207);}\n"
                "QPushButton::pressed{background-color: rgb(255, 0, 0);}"
            )

    # ---------------------------------------------------------

    def init_application(self) -> None:
        """
        Initialize the application components and configure default settings.
        
        This method sets up the UI, connections, and configuration required for the application
        to run properly. It performs the following tasks:
        1. Sets the default view tabs
        2. Configures button click events
        3. Sets up line edit events
        4. Creates streaming threads
        5. Initializes settings and UI components
        """
        logger.log("Initializing application components...", level="info")
        
        # Set default UI views
        self._init_interface_views()
        
        # Setup event handlers
        self._init_event_handlers()
        
        # Create streaming components
        self._create_streaming_threads()
        
        # Configure settings
        self._handling_settings(mode="init")
    
    def _init_interface_views(self) -> None:
        """
        Set up the initial UI views and default tab selections.
        
        This configures the default screens and tabs that are shown when
        the application first starts.
        """
        # Set main view to the first page
        self.ui.stackedWidget.setCurrentIndex(0)
        
        # Set the tab view to the first tab
        self.ui.tabWidget.setCurrentIndex(0)
        
        # Configure initial status indicators
        self._update_status_indicators()
        
        # Mở rộng giới hạn nhập cho các ô kích thước map (mặc định Qt chỉ cho nhập đến 99)
        self.ui.spinBox_5.setMaximum(10000)
        self.ui.spinBox_6.setMaximum(10000)
        self.ui.spinBox_7.setMaximum(10000)
        self.ui.spinBox_8.setMaximum(10000)

    def _init_event_handlers(self) -> None:
        """
        Initialize all event handlers for UI components.
        
        This sets up the connections between UI elements (buttons, line edits)
        and their corresponding handler functions.
        """
        # Connect button click events
        self._button_clicked_event()
        
        # Connect line edit events
        self._line_edit_event()
        
        # Connect custom signals
        self._connect_custom_signals()
        
    def _update_status_indicators(self) -> None:
        """
        Update the status indicators for all UAVs.
        
        This updates the visual indicators showing the connection status,
        battery level, and other status information for each UAV.
        """
        # Update connection status indicators
        for uav_index in range(1, MAX_UAV_COUNT + 1):
            self.set_connection_display(uav_index, UAVs[uav_index]["status"])

    def _connect_custom_signals(self) -> None:
        """
        Connect custom Qt signals to their respective handler functions.
        
        This sets up signal connections for custom events like streaming
        updates and parameter changes.
        """
        # Add any additional signal connections here
        pass

    # ---------------------------<UI Events>---------------------------
    def _button_clicked_event(self) -> None:
        """
        Maps UI button click events to UAV control functions using async tasks.

        Connects buttons to functions for UAV operations like arming, disarming, opening/closing,
        landing, taking off, pausing missions, connecting, returning, and pushing missions.
        Also maps buttons for setting/getting UAV flight info, updating settings, and navigation.

        Buttons mapped:
        - Arm, Disarm, Open/Close, Landing, Take Off, Pause Mission, Connect, Return, Mission, Push Mission
        - Set/Get Flight Info (for each UAV)
        - Update Settings (for 'settings' and 'overview' pages)
        - Go To (for 'settings' and 'overview' pages)
        """
        # Define button mappings for main control functions
        button_mappings = {
            self.ui.btn_arm: lambda: self.uav_arm_callback(self.active_tab_index),
            self.ui.btn_disarm: lambda: self.uav_disarm_callback(self.active_tab_index),
            self.ui.btn_open_close: lambda: self.uav_toggle_open_callback(self.active_tab_index),
            self.ui.btn_landing: lambda: self.uav_land_callback(self.active_tab_index),
            self.ui.btn_take_off: lambda: self.uav_takeoff_callback(self.active_tab_index),
            self.ui.btn_pause_resume: lambda: self.uav_toggle_pause_mission_callback(self.active_tab_index),
            self.ui.btn_connect: lambda: self.uav_connect_callback(self.active_tab_index),
            self.ui.btn_rtl: lambda: self.uav_return_callback(self.active_tab_index, rtl=True),
            self.ui.btn_return: lambda: self.uav_return_callback(self.active_tab_index, rtl=False),
            self.ui.btn_mission: lambda: self.uav_mission_callback(self.active_tab_index)
        }
        
        # Connect main control buttons
        for button, callback in button_mappings.items():
            button.clicked.connect(lambda checked=False, cb=callback: asyncio.create_task(cb()))
        
        # Xử lý riêng nút Push Mission để tránh lỗi kẹt Asyncio Loop với QFileDialog
        self.ui.btn_push_mission.clicked.connect(
            lambda: self.uav_push_mission_sync_handler(self.active_tab_index)
        )

        # Connect camera toggle button (non-async)
        self.ui.btn_toggle_camera.clicked.connect(
            lambda: self.uav_toggle_camera_callback(self.active_tab_index)
        )
        
        # Connect parameter buttons for each UAV
        self._connect_parameter_buttons()
        
        # Connect settings configuration buttons
        self.ui.btn_sett_cf_nSwarms.clicked.connect(
            lambda: self._handling_settings(mode="settings")
        )
        self.ui.btn_ovv_cf_nSwarms.clicked.connect(
            lambda: self._handling_settings(mode="overview")
        )
        
        # Connect GoTo navigation buttons
        self._connect_goto_buttons()
        
        # Connect Simulation button
        self.ui.pushButton.clicked.connect(
            lambda: asyncio.create_task(self.run_simulation_callback())
        )
        
        # Đồng bộ Combobox Map Type với giao diện nhập tham số tương ứng
        self.ui.comboBox_3.currentIndexChanged.connect(self._on_map_type_changed)
        # Chạy 1 lần lúc khởi động để đồng bộ giao diện
        self._on_map_type_changed(self.ui.comboBox_3.currentIndex())
        
    def _connect_parameter_buttons(self) -> None:
        """
        Connect parameter control buttons for each UAV.
        
        This connects the set/get parameter buttons for all UAVs to
        the appropriate handler functions.
        """
        # Connect Set Parameter buttons
        for uav_index in range(1, MAX_UAV_COUNT + 1):
            idx = uav_index - 1  # Adjust for zero-based indexing
            
            # Create a closure to capture the current UAV index
            def create_set_callback(uav_idx):
                return lambda: asyncio.create_task(self.uav_fn_set_flight_info(uav_idx))
            
            def create_get_callback(uav_idx):
                return lambda: asyncio.create_task(self.uav_fn_get_flight_info(uav_idx, True))
            
            # Connect Set Parameter button
            self.uav_set_param_buttons[idx].clicked.connect(create_set_callback(uav_index))
            
            # Connect Get Parameter button
            self.uav_get_param_buttons[idx].clicked.connect(create_get_callback(uav_index))

    def _connect_goto_buttons(self) -> None:
        # GoTo button mapping for settings and overview pages
        for uav_index in range(MAX_UAV_COUNT + 1):  # 0-6 for all UAVs plus all-UAV control
            # Create closures to capture the current UAV index
            def create_goto_settings_callback(uav_idx):
                return lambda: asyncio.create_task(
                    self.uav_goto_callback(uav_index=uav_idx, page="settings")
                )
            
            def create_goto_overview_callback(uav_idx):
                return lambda: asyncio.create_task(
                    self.uav_goto_callback(uav_index=uav_idx, page="overview")
                )
            
            # Connect Settings page GoTo button
            self.uav_sett_goTo_buttons[uav_index].clicked.connect(
                create_goto_settings_callback(uav_index)
            )
            
            # Connect Overview page GoTo button
            self.uav_ovv_goTo_buttons[uav_index].clicked.connect(
                create_goto_overview_callback(uav_index)
            )

    def _line_edit_event(self) -> None:
        """
        Connect line edit events to their handler functions.
        
        This connects the returnPressed event of command input fields
        to the process_command function for each UAV.
        """
        for index in range(MAX_UAV_COUNT):
            # Create a closure to capture the current UAV index
            def create_command_callback(uav_idx):
                return lambda: asyncio.create_task(self.process_command(uav_idx))
            
            # Connect the returnPressed event to the process_command function
            self.uav_update_commands[index].returnPressed.connect(
                create_command_callback(index + 1)
            )

    def _create_streaming_threads(self, uav_indexes=None) -> None:
        """
        Create and configure video streaming threads for UAVs.
        
        This method sets up streaming threads for specified UAVs, configuring capture
        settings, recording options, and object detection. It connects each thread's
        signal to the streaming display handler.
        
        Args:
            uav_indexes (list, optional): Specific UAV indexes to configure. 
                                        If None, configures all UAVs.
        
        Returns:
            None
        """
        global UAVs
        
        try:
            # If no specific indexes provided, use all available UAVs
            uav_indexes = range(1, MAX_UAV_COUNT + 1) if uav_indexes is None else uav_indexes
            
            for uav_index in uav_indexes:
                # Skip UAVs that aren't eligible for streaming
                if not self._can_stream(uav_index):
                    continue
                    
                # Configure stream settings
                stream_config = self._create_stream_config(uav_index)
                
                # Determine detection model if enabled
                detection_model = (
                    self.uav_detection_models[uav_index - 1]
                    if UAVs[uav_index]["detection_enable"]
                    else None
                )
                
                # Create the streaming thread
                self.uav_stream_threads[uav_index - 1] = StreamQtThread(
                    uav_index=uav_index,
                    stream_config=stream_config,
                    detection_model=detection_model
                )
                
                # Log the stream configuration
                self._log_stream_creation(uav_index)
                
                # Safely connect signal to slot (disconnect first to prevent duplicate connections)
                asyncio.create_task(self._connect_stream_signal(uav_index))
                
        except Exception as e:
            logger.log(repr(e), level="error")
            self.popup_msg(
                type_msg="Error", 
                msg=repr(e), 
                src_msg="_create_streaming_threads()"
            )
            
    def _create_stream_config(self, uav_index):
        """Create stream configuration dictionary for a UAV"""
        global UAVs
        
        # Capture settings
        capture = {
            "index": uav_index,
            "address": UAVs[uav_index]["streaming_address"],
            "width": DEFAULT_STREAM_SIZE[0],
            "height": DEFAULT_STREAM_SIZE[1],
            "fps": DEFAULT_STREAM_FPS,
        }
        
        # Recording settings
        writer = {
            "index": uav_index,
            "enable": UAVs[uav_index]["recording_enable"],
            "filename": DEFAULT_STREAM_VIDEO_LOG_PATHS[uav_index - 1],
            "fourcc": FOURCC,
            "frameSize": DEFAULT_STREAM_SIZE,
        }
        
        return {
            "capture": capture,
            "writer": writer,
        }
        
    def _log_stream_creation(self, uav_index):
        """Log the creation of a streaming thread"""
        recording_path = (
            os.path.relpath(DEFAULT_STREAM_VIDEO_LOG_PATHS[uav_index - 1], __current_path__)
            if UAVs[uav_index]["recording_enable"]
            else 'None'
        )
        
        logger.log(
            f"UAV-{uav_index} stream started: \n"
            f"  -- Capture stream from {os.path.relpath(UAVs[uav_index]['streaming_address'], __current_path__)} \n"
            f"  -- Save recording to {recording_path}",
            level="info"
        )
        
        logger.log(f"UAV-{uav_index} streaming thread created!", level="info")
        
    async def _connect_stream_signal(self, uav_index):
        """Connect the streaming thread signal to the display slot"""
        try:
            # Try to disconnect any existing connection to prevent duplicates
            self.uav_stream_threads[uav_index - 1].change_image_signal.disconnect()
        except Exception:
            # Ignore errors if there was no existing connection
            pass
            
        # Connect the signal to the slot with queued connection type
        self.uav_stream_threads[uav_index - 1].change_image_signal.connect(
            self.stream_on_uav_screen,
            Qt.QueuedConnection
        )
    # //-/////////////////////////////////////////////////////////////

    def _handling_settings(self, mode="init") -> None:
        """
        Handle configuration settings for the interface.
        
        This method manages configuration settings across different modes:
        - init: Load initial settings from configuration
        - settings: Apply settings from the Settings tab
        - overview: Apply settings from the Overview tab
        
        Args:
            mode (str): The mode to handle ('init', 'settings', or 'overview')
        """
        try:
            logger.log(f"Handling settings in '{mode}' mode", level="info")
            
            # Handle checkbox states and related UAV settings
            self._handling_checkboxes(mode=mode)
            
            # Handle table data and connection settings
            self._handling_tables(mode=mode)
            
        except Exception as e:
            logger.log(f"Error handling settings in '{mode}' mode: {e}", level="error")
            self.popup_msg(
                msg=f"Error handling settings: {e}", 
                src_msg="_handling_settings", 
                type_msg="Error"
            )

    def _handling_checkboxes(self, mode="init") -> None:
        """
        Handle checkbox states and update UAV detection/streaming settings.
        
        This method synchronizes checkbox states between UI elements and UAV settings
        based on the specified mode.
        
        Args:
            mode (str): The mode to handle ('init', 'settings', or 'overview')
            
        Returns:
            None
        """
        global UAVs
        
        try:
            if mode == "init":
                # Initialize UI checkboxes based on configuration
                for i, widget in enumerate(self.sett_checkBox_detect_lists):
                    widget.setChecked(UAVs[i + 1]["detection_enable"])
                    
                for i, widget in enumerate(self.ovv_checkBox_detect_lists):
                    widget.setChecked(UAVs[i + 1]["detection_enable"])
                    
                for i, widget in enumerate(self.sett_checkBox_active_lists):
                    widget.setChecked(UAVs[i + 1]["streaming_enable"])
                    
            elif mode == "settings":
                # Update UAV settings from Settings page UI
                for i, widget in enumerate(self.sett_checkBox_detect_lists):
                    UAVs[i + 1]["detection_enable"] = widget.isChecked()
                    
                # Sync to Overview page
                for i, widget in enumerate(self.ovv_checkBox_detect_lists):
                    widget.setChecked(UAVs[i + 1]["detection_enable"])
                    
                # Update streaming settings
                for i, widget in enumerate(self.sett_checkBox_active_lists):
                    UAVs[i + 1]["streaming_enable"] = widget.isChecked()
                    
            elif mode == "overview":
                # Update UAV settings from Overview page UI
                for i, widget in enumerate(self.ovv_checkBox_detect_lists):
                    UAVs[i + 1]["detection_enable"] = widget.isChecked()
                    
                # Sync to Settings page
                for i, widget in enumerate(self.sett_checkBox_detect_lists):
                    widget.setChecked(UAVs[i + 1]["detection_enable"])
                    
            logger.log(f"Checkbox settings updated in '{mode}' mode", level="debug")
            
        except Exception as e:
            logger.log(f"Error handling checkboxes in '{mode}' mode: {e}", level="error")
            self.popup_msg(
                msg=f"Error handling checkboxes: {e}", 
                src_msg="_handling_checkboxes", 
                type_msg="Error"
            )

    def _handling_tables(self, mode="init") -> None:
        """
        Update table data and related UAV connection settings.
        
        This method handles table data for UAV connection and streaming configuration
        according to the specified mode.
        
        Args:
            mode (str): The mode to handle ('init', 'settings', or 'overview')
            
        Returns:
            None
        """
        global UAVs
        
        try:
            # Common setup for all modes
            headers = ["id", "connection_address", "streaming_address"]
            connection_allow_indexes = self._get_enabled_uav_indexes("connection")
            streaming_enabled_indexes = self._get_enabled_uav_indexes("streaming")
            
            if mode == "init":
                # Prepare initial table data from configuration
                data = {
                    headers[0]: [uav_index for uav_index in range(1, MAX_UAV_COUNT + 1)],
                    headers[1]: [
                        f"{UAVs[uav_index]['system_address']} -p {UAVs[uav_index]['system']._port}"
                        for uav_index in range(1, MAX_UAV_COUNT + 1)
                    ],
                    headers[2]: [
                        UAVs[uav_index]['streaming_address']
                        for uav_index in range(1, MAX_UAV_COUNT + 1)
                    ],
                }
                nSwarms = len(connection_allow_indexes)
                
            else:
                # Get number of swarms from appropriate UI element
                if mode == "settings":
                    nSwarms = min(
                        int(self.ui.nSwarms_sett.value()), 
                        len(connection_allow_indexes)
                    )
                    data = get_values_from_table(self.ui.table_uav_large, headers=headers)
                else:  # overview mode
                    nSwarms = min(
                        int(self.ui.nSwarms_ovv.value()), 
                        len(connection_allow_indexes)
                    )
                    data = get_values_from_table(self.ui.table_uav_small, headers=headers)
                
                # Update UAV configuration from table data
                self._update_uav_config_from_table(data, connection_allow_indexes)
            
            # Update tables with current data
            self._update_tables(
                data=data,
                connection_allow_indexes=connection_allow_indexes,
                streaming_enabled_indexes=streaming_enabled_indexes,
                nSwarms=nSwarms,
                headers=headers,
            )
            
            logger.log(f"Table settings updated in '{mode}' mode", level="debug")
            
        except Exception as e:
            logger.log(f"Error handling tables in '{mode}' mode: {e}", level="error")
            self.popup_msg(
                msg=f"Error handling tables: {e}", 
                src_msg="_handling_tables", 
                type_msg="Error"
            )

    def _get_enabled_uav_indexes(self, feature_type):
        """
        Get indexes of UAVs with a specific feature enabled.
        
        Args:
            feature_type (str): The feature to check ('connection' or 'streaming')
            
        Returns:
            list: List of UAV indexes with the specified feature enabled
        """
        global UAVs
        
        if feature_type == "connection":
            return [
                index + 1 for index in range(MAX_UAV_COUNT) 
                if UAVs[index + 1]["connection_allow"]
            ]
        elif feature_type == "streaming":
            return [
                index + 1 for index in range(MAX_UAV_COUNT) 
                if UAVs[index + 1]["streaming_enable"]
            ]
        else:
            return []

    def _update_uav_config_from_table(self, data, connection_allow_indexes):
        """
        Update UAV configuration from table data.
        
        Args:
            data (dict): Table data containing connection and streaming addresses
            connection_allow_indexes (list): Indexes of UAVs with connection allowed
            
        Returns:
            None
        """
        global UAVs
        
        # Process each UAV's settings
        for index in range(MAX_UAV_COUNT):
            uav_index = index + 1
            if uav_index in connection_allow_indexes:
                # Parse connection address into components
                conn_address = data["connection_address"][index]
                address_parts, client_port = conn_address.split("-p")
                proto, server_parts = address_parts.split(":", 1)
                server_host = server_parts.split(":", 1)[0].replace("//", "")
                bind_port = server_parts.split(":", 1)[1] if ":" in server_parts else "0"
                
                # Update MAVSDK server configuration
                UAVs[uav_index]["server"]["shell"] = MAVSDKServer(
                    id=uav_index,
                    protocol=proto,
                    server_host=server_host,
                    port=int(client_port),
                    bind_port=int(bind_port),
                )
                
                # Update connection addresses
                UAVs[uav_index]["system_address"] = f"{proto}:{server_parts}"
                UAVs[uav_index]["system"]._port = int(client_port)
                
                # Update streaming address
                UAVs[uav_index]["streaming_address"] = data["streaming_address"][index].strip()
        
        # Reset connection and streaming status after configuration change
        for uav_index in range(1, MAX_UAV_COUNT + 1):
            UAVs[uav_index]["status"]["connection_status"] = False
            UAVs[uav_index]["status"]["streaming_status"] = False
        
        # Recreate streaming threads with new configuration
        self._create_streaming_threads()
        logger.log("Updated UAV configuration from table data", level="info")

    def _update_tables(
        self, data, connection_allow_indexes, streaming_enabled_indexes, nSwarms, headers
    ) -> None:
        """
        Update UAV tables with current configuration data.
        
        Args:
            data (dict): Table data to display
            connection_allow_indexes (list): Indexes of UAVs with connection allowed
            streaming_enabled_indexes (list): Indexes of UAVs with streaming enabled
            nSwarms (int): Number of swarm UAVs to display
            headers (list): Column headers for the table
            
        Returns:
            None
        """
        # Convert to DataFrame if needed
        df = pd.DataFrame.from_dict(data) if not isinstance(data, pd.DataFrame) else data
        
        # Update large table (settings page)
        draw_table(
            table_widget=self.ui.table_uav_large,
            data=df,
            connection_allow_indexes=connection_allow_indexes[:nSwarms],
            streaming_enabled_indexes=streaming_enabled_indexes,
            headers=headers,
        )
        
        # Update small table (overview page)
        draw_table(
            table_widget=self.ui.table_uav_small,
            data=df,
            connection_allow_indexes=connection_allow_indexes[:nSwarms],
            streaming_enabled_indexes=streaming_enabled_indexes,
            headers=headers,
        )
        
        # Update nSwarms value in both settings and overview pages
        self.ui.nSwarms_sett.setValue(nSwarms)
        self.ui.nSwarms_ovv.setValue(nSwarms)
        
        logger.log(f"Updated UAV tables with {nSwarms} swarms", level="debug")

    # ////////////////////////////////////////////////////////////////

    async def process_command(self, uav_index) -> None:
        """
        Processes a command for a specific UAV based on the given index.
        Args:
            uav_index (int): The index of the UAV to process the command for.
        Returns:
            None
        Raises:
            Exception: If an error occurs during command processing.
        The function performs the following steps:
        1. Checks if the UAV is connected and allowed to receive commands.
        2. Retrieves the command text from the corresponding UAV update command input.
        3. If the command is "hold", it instructs the UAV to hold its position.
        4. If the command is a movement or gimbal control command, it parses the command and value,
           and performs the corresponding action:
           - Movement commands: "forward", "backward", "left", "right", "up", "down"
           - Gimbal control commands: "pitch", "yaw", "gimbal"
        5. Clears the input after processing the command.
        6. Displays an error message if an invalid input is encountered.
        """
        global UAVs

        try:
            text = self.uav_update_commands[uav_index - 1].text()
            if "=" not in text:
                command = text.strip().lower()

                if command == "gimbal":
                    self.open_gimbal_c12()
                else:
                    self.popup_msg(
                        f"Unknown command: {command}",
                        src_msg="process_command",
                        type_msg="Error",
                    )

                self.uav_update_commands[uav_index - 1].clear()
                return
            
            if not (
                UAVs[uav_index]["status"]["connection_status"]
                and UAVs[uav_index]["connection_allow"]
            ):
                return

            text = self.uav_update_commands[uav_index - 1].text()

            if text.lower().strip() == "hold":
                await UAVs[uav_index]["system"].action.hold()
            else:
                command, value = str(text).split("=")
                command = command.strip().lower()
                value = value.strip().lower()

                # NOTE: if command <do something more here>

                # * 1. control movement command
                if command in ["forward", "backward", "left", "right", "up", "down"]:
                    distance = float(value)
                    await uav_fn_goto_distance(
                        drone=UAVs[self.active_tab_index],
                        distance=distance,
                        direction=command,
                    )

                # * 2. control gimbal command
                if command in ["pitch", "yaw"]:
                    angle = float(value)
                    control_value = (
                        {"pitch": angle, "yaw": 0}
                        if command == "pitch"
                        else {"pitch": 0, "yaw": angle}
                    )
                    await uav_fn_control_gimbal(
                        drone=UAVs[self.active_tab_index], control_value=control_value
                    )

        except Exception as e:
            self.popup_msg(
                f"Invalid input: {repr(e)}", src_msg="process_command", type_msg="Error"
            )
    # open gimbalc12...................................................................
    def open_gimbal_c12(self):
        """Mở cửa sổ điều khiển Gimbal C12 (file GimbalC12.py cùng thư mục)."""
        try:
            subprocess.Popen([sys.executable, GIMBAL_C12_PATH])
        except Exception as e:
            self.popup_msg(
                f"Không mở được GimbalC12.py: {repr(e)}",
                src_msg="open_gimbal_c12",
                type_msg="Error",
            )
    # -----------------------< UAV buttons callback functions >-----------------------
    async def uav_connect_callback(self, uav_index) -> None:
        """
        Asynchronous callback function to handle UAV connection. Connect to a specific UAV or all UAVs.
        
        It performs several steps:
        1. Initializes the server for the UAV.
        2. Connects to the UAV system and Checks the connection status.
        3. Updates the connection status display.
        4. Overwrites and exports UAV parameters.
        5. Continuously updates the UAV status.
        If the UAV index is not within the valid range, it attempts to connect to all UAVs.
        
        Args:
            uav_index (int): The UAV to connect to (1-MAX_UAV_COUNT), or 0 for all UAVs
            
        Returns:
            None
            
        Raises:
            Exception: If there is an error during the connection process, it logs the error and displays a popup message.
        """
        global UAVs

        # Handle the case of connecting to all UAVs
        if uav_index not in range(1, MAX_UAV_COUNT + 1):
            connect_tasks = [
                self.uav_connect_callback(i) for i in range(1, MAX_UAV_COUNT + 1)
            ]
            await asyncio.gather(*connect_tasks)
            return

        # Skip if connection is not allowed
        if not UAVs[uav_index]["connection_allow"]:
            self.update_terminal(f"[INFO] Connection not allowed for UAV {uav_index}")
            return
            
        try:
            self.update_terminal(f"[INFO] Sent CONNECT command to UAV {uav_index}")
            
            # Reset connection status
            UAVs[uav_index]["status"]["connection_status"] = False
            self.set_connection_display(uav_index, UAVs[uav_index]["status"])

            # 1. Initialize server
            await self._initialize_server(uav_index)
            
            # 2. Connect to the UAV system
            await self._connect_to_system(uav_index)
            
            # 3. Update connection status display
            self.set_connection_display(uav_index, UAVs[uav_index]["status"])
            
            # 4. Configure UAV parameters
            await self._configure_uav_parameters(uav_index)
            
            # 5. Start continuous status updates
            await self.uav_fn_get_status(uav_index, verbose=True)

        except Exception as e:
            UAVs[uav_index]["status"]["connection_status"] = False
            self.set_connection_display(uav_index, UAVs[uav_index]["status"])
            logger.log(f"Connection error to UAV {uav_index}: {repr(e)}", level="error")
            self.popup_msg(
                f"Connection error to UAV {uav_index}: {repr(e)}",
                src_msg="uav_connect_callback",
                type_msg="error",
            )

    async def _initialize_server(self, uav_index):
        """Initialize the MAVSDK server for a UAV."""
        global UAVs
        
        if UAVs[uav_index]["server"]["start"]:
            UAVs[uav_index]["server"]["shell"].stop()
            UAVs[uav_index]["server"]["start"] = False
            await asyncio.sleep(1)

        UAVs[uav_index]["server"]["shell"].start()
        UAVs[uav_index]["server"]["start"] = True
        await asyncio.sleep(5)  # Allow time for server to start

    async def _connect_to_system(self, uav_index):
        """Connect to the UAV system and verify connection state."""
        global UAVs

        self.show_drones(init=False)
        
        logger.log(f"Waiting for UAV {uav_index} to connect...", level="info")
        
        # 1. Connect to the system
        await UAVs[uav_index]["system"].connect(
            system_address=UAVs[uav_index]["system_address"]
        )
        
        # 2. Check connection status
        async for state in UAVs[uav_index]["system"].core.connection_state():
            if state.is_connected:
                logger.log(f"UAV-{uav_index} -- Connected", level="info")
                self.update_terminal(f"[INFO] Received CONNECT signal from UAV {uav_index}")
                UAVs[uav_index]["status"]["connection_status"] = True
            else:
                logger.log(f"UAV-{uav_index} -- Disconnected", level="info")
                self.update_terminal(f"[INFO] Cannot receive CONNECT signal from UAV {uav_index}")
                UAVs[uav_index]["status"]["connection_status"] = False
            break

    async def _configure_uav_parameters(self, uav_index):
        """Configure UAV parameters after connection."""
        global UAVs
        
        # Overwrite parameters from configuration
        await uav_fn_overwrite_params(
            UAVs[uav_index], parameters=OVERWRITE_PARAMS[uav_index]
        )
        
        # Set additional parameters manually
        await UAVs[uav_index]["system"].action.set_takeoff_altitude(
            altitude=UAVs[uav_index]["init_params"]["altitude"]
        )
        await UAVs[uav_index]["system"].action.set_current_speed(3)
        
        try:
            await UAVs[uav_index]["system"].param.set_param_float("RTL_RETURN_ALT", 5.0)
            print(f"[INFO] UAV-{uav_index}: Đặt độ cao RTL thành 5m thành công")
        except Exception as e:
            print(f"[ERROR] UAV-{uav_index}: Lỗi khi đặt RTL_RETURN_ALT - {e}")
        # Export parameters to file
        await uav_fn_export_params(
            drone=UAVs[uav_index], save_path=parameter_data_files[uav_index - 1]
        )

    async def uav_arm_callback(self, uav_index) -> None:
        """
        Arm a specific UAV or all UAVs.
        
        This method sends an ARM command to the specified UAV(s), waits for
        confirmation, and updates the arming status in the UI.
        
        Args:
            uav_index (int): The UAV to arm (1-MAX_UAV_COUNT), or 0 for all available UAVs
            
        Returns:
            None
        """
        global UAVs
        
        # Handle the case of arming all available UAVs
        if uav_index not in range(1, MAX_UAV_COUNT + 1):
            arm_tasks = [
                self.uav_arm_callback(i) for i in AVAIL_UAV_INDEXES
            ]
            await asyncio.gather(*arm_tasks)
            return
        
        # Skip if UAV is not connected or not allowed
        if not self._check_uav_connection(uav_index):
            return
            
        try:
            self.update_terminal(f"[INFO] Sent ARM command to UAV {uav_index}")
            
            # Ensure connection is established
            await UAVs[uav_index]["system"].connect(
                system_address=UAVs[uav_index]["system_address"]
            )
            
            # Send arm command
            await UAVs[uav_index]["system"].action.arm()
            await asyncio.sleep(3)
            
            # Temporarily disarm (may be application-specific behavior)
            await self.uav_disarm_callback(uav_index)
            
            # Update status
            UAVs[uav_index]["status"]["arming_status"] = "ARMED"
            self._update_uav_info_display(uav_index)
            
        except Exception as e:
            UAVs[uav_index]["status"]["arming_status"] = "DISARMED"
            self._update_uav_info_display(uav_index)
            
            # Lấy thông báo lỗi chi tiết thay vì dùng repr(e)
            error_detail = str(e)
            if hasattr(e, '_result'):
                error_detail = f"{e._result.result_str} (Code: {e._result.result})"
                
            logger.log(f"Arming error: {error_detail}", level="error")
            self.popup_msg(f"Error: {error_detail}", src_msg="uav_arm_callback", type_msg="Error")

    async def uav_disarm_callback(self, uav_index) -> None:
        """
        Disarm a specific UAV or all UAVs.
        
        This method sends a DISARM command to the specified UAV(s) and updates
        the arming status in the UI.
        
        Args:
            uav_index (int): The UAV to disarm (1-MAX_UAV_COUNT), or 0 for all UAVs
            
        Returns:
            None
        """
        global UAVs
        
        # Handle the case of disarming all UAVs
        if uav_index not in range(1, MAX_UAV_COUNT + 1):
            disarm_tasks = [
                self.uav_disarm_callback(i) for i in range(1, MAX_UAV_COUNT + 1)
            ]
            await asyncio.gather(*disarm_tasks)
            return
        
        # Skip if UAV is not connected or not allowed
        if not self._check_uav_connection(uav_index):
            return
            
        try:
            self.update_terminal(f"[INFO] Sent DISARM command to UAV {uav_index}")
            
            # Send disarm command
            await UAVs[uav_index]["system"].action.disarm()
            
            # Update status
            UAVs[uav_index]["status"]["arming_status"] = "DISARMED"
            self._update_uav_info_display(uav_index)
            
        except Exception as e:
            logger.log(f"Disarming error: {repr(e)}", level="error")
            self.popup_msg(
                f"Error: {repr(e)}", src_msg="uav_disarm_callback", type_msg="Error"
            )

    async def uav_takeoff_callback(self, uav_index) -> None:
        """
        Initiate takeoff for a specific UAV or all UAVs.
        
        This method sends a TAKEOFF command to the specified UAV(s), arms the UAV,
        initiates takeoff, and updates the status in the UI.
        
        Args:
            uav_index (int): The UAV to command takeoff (1-MAX_UAV_COUNT), or 0 for all available UAVs
            
        Returns:
            None
        """
        global UAVs
        
        # Handle the case of taking off all available UAVs
        if uav_index not in range(1, MAX_UAV_COUNT + 1):
            takeoff_tasks = [
                self.uav_takeoff_callback(i) for i in AVAIL_UAV_INDEXES
            ]
            await asyncio.gather(*takeoff_tasks)
            return
        
        # Skip if UAV is not connected or not allowed
        if not self._check_uav_connection(uav_index):
            return
            
        try:
            self.update_terminal(f"[INFO] Sent TAKEOFF command to UAV {uav_index}")
            
            # Ensure connection is established
            await UAVs[uav_index]["system"].connect(
                system_address=UAVs[uav_index]["system_address"]
            )
            
            # Arm and take off
            await UAVs[uav_index]["system"].action.arm()
            await UAVs[uav_index]["system"].action.takeoff()
            
            # Save initial position information
            await self._save_initial_position(uav_index)
            
            # Update status
            UAVs[uav_index]["status"]["connection_status"] = True
            UAVs[uav_index]["status"]["mode_status"] = "TAKING OFF"
            self._update_uav_info_display(uav_index)
            
        except Exception as e:
            logger.log(f"Takeoff error: {repr(e)}", level="error")
            self.popup_msg(
                f"Error: {repr(e)}", src_msg="uav_takeoff_callback", type_msg="Error"
            )

    async def _save_initial_position(self, uav_index):
        """Save the initial position of a UAV to a file."""
        # Update initial position from current position
        UAVs[uav_index]["init_params"]["latitude"] = round(
            UAVs[uav_index]["status"]["position_status"][0], 12
        )
        UAVs[uav_index]["init_params"]["longitude"] = round(
            UAVs[uav_index]["status"]["position_status"][1], 12
        )
        
        # Save to YAML file
        yaml_file = INIT_UAVS_POS_CONFIG_PATH # Sử dụng đường dẫn đã định nghĩa ở trên
        try:
            data = {}
            if os.path.exists(yaml_file):
                with open(yaml_file, "r") as f:
                    data = yaml.safe_load(f) or {}
            
            uav_key = f"uav_{uav_index}"
            if uav_key not in data:
                data[uav_key] = {}
                
            data[uav_key]["latitude"] = UAVs[uav_index]["init_params"]["latitude"]
            data[uav_key]["longitude"] = UAVs[uav_index]["init_params"]["longitude"]
            
            with open(yaml_file, "w") as f:
                yaml.dump(data, f, default_flow_style=False, sort_keys=False)
        except Exception as e:
            logger.log(f"Failed to save initial position to YAML: {e}", level="error")

    async def uav_land_callback(self, uav_index) -> None:
        """
        Command a specific UAV or all UAVs to land.
        
        This method sends a LANDING command to the specified UAV(s) and updates
        the mode status in the UI.
        
        Args:
            uav_index (int): The UAV to command landing (1-MAX_UAV_COUNT), or 0 for all UAVs
            
        Returns:
            None
        """
        global UAVs
        
        # Handle the case of landing all UAVs
        if uav_index not in range(1, MAX_UAV_COUNT + 1):
            landing_tasks = [
                self.uav_land_callback(i) for i in range(1, MAX_UAV_COUNT + 1)
            ]
            await asyncio.gather(*landing_tasks)
            return
        
        # Skip if UAV is not connected or not allowed
        if not self._check_uav_connection(uav_index):
            return
            
        try:
            self.update_terminal(f"[INFO] Sent LANDING command to UAV {uav_index}")
            
            # Send land command
            await UAVs[uav_index]["system"].action.land()
            
            # Update status
            UAVs[uav_index]["status"]["mode_status"] = "LANDING"
            self._update_uav_info_display(uav_index)
            
        except Exception as e:
            logger.log(f"Landing error: {repr(e)}", level="error")
            self.popup_msg(f"Error: {repr(e)}", src_msg="uav_land_callback", type_msg="Error")

    async def uav_return_callback(self, uav_index, rtl=False) -> None:
        """
        Command a specific UAV or all UAVs to return.
        
        This method sends either a Return-To-Launch (RTL) command or a return to
        initial position command to the specified UAV(s) and updates the status in the UI.
        
        Args:
            uav_index (int): The UAV to command return (1-MAX_UAV_COUNT), or 0 for all available UAVs
            rtl (bool): If True, use RTL mode (land at return point), otherwise just return to position
            
        Returns:
            None
        """
        global UAVs
        
        # Handle the case of returning all available UAVs
        if uav_index not in range(1, MAX_UAV_COUNT + 1):
            return_tasks = [
                self.uav_return_callback(i, rtl=rtl) for i in range(1, MAX_UAV_COUNT + 1)
            ]
            await asyncio.gather(*return_tasks)
            return
        
        # Skip if UAV is not connected or not allowed
        if not self._check_uav_connection(uav_index):
            return
            
        try:
            # Get position information
            init_latitude = UAVs[uav_index]["init_params"]["latitude"]
            init_longitude = UAVs[uav_index]["init_params"]["longitude"]
            current_latitude = UAVs[uav_index]["status"]["position_status"][0]
            current_longitude = UAVs[uav_index]["status"]["position_status"][1]
            
            if rtl:
                # Return to launch (return and land)
                self.update_terminal(f"[INFO] Sent RTL command to UAV {uav_index}")
                
                # If already at initial position, just land
                if (init_latitude, init_longitude) == (current_latitude, current_longitude):
                    self.update_terminal(
                        f"[INFO] UAV {uav_index} is already at the initial position, landing..."
                    )
                    await UAVs[uav_index]["system"].action.land()
                else:
                    await UAVs[uav_index]["system"].action.return_to_launch()
                
                # Update status
                UAVs[uav_index]["status"]["mode_status"] = "RTL"
            else:
                # Return to initial position without landing
                self.update_terminal(
                    f"[INFO] Sent RETURN command to UAV {uav_index} to lat: {init_latitude} long: {init_longitude}"
                )
                
                # Go to the initial position
                await uav_fn_goto_location(
                    drone=UAVs[uav_index],
                    latitude=init_latitude,
                    longitude=init_longitude,
                )
                
                # Update status
                UAVs[uav_index]["status"]["mode_status"] = "RETURN"
            
            # Update display
            self._update_uav_info_display(uav_index)
            
            # Clean up mission logs
            clear_mission_logs(uav_index, save_dir=__current_path__)
            
        except Exception as e:
            logger.log(f"Return error: {repr(e)}", level="error")
            self.popup_msg(
                f"Error: {repr(e)}", src_msg="uav_return_callback", type_msg="Error"
            )

    async def uav_mission_callback(self, uav_index) -> None:
        """NOTE: convert file points to .plan file as in ./data/mission.plan
        Executes a mission for a specified UAV or all UAVs if uav_index is 0.

        Args:
            uav_index (int): The index of the UAV to execute the mission for. If 0, the mission is executed for all UAVs.

        Returns:
            None

        Raises:
            Exception: If there is an error during the mission execution.

        The function performs the following steps:
        1. Checks if the UAV is connected.
        2. Reads mission points from a file and creates mission items.
        3. Uploads the mission to the UAV.
        4. Arms the UAV and starts the mission.
        5. Monitors mission progress and initiates return to launch upon mission completion.
        6. Updates the UAV's mode status and information view.
        7. If uav_index is 0, executes the mission for all UAVs concurrently.
        """
        global UAVs
        
        # Handle the case of missions for all UAVs
        if uav_index not in range(1, MAX_UAV_COUNT + 1):
            mission_tasks = [
                self.uav_mission_callback(i) for i in range(1, MAX_UAV_COUNT + 1)
            ]
            await asyncio.gather(*mission_tasks)
            return
        
        # Handle regular UAV mission
        if uav_index in AVAIL_UAV_INDEXES:
            # await self._execute_standard_mission(uav_index)
            await asyncio.gather(
                self._execute_standard_mission(uav_index),
                # self.uav_fn_get_position(uav_index),
            )
        
        # Handle rescue UAV mission
        elif uav_index == RESCUE_UAV_INDEX:
            # Check if this is first time or if rescue tab is selected
            # if not (UAVs[RESCUE_UAV_INDEX]["rescue_first_time"] or 
            #         (self.ui.tabWidget.currentIndex() == RESCUE_UAV_INDEX)):
            #     return
            
            await self.uav_fn_rescue()
            
    async def _execute_standard_mission(self, uav_index, plan_file=None):
        """Execute a standard mission for a regular UAV."""
        # Skip if UAV is not connected or not allowed
        if not self._check_uav_connection(uav_index):
            return
            
        try:
            # Health check before mission
            self.update_terminal(
                "Waiting for drone to have a global position estimate...", uav_index=uav_index
            )
            logger.log(f"UAV-{uav_index} -- Global position for estimate OK", level="info")
            
            # Clear detection log files
            detection_log_files = glob.glob(f"{__current_path__}/logs/rescue_pos/*.log")
            for f in detection_log_files:
                os.remove(f)
            
            # Start new mission
            self.update_terminal(f"[INFO] Sent MISSION command to UAV {uav_index}")
            UAVs[uav_index]["status"]["on_mission"] = True
            UAVs[uav_index]["status"]["mission_start_time"] = datetime.now().strftime("%Y%m%d_%H%M%S")
            if self.active_tab_index == uav_index:
                self._set_pause_button_style("Pause")
            
            if plan_file is None:
                plan_file = f"{__current_path__}/logs/points/reduced_point{uav_index}.plan"
                
            self.update_terminal(f"[INFO] Bắt đầu nạp {plan_file} và tiến hành cất cánh...", uav_index=0)
            progress_task = asyncio.create_task(self.monitor_mission_progress(uav_index))
                
            await uav_fn_do_mission(
                drone=UAVs[uav_index],
                mission_plan_file=plan_file,
            )
            
            if not progress_task.done():
                progress_task.cancel()
            # Update display
            self._update_uav_info_display(uav_index)
            
            # Check if mission is finished and initiate return if needed
            if await UAVs[uav_index]["system"].mission.is_mission_finished():
                await UAVs[uav_index]["system"].action.return_to_launch()
                clear_mission_logs(uav_index, save_dir=__current_path__)
                
            UAVs[uav_index]["status"]["on_mission"] = False
            if self.active_tab_index == uav_index:
                self._set_pause_button_style("Resume")
                
        except Exception as e:
            logger.log(f"Mission error: {repr(e)}", level="error")
            self.popup_msg(
                f"Error: {repr(e)}", src_msg="uav_mission_callback", type_msg="Error"
            )
            
    def uav_push_mission_sync_handler(self, uav_index) -> None:
        """
        Synchronous handler for Push Mission to prevent QFileDialog from blocking the asyncio event loop.
        """
        global UAVs
        
        print(f"\n[DEBUG 1] Button Push Mission clicked! uav_index = {uav_index}")

        # Handle pushing mission to ALL UAVs
        if uav_index not in range(1, MAX_UAV_COUNT + 1):
            mission_file = QFileDialog.getOpenFileName(
                parent=self,
                caption="Select Mission File for ALL UAVs",
                directory=str(__current_path__),
                filter="Mission Files (*.plan *.txt *.TXT);;All Files (*)",
            )[0]
            
            if not mission_file:
                return
                
            push_tasks = []
            for i in AVAIL_UAV_INDEXES:
                if self._check_uav_connection(i):
                    self.update_terminal(f"[INFO] Sent PUSH MISSION command to UAV {i}")
                    push_tasks.append(
                        uav_fn_upload_mission(drone=UAVs[i], mission_plan_file=mission_file)
                    )
                    UAVs[i]["status"]["mode_status"] = "Mission uploaded"
                    self._update_uav_info_display(i)
                    
            if push_tasks:
                asyncio.create_task(asyncio.gather(*push_tasks))
            else:
                self.popup_msg("No connected UAVs to push mission to.", src_msg="Push Mission", type_msg="Warning")
            return
            
        # Handle pushing mission to SINGLE UAV
        if not self._check_uav_connection(uav_index):
            self.popup_msg(f"Please connect UAV {uav_index} first!", src_msg="Push Mission", type_msg="Warning")
            return

        if not os.path.exists(plans_log_dir):
            os.makedirs(plans_log_dir, exist_ok=True)

        # Safely open file dialog outside of async context
        mission_file = QFileDialog.getOpenFileName(
            parent=self,
            caption="Select Mission File",
            directory=str(__current_path__),
            filter="Mission Files (*.plan *.txt *.TXT);;All Files (*)",
        )[0]
        
        if not mission_file:
            return
            
        # Spawn async task for MAVSDK upload
        asyncio.create_task(self._async_push_mission(uav_index, mission_file))

    async def _async_push_mission(self, uav_index, mission_file):
        try:
            self.update_terminal(f"[INFO] Uploading mission from {mission_file} to UAV {uav_index}")
            await uav_fn_upload_mission(drone=UAVs[uav_index], mission_plan_file=mission_file)
            
            # 2. Ép con trỏ waypoint bắt đầu từ điểm xuất phát (index 0)
            await UAVs[uav_index]["system"].mission.set_current_mission_item(0)

            # Update status
            UAVs[uav_index]["status"]["mode_status"] = "Mission uploaded"
            self._update_uav_info_display(uav_index)
            
        except Exception as e:
            logger.log(f"Mission push error: {repr(e)}", level="error")
            self.popup_msg(f"Error pushing mission: {repr(e)}", src_msg="Push Mission", type_msg="Error")

    async def uav_toggle_pause_mission_callback(self, uav_index) -> None:
        """
        Toggle pause/resume mission for a specific UAV or all UAVs.
        
        This method pauses an ongoing mission or resumes a paused mission
        for the specified UAV(s).
        
        Args:
            uav_index (int): The UAV to toggle mission state (1-MAX_UAV_COUNT),
                            or 0 for all UAVs
            
        Returns:
            None
        """

        global UAVs
        
        # Handle the case of toggling all UAVs
        if uav_index not in range(1, MAX_UAV_COUNT + 1):
            toggle_tasks = [
                self.uav_toggle_pause_mission_callback(i) for i in range(1, MAX_UAV_COUNT + 1)
            ]
            await asyncio.gather(*toggle_tasks)
            return
        
        # Skip if UAV is not connected or not allowed
        if not self._check_uav_connection(uav_index):
            return
            
        try:
            # Determine current mission state and toggle it
            is_on_mission = UAVs[uav_index]["status"]["on_mission"]
            
            if is_on_mission:
                # Pause the mission
                self.update_terminal(f"[INFO] Sent PAUSE MISSION command to UAV {uav_index}")
                await UAVs[uav_index]["system"].mission.pause_mission()
                UAVs[uav_index]["status"]["on_mission"] = False
                UAVs[uav_index]["status"]["mode_status"] = "Mission paused"
                if self.active_tab_index == uav_index:
                    self._set_pause_button_style("Resume")
            else:
                # Resume the mission
                self.update_terminal(f"[INFO] Sent RESUME MISSION command to UAV {uav_index}")
                await UAVs[uav_index]["system"].mission.start_mission()
                UAVs[uav_index]["status"]["on_mission"] = True
                UAVs[uav_index]["status"]["mode_status"] = "Mission active"
                if self.active_tab_index == uav_index:
                    self._set_pause_button_style("Pause")
            
            # Update display
            self._update_uav_info_display(uav_index)
            
        except Exception as e:
            logger.log(f"Mission toggle error: {repr(e)}", level="error")
            self.popup_msg(
                f"Error toggling mission: {repr(e)}",
                src_msg="uav_toggle_pause_mission_callback",
                type_msg="Error"
            )

    async def uav_toggle_open_callback(self, uav_index) -> None:
        """
        Toggle actuator open/close state for a specific UAV or all UAVs.
        
        This method toggles the state of the actuator (e.g., payload, gripper)
        for the specified UAV(s) by controlling the gimbal pitch.
        
        Args:
            uav_index (int): The UAV to toggle actuator (1-MAX_UAV_COUNT),
                            or 0 for all UAVs
            
        Returns:
            None
        """
        global UAVs
        
        # Handle the case of toggling all UAVs
        if uav_index not in range(1, MAX_UAV_COUNT + 1):
            toggle_tasks = [
                self.uav_toggle_open_callback(i) for i in range(1, MAX_UAV_COUNT + 1)
                if UAVs[i]["connection_allow"]
            ]
            await asyncio.gather(*toggle_tasks)
            return
        
        # Skip if UAV is not connected or not allowed
        if not self._check_uav_connection(uav_index):
            return
            
        try:
            # Determine current actuator state
            current_state = UAVs[uav_index]["status"]["actuator_status"]
            new_state = not current_state # Toggle state
            
            # Command based on the new state
            if new_state:
                # ======== Replace with the actual actuator control function ========
                # Open actuator (gimbal down)
                self.update_terminal(f"[INFO] Sent OPEN command to UAV {uav_index}")
                # await uav_fn_control_gimbal(
                #     drone=UAVs[uav_index], control_value={"pitch": -90, "yaw": 0}
                # )
                self.update_terminal(f"[INFO] Sent CLOSE command to UAV {uav_index}")
                await UAVs[uav_index]["system"].action.set_actuator(4, -1)
                await asyncio.sleep(3)
                # ====================================================================
            else:
                # ======== Replace with the actual actuator control function ========
                # Close actuator (gimbal up)
                self.update_terminal(f"[INFO] Sent CLOSE command to UAV {uav_index}")
                await UAVs[uav_index]["system"].action.set_actuator(4, 1)
                await asyncio.sleep(3)
                # ====================================================================
            
            # Update status
            UAVs[uav_index]["status"]["actuator_status"] = new_state
            self._update_uav_info_display(uav_index)
            
        except Exception as e:
            logger.log(f"Actuator toggle error: {repr(e)}", level="error")
            self.popup_msg(
                f"Error toggling actuator: {repr(e)}",
                src_msg="uav_toggle_open_callback",
                type_msg="Error"
            )

    def uav_toggle_camera_callback(self, uav_index) -> None:
        """
        Toggle camera streaming for a specific UAV or all UAVs.
        
        This method starts or stops the video streaming thread for the 
        specified UAV(s).
        
        Args:
            uav_index (int): The UAV to toggle camera (1-MAX_UAV_COUNT),
                            or 0 for all UAVs
            
        Returns:
            None
        """
        global UAVs
        
        # Handle the case of toggling all UAVs
        if uav_index not in range(1, MAX_UAV_COUNT + 1):
            for i in range(1, MAX_UAV_COUNT + 1):
                # Only toggle UAVs that are eligible for streaming
                if self._can_stream(i):
                    self.uav_toggle_camera_callback(i)
            return
        
        # Skip if UAV is not eligible for streaming
        if not self._can_stream(uav_index):
            logger.log(
                f"Camera toggle skipped for UAV {uav_index}: not eligible for streaming",
                level="warning"
            )
            return
            
        try:
            # Determine current streaming state
            is_streaming = UAVs[uav_index]["status"]["streaming_status"]
            
            if not is_streaming:
                # Start streaming
                if self.uav_stream_threads[uav_index - 1] is None:
                    self._create_streaming_threads(uav_indexes=[uav_index])
                    
                self.uav_stream_threads[uav_index - 1].start()
                UAVs[uav_index]["status"]["streaming_status"] = True
                
                logger.log(f"UAV-{uav_index} streaming started", level="info")
                self.ui.btn_toggle_camera.setStyleSheet("background-color: green")
            else:
                # Stop streaming
                self.uav_stream_threads[uav_index - 1].stop()
                UAVs[uav_index]["status"]["streaming_status"] = False
                
                logger.log(f"UAV-{uav_index} streaming stopped", level="info")
                self.ui.btn_toggle_camera.setStyleSheet("background-color: red")
            
            # Update thread status
            self.uav_stream_threads[uav_index - 1].isRunning = UAVs[uav_index]["status"][
                "streaming_status"
            ]
            
        except Exception as e:
            logger.log(f"Camera toggle error: {repr(e)}", level="error")
            self.popup_msg(
                f"Error toggling camera: {repr(e)}", 
                src_msg="uav_toggle_camera_callback",
                type_msg="Error"
            )

    async def uav_goto_callback(self, uav_index, page="settings", *args) -> None:
        global UAVs
        
        try:
            # Get coordinates from the appropriate page
            longitude, latitude = self._get_coordinates_from_page(page, uav_index)
            # Ensure coordinates are valid
            if longitude is None or latitude is None:
                logger.log("Invalid coordinates for goto command", level="warning")
                self.popup_msg(
                    "Invalid coordinates for goto command",
                    src_msg="uav_goto_callback",
                    type_msg="Warning"
                )
                return
                
            # Sync coordinates between settings and overview pages
            self._sync_coordinates_between_pages(longitude, latitude)
            
            # Execute goto command for specific UAV or all UAVs
            if uav_index in range(1, MAX_UAV_COUNT + 1):
                # Skip if UAV is not connected or not allowed
                if not self._check_uav_connection(uav_index):
                    return
                # Send the command
                self.update_terminal(
                    f"[INFO] Sent GOTO command to UAV {uav_index}: lat={latitude}, lon={longitude}")
                await uav_fn_goto_location(
                    drone=UAVs[uav_index], latitude=latitude, longitude=longitude)
                # Update status
                UAVs[uav_index]["status"]["mode_status"] = "Going to position"
                self._update_uav_info_display(uav_index)
            else:
                # Command all UAVs to go to the same position
                goto_tasks = []
                for i in range(1, MAX_UAV_COUNT + 1):
                    if self._check_uav_connection(i):
                        self.update_terminal(
                            f"[INFO] Sent GOTO command to UAV {i}: lat={latitude}, lon={longitude}"
                        )
                        goto_tasks.append(
                            uav_fn_goto_location(
                                drone=UAVs[i], latitude=latitude, longitude=longitude
                            )
                        )
                        
                        # Update status
                        UAVs[i]["status"]["mode_status"] = "Going to position"
                        self._update_uav_info_display(i)
                        
                if goto_tasks:
                    await asyncio.gather(*goto_tasks)
                
        except Exception as e:
            logger.log(f"Goto error: {repr(e)}", level="error")
            self.popup_msg(
                f"Error in goto command: {repr(e)}",
                src_msg="uav_goto_callback",
                type_msg="Error"
            )

    def _get_coordinates_from_page(self, page, uav_index):
        """Get coordinates from the specified page with fallback to defaults."""
        global UAVs
        
        # Set default coordinates (offset by UAV index to avoid collisions)
        default_longitude = UAVs[uav_index]["init_params"]["longitude"] if uav_index > 0 else INIT_LON
        default_latitude = UAVs[uav_index]["init_params"]["latitude"] if uav_index > 0 else INIT_LAT
        
        # Get coordinates from the specified page
        if page == "settings":
            longitude_text = self.ui.lineEdit_sett_longitude.text().strip()
            latitude_text = self.ui.lineEdit_sett_latitude.text().strip()
        else:  # overview page
            longitude_text = self.ui.lineEdit_ovv_longitude.text().strip()
            latitude_text = self.ui.lineEdit_ovv_latitude.text().strip()
        
        # Parse coordinates with fallback to defaults
        try:
            longitude = float(longitude_text) if longitude_text else default_longitude
            latitude = float(latitude_text) if latitude_text else default_latitude
            return longitude, latitude
        except ValueError:
            logger.log(f"Invalid coordinate format: lon={longitude_text}, lat={latitude_text}", level="error")
            return None, None

    def _sync_coordinates_between_pages(self, longitude, latitude):
        """Synchronize coordinates between settings and overview pages."""
        # Format coordinates to ensure consistent display
        lon_str = f"{longitude:.8f}"
        lat_str = f"{latitude:.8f}"
        
        # Update both pages to maintain consistency
        self.ui.lineEdit_ovv_longitude.setText(lon_str)
        self.ui.lineEdit_ovv_latitude.setText(lat_str)
        self.ui.lineEdit_sett_longitude.setText(lon_str)
        self.ui.lineEdit_sett_latitude.setText(lat_str)

    def _update_uav_info_display(self, uav_index):
        """Update the information display for a UAV."""
        global UAVs
        
        self.uav_information_views[uav_index - 1].setText(
            self.template_information(uav_index, **UAVs[uav_index]["status"])
        )
        
    def set_connection_display(self, uav_index, uav_status):
        """
        Updates the connection status of a UAV in the UI.

        Args:
            uav_index (int): The index of the UAV to update.
            status (bool): The connection status of the UAV.

        Returns:
            None
        """
        global UAVs
        
        if uav_status["connection_status"]:
            self.uav_label_params[uav_index - 1].setStyleSheet("background-color: green")
        else:
            self.uav_label_params[uav_index - 1].setStyleSheet("background-color: red")

        self.uav_information_views[uav_index - 1].setText(
            self.template_information(uav_index, **uav_status)
        )

    # --------------------------<UAVs get status functions>-----------------------------
    async def uav_fn_get_status(self, uav_index, verbose=1) -> None:
        """
        Retrieve and update all status information for a UAV or all UAVs.
        
        This function fetches and updates position, mode, battery, arm status, GPS info,
        and flight parameters for the specified UAV. It can also handle retrieving status
        for all UAVs when uav_index is out of range.
        
        Args:
            uav_index (int): The UAV to get status for (1-MAX_UAV_COUNT), or out of range for all UAVs
            verbose (int): If 1, also display status text messages in the terminal
            
        Returns:
            None
        """
        global UAVs
        
        # Handle getting status for all UAVs
        if uav_index not in range(1, MAX_UAV_COUNT + 1):
            status_tasks = [
                self.uav_fn_get_status(i, verbose=verbose)
                for i in range(1, MAX_UAV_COUNT + 1)
                if UAVs[i]["connection_allow"]
            ]
            await asyncio.gather(*status_tasks)
            return
        
        # Skip if UAV is not connected and not allowed
        if not (UAVs[uav_index]["status"]["connection_status"] and UAVs[uav_index]["connection_allow"]):
            return
        
        try:
            # Create a list of status retrieval functions to run concurrently
            status_tasks = [
                self.uav_fn_get_position(uav_index),
                self.uav_fn_get_mode(uav_index),
                self.uav_fn_get_battery(uav_index),
                self.uav_fn_get_arm_status(uav_index),
                self.uav_fn_get_gps(uav_index),
                self.uav_fn_get_flight_info(uav_index, copy=False),
            ]
            
            # Add status text messages if verbose mode is enabled
            if verbose:
                status_tasks.append(self.uav_fn_print_status(uav_index))
            
            # Run all status tasks concurrently
            await asyncio.gather(*status_tasks)
            
        except Exception as e:
            logger.log(f"Failed to get status for UAV {uav_index}: {e}", level="error")
            UAVs[uav_index]["status"]["connection_status"] = False
            self.set_connection_display(uav_index, UAVs[uav_index]["status"])
            self.popup_msg(
                f"Error retrieving UAV {uav_index} status: {repr(e)}", 
                src_msg="uav_fn_get_status", 
                type_msg="error"
            )

    #SEND COORDINATE ham gui tin nhan
    async def send_coordinate(self) -> None:
        # Ket noi cong com
        port = "/dev/ttyUSB0"
        baudrate = 9600
        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=1)
            time.sleep(2)
            self.ui.mainTerminal.appendPlainText("Connected to " + port)


            # Gui tin nhan


            phone_number = "0972368553"
            script_dir = os.path.dirname(os.path.abspath(__file__))
            log_file_path = os.path.join(script_dir, "logs", "detected_pos", "detection_pos_uav_1.log")


            try:
                with open(log_file_path, "r", encoding="utf-8") as file:
                    message = file.readline().strip()
            except FileNotFoundError:
                self.ui.mainTerminal.appendPlainText("Error: File not found")
                return
            self.serial_port.write("AT+CMGF=1\r\n".encode())
            time.sleep(1)
            self.serial_port.write(f"AT+CMGS=\"{phone_number}\"\r\n".encode())
            time.sleep(1)
            self.serial_port.write((message + "\x1A").encode())
            time.sleep(3)
            response = self.serial_port.read_all().decode(errors='ignore')
            self.ui.mainTerminal.appendPlainText("Response: " + response)
        except serial.SerialException as e:
            self.ui.mainTerminal.appendPlainText("Error: " + str(e))

    async def uav_fn_get_position(self, uav_index) -> None:
        """
        Retrieve and update position data for a UAV.
        
        This function gets the current altitude (relative and absolute) and geographic
        position (latitude and longitude) of the specified UAV.
        
        Args:
            uav_index (int): The UAV to get position for (1-MAX_UAV_COUNT)
            
        """
        global UAVs
        
        try:
            # Get a single position update
            async for position in UAVs[uav_index]["system"].telemetry.position():
                # Extract position data with appropriate precision
                alt_rel = round(position.relative_altitude_m, 12)
                alt_msl = round(position.absolute_altitude_m, 12)
                latitude = round(position.latitude_deg, 12)
                longitude = round(position.longitude_deg, 12)
                
                # Update the UAV status dictionary
                UAVs[uav_index]["status"]["altitude_status"] = [alt_rel, alt_msl]
                UAVs[uav_index]["status"]["position_status"] = [latitude, longitude]
                
                # Update the position in the current position log file
                self._update_position_log(uav_index, latitude, longitude, alt_msl)
                
                # Update the UI
                self._update_uav_info_display(uav_index)
                
                # show on map
                # self.show_drones(init=False)
                    
                # Only process one position update per call, comment out if you want to make it continuous
                # break
                
        except Exception as e:
            logger.log(f"Failed to get position for UAV {uav_index}: {e}", level="error")
                
        return
    
    def _update_position_log(self, uav_index, latitude, longitude, altitude=0.0):
        """Update the current position log file for the UAV."""
        global UAVs
        try:
            # position_file = f"{__current_path__}/logs/drone_current_pos/uav_{uav_index}.txt"
            # os.makedirs(os.path.dirname(position_file), exist_ok=True)
            position_file = drone_current_pos_files[uav_index - 1]
            
            if not os.path.exists(position_file):
                os.makedirs(os.path.dirname(position_file), exist_ok=True)
                
            with open(position_file, "w") as f:
                f.write(f"{latitude},{longitude}")
                
            # Chỉ ghi lịch sử khi UAV đang thực sự trong nhiệm vụ (on_mission == True)
            if UAVs[uav_index]["status"].get("on_mission", False):
                mission_time = UAVs[uav_index]["status"].get("mission_start_time", datetime.now().strftime("%Y%m%d"))
                history_file = position_file.replace(".txt", f"_history_{mission_time}.txt")
                with open(history_file, "a") as f:
                    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                    f.write(f"{timestamp},{latitude},{longitude},{altitude}\n")
                
        except Exception as e:
            logger.log(f"Failed to update position log for UAV {uav_index}: {e}", level="warning")

    async def uav_fn_get_mode(self, uav_index) -> None:
        """
        Retrieve and update the flight mode status of a UAV.
        
        Args:
            uav_index (int): The UAV to get mode for (1-MAX_UAV_COUNT)
            
        Returns:
            None
        """
        global UAVs
        
        try:
            # Get a single flight mode update
            async for mode in UAVs[uav_index]["system"].telemetry.flight_mode():
                # Update the UAV status dictionary
                UAVs[uav_index]["status"]["mode_status"] = mode
                
                # Update the UI
                self._update_uav_info_display(uav_index)
                
                # Only process one position update per call, comment out if you want to make it continuous
                # break
                
        except Exception as e:
            logger.log(f"Failed to get flight mode for UAV {uav_index}: {e}", level="error")


    async def uav_fn_get_battery(self, uav_index) -> None:
        """
        Retrieve and update the battery status of a UAV.
        
        Args:
            uav_index (int): The UAV to get battery status for (1-MAX_UAV_COUNT)
            
        Returns:
            None
        """
        global UAVs
        
        try:
            # Get a single battery update
            async for battery in UAVs[uav_index]["system"].telemetry.battery():
                # Calculate and format battery percentage
                battery_percent = round(battery.remaining_percent * 100, 1)
                battery_status = f"{battery_percent}%"
                
                # Update the UAV status dictionary
                UAVs[uav_index]["status"]["battery_status"] = battery_status
                
                # Update the UI with critical warning if battery is low
                self._update_uav_info_display(uav_index)
                
                # Log warning if battery is low
                if battery_percent < 10:
                    logger.log(f"WARNING: UAV {uav_index} battery at {battery_percent}%", level="warning")
                    self.update_terminal(f"[WARNING] UAV {uav_index} battery at {battery_percent}%", uav_index)
                    break
                # Only process one battery update per call
                # break
                
        except Exception as e:
            logger.log(f"Failed to get battery status for UAV {uav_index}: {e}", level="error")


    async def uav_fn_get_arm_status(self, uav_index) -> None:
        """
        Retrieve and update the arming status of a UAV.
        
        Args:
            uav_index (int): The UAV to get arm status for (1-MAX_UAV_COUNT)
            
        Returns:
            None
        """
        global UAVs
        
        try:
            # Get a single arm status update
            async for armed in UAVs[uav_index]["system"].telemetry.armed():
                # Convert boolean to status string
                arm_status = "ARMED" if armed else "DISARMED"
                
                # Update the UAV status dictionary
                UAVs[uav_index]["status"]["arming_status"] = arm_status
                
                # Update the UI
                self._update_uav_info_display(uav_index)
                
                # Only process one arm status update per call
                # break
                
        except Exception as e:
            logger.log(f"Failed to get arm status for UAV {uav_index}: {e}", level="error")

    async def uav_fn_get_gps(self, uav_index) -> None:
        """
        Retrieve and update GPS information for a UAV.
        
        Args:
            uav_index (int): The UAV to get GPS info for (1-MAX_UAV_COUNT)
            
        Returns:
            None
        """
        global UAVs
        
        try:
            # Get a single GPS update
            async for gps in UAVs[uav_index]["system"].telemetry.gps_info():
                # Get GPS fix type
                gps_status = gps.fix_type
                
                # Update the UAV status dictionary
                UAVs[uav_index]["status"]["gps_status"] = gps_status
                
                # Update the UI
                self._update_uav_info_display(uav_index)
                
                # Log warning if GPS fix is poor
                if gps_status.value < 3:  # Less than 3D fix
                    logger.log(f"WARNING: UAV {uav_index} has poor GPS fix: {gps_status}", level="warning")
                    self.update_terminal(f"[WARNING] UAV {uav_index} has poor GPS fix: {gps_status}", uav_index)
                
                # Only process one GPS update per call
                # break
                
        except Exception as e:
            logger.log(f"Failed to get GPS info for UAV {uav_index}: {e}", level="error")

    async def uav_fn_get_flight_info(self, uav_index, copy=False) -> None:
        """
        Retrieve and update flight parameters for a UAV.
        
        This function gets the current flight parameters from the UAV and updates
        the parameter display fields in the UI. If 'copy' is True, it also copies 
        the values to the parameter input fields.
        
        Args:
            uav_index (int): The UAV to get parameters for (1-MAX_UAV_COUNT)
            copy (bool): If True, copy parameters to input fields
            
        Returns:
            None
        """
        global UAVs
        
        try:
            # Get parameters from the UAV
            parameters = await uav_fn_get_params(
                drone=UAVs[uav_index],
                list_params=displayed_parameter_list,
            )
            
            # Update parameter display fields
            for i, (param_name, value) in enumerate(parameters.items()):
                # Format the value to one decimal place
                formatted_value = str(round(value, 1))
                
                # Update the display field
                self.uav_param_displays[uav_index - 1].children()[i + 1].setText(formatted_value)
                
                # If requested, also copy to the input field
                if copy:
                    self.uav_param_sets[uav_index - 1].children()[i + 1].setText(formatted_value)
                    
        except Exception as e:
            logger.log(f"Failed to get flight parameters for UAV {uav_index}: {e}", level="error")
            self.popup_msg(
                f"Error retrieving flight parameters: {repr(e)}", 
                src_msg="uav_fn_get_flight_info", 
                type_msg="error"
            )

    async def uav_fn_set_flight_info(self, uav_index) -> None:
        """
        Set flight parameters for a UAV.
        
        This function gets parameter values from the input fields, validates them,
        and sends them to the UAV. It then refreshes the parameter display.
        
        Args:
            uav_index (int): The UAV to set parameters for (1-MAX_UAV_COUNT)
            
        Returns:
            None
        """
        global UAVs
        
        try:
            # Initialize parameters dictionary
            parameters = {}
            
            # Get widgets containing current and new values
            input_widgets = self.uav_param_sets[uav_index - 1].children()[1:-1]
            display_widgets = self.uav_param_displays[uav_index - 1].children()[1:-1]
            
            # Populate parameters from input fields, falling back to current values if empty
            for i, (input_widget, display_widget) in enumerate(zip(input_widgets, display_widgets)):
                param_name = displayed_parameter_list[i]
                input_text = input_widget.text()
                
                if not input_text:
                    # Use current value if input is empty
                    parameters[param_name] = float(display_widget.text())
                else:
                    try:
                        # Validate and convert input to float
                        parameters[param_name] = float(input_text)
                    except ValueError:
                        # Handle invalid input
                        logger.log(f"Invalid value for parameter {param_name}: {input_text}", level="warning")
                        self.popup_msg(
                            f"Invalid value for {param_name}: {input_text}", 
                            src_msg="uav_fn_set_flight_info", 
                            type_msg="Warning"
                        )
                        # Use current value instead
                        parameters[param_name] = float(display_widget.text())
            
            # Send parameters to the UAV and save to file
            await uav_fn_set_params(
                drone=UAVs[uav_index],
                parameters=parameters,
                param_file=parameter_data_files[uav_index - 1],
            )
            
            # Refresh parameter display
            await self.uav_fn_get_flight_info(uav_index=uav_index, copy=False)
            
            # Log and display success message
            logger.log(f"Updated flight parameters for UAV {uav_index}", level="info")
            self.update_terminal(f"[INFO] Updated flight parameters for UAV {uav_index}")
            
        except Exception as e:
            logger.log(f"Failed to set flight parameters for UAV {uav_index}: {e}", level="error")
            self.popup_msg(
                f"Error setting flight parameters: {repr(e)}", 
                src_msg="uav_fn_set_flight_info", 
                type_msg="Error"
            )

    async def uav_fn_print_status(self, uav_index) -> None:
        """
        Display status text messages from a UAV in the terminal.
        
        Args:
            uav_index (int): The UAV to get status messages from (1-MAX_UAV_COUNT)
            
        Returns:
            None
        """
        global UAVs
        
        # Skip if UAV is not connected or not allowed
        if not self._check_uav_connection(uav_index):
            return
        
        try:
            # Get and display status text messages
            async for status in UAVs[uav_index]["system"].telemetry.status_text():
                # Format the status message
                status_text = f"> {status.type} - {status.text}"
                
                # Display in the terminal
                self.update_terminal(status_text, uav_index)
                
                # Log to file based on severity
                if status.type.name in ["ERROR", "CRITICAL"]:
                    logger.log(f"UAV {uav_index}: {status.text}", level="error")
                elif status.type.name == "WARNING":
                    logger.log(f"UAV {uav_index}: {status.text}", level="warning")
                else:
                    logger.log(f"UAV {uav_index}: {status.text}", level="debug")
                    
        except Exception as e:
            logger.log(f"Failed to print status for UAV {uav_index}: {e}", level="error")

    # -----------------------------< UAVs streaming functions >-----------------------------
    @pyqtSlot(np.ndarray, list)
    def stream_on_uav_screen(self, annotated_frame=None, results=None) -> None:
        """
        Display video stream on the UAV screen with optional object detection annotations.
        
        This method processes video frames for display in the UAV interface. It handles raw 
        or annotated frames with detection results, manages frame rate throttling, and exports 
        detection data when targets are found.
        
        Args:
            frame (np.ndarray): The original video frame without annotations
            annotated_frame (np.ndarray): The frame with detection annotations
            results (list): Contains [uav_index, current_fps, detected_results] where:
                            - uav_index: The UAV identifier
                            - current_fps: Current frames per second
                            - detected_results: Detection results including track IDs and object data
        
        Returns:
            None

        Notes:
            - The method only processes frames if UAV connection is allowed and streaming is enabled
            - Frame rate is limited according to DEFAULT_STREAM_FPS
            - When detection is enabled and a person is detected, the frame is saved and GPS coordinates
            are exported to logs
        """

        global UAVs
        if not results:
            logger.log("Received empty results in stream handler", level="warning")
            return
            
        # Unpack the results
        uav_index, current_fps, detected_results = results
        uav_index = int(uav_index)
        
        # Skip processing if UAV is not eligible for streaming
        if not self._can_stream(uav_index):
            return
            
        try:
            # Apply frame rate limiting
            if not self._should_process_frame(uav_index, current_fps):
                return
                
            # Select the appropriate frame to display
            streaming_frame = annotated_frame
            
            # Display the frame
            asyncio.create_task(self.update_uav_screen_view(
                uav_index, streaming_frame, screen_name=DEFAULT_STREAM_SCREEN
            ))
            
            # Process detection results if available and detection is enabled
            if UAVs[uav_index]["detection_enable"] and detected_results:
                asyncio.create_task(self._process_detection_results(uav_index, annotated_frame, detected_results))

                
        except Exception as e:
            # Update status and show error message
            UAVs[uav_index]["status"]["streaming_status"] = False
            logger.log(f"Stream display error for UAV {uav_index}: {repr(e)}", level="error")
            self.popup_msg(
                f"Stream display error: {repr(e)}",
                src_msg="stream_on_uav_screen",
                type_msg="error",
            )
            
            
    def _check_uav_connection(self, uav_index, strictly=True):
        """Check if a UAV is connected and allowed to receive commands."""
        if strictly:
            return (UAVs[uav_index]["status"]["connection_status"] and 
                    UAVs[uav_index]["connection_allow"])
        else:
            return (UAVs[uav_index]["status"]["connection_status"] or
                    UAVs[uav_index]["connection_allow"])
            
    def _can_stream(self, uav_index):
        """Check if UAV is eligible for stream display."""
        return (
            self._check_uav_connection(uav_index=uav_index, strictly=False) and 
            UAVs[uav_index]["streaming_enable"]
        )

    def _should_process_frame(self, uav_index, current_fps):
        """Apply frame rate limiting to avoid overloading the UI."""
        # Calculate the frame skip rate to achieve target FPS
        max_frame_cnt = max(1, current_fps // DEFAULT_STREAM_FPS)
        
        # Increment the frame counter for this UAV
        self.uav_stream_frame_cnt[uav_index - 1] += 1
        
        # Process frame if it's time to display based on our rate limiting
        return self.uav_stream_frame_cnt[uav_index - 1] % max_frame_cnt == 0

    def _select_frame_type(self, uav_index, frame, annotated_frame):
        """Select which frame to display based on detection settings."""
        # Use annotated frame if detection is enabled, otherwise use raw frame
        return annotated_frame if UAVs[uav_index]["detection_enable"] else frame

    async def _process_detection_results(self, uav_index, annotated_frame, detected_results):
        """Process object detection results and handle detected targets."""
        global UAVs
        
        if detected_results is None or len(detected_results) != 2:
            return
            
        track_ids, objects = detected_results
        
        for track_id, obj in zip(track_ids, objects):
            # Skip if not a detected person
            if not obj["detected"] or obj["class"] != "person":
                continue
            # Disable detection feature after finding a target
            await UAVs[uav_index]["system"].mission.pause_mission()
            #await UAVs[uav_index]["system"].action.hold()  
            UAVs[uav_index]["detection_enable"] = False 
            UAVs[uav_index]["status"]["on_mission"] = False
            if self.active_tab_index == uav_index:
                self._set_pause_button_style("Resume")

            # Get UAV position and frame information
            frame_shape = annotated_frame.shape
            detected_pos = (obj["x"], obj["y"])
            
            # Get current GPS coordinates
            # with open(drone_current_pos_files[uav_index - 1], "r") as f:
            #     gps_data = f.read()
            #     uav_lat, uav_long = map(float, gps_data.split(","))
                
            uav_lat, uav_long = UAVs[uav_index]["status"]["position_status"]
            uav_alt = UAVs[uav_index]["status"]["altitude_status"][0]
            uav_gps = [uav_lat, uav_long, uav_alt]

            # Export detection to GPS log
            asyncio.create_task(export_points_to_gps_log(
                uav_index=uav_index,
                detected_pos=detected_pos,
                frame_shape=frame_shape,
                uav_gps=uav_gps,
            ))
            # Save the detection frame
            asyncio.create_task(self._save_detection_image(uav_index, track_id, annotated_frame))
            
            # Log detection to terminal
            asyncio.create_task(self._log_detection(uav_index, obj["class"], detected_pos, frame_shape, uav_gps))

            await asyncio.sleep(1)
            UAVs[uav_index]["system"].mission.start_mission()
            UAVs[uav_index]["status"]["on_mission"] = True
            if self.active_tab_index == uav_index:
                self._set_pause_button_style("Pause")
            await asyncio.sleep(25)
            UAVs[uav_index]["detection_enable"] = True

            
            # Only process the first detected person
            break

    async def _save_detection_image(self, uav_index, track_id, frame):
        """Save a detection image to the logs directory."""
        image_path = f"{__current_path__}/logs/images/UAV{uav_index}_locked_target_{track_id}.png"
        os.makedirs(os.path.dirname(image_path), exist_ok=True)
        cv2.imwrite(image_path, frame)
        logger.log(f"Saved detection image to {image_path}", level="info")

    async def _log_detection(self, uav_index, class_name, detected_pos, frame_shape, uav_gps):
        """Log detection information to the terminal."""
        global UAVs
        
        detection_msg = (
            f"UAV-{uav_index} at GPS ({uav_gps[0]}, {uav_gps[1]}, {uav_gps[2]}m) "
            f"detected {class_name} at X: {detected_pos[0]:.1f} Y: {detected_pos[1]:.1f} "
            f"with frame size: {frame_shape[1]}x{frame_shape[0]}"
        )
        self.update_terminal(detection_msg, 0)
        logger.log(detection_msg, level="info")

    async def monitor_mission_progress(self, uav_index):
        """Monitor mission progress and update the UI progress bar for the given UAV."""
        global UAVs
        try:
            progress_bars = [
                self.ui.progressUAV1_2, self.ui.progressUAV2_2, self.ui.progressUAV3_2,
                self.ui.progressUAV4_2, self.ui.progressUAV5_2, self.ui.progressUav6_2
            ]
            labels = [
                self.ui.progressLabel1_2, self.ui.progressLabel2_2, self.ui.progressLabel3_2,
                self.ui.progressLabel4_2, self.ui.progressLabel5_2, self.ui.progressLabel6_2
            ]
            
            if not (1 <= uav_index <= 6): return
                
            bar = progress_bars[uav_index - 1]
            label = labels[uav_index - 1]
            
            bar.setValue(0)
            label.setText("0/0")
            
            while True:
                try:
                    async for progress in UAVs[uav_index]["system"].mission.mission_progress():
                        current = progress.current
                        total = progress.total
                        if total > 0:
                            bar.setMaximum(total)
                            bar.setValue(current)
                            if current == total:
                                label.setText(f"Finished ({total})")
                            else:
                                label.setText(f"{current}/{total}")
                except Exception:
                    pass
                await asyncio.sleep(0.5)
        except asyncio.CancelledError:
            pass
        except Exception as e:
            logger.error(f"Error monitoring mission progress for UAV {uav_index}: {e}")

    def _on_map_type_changed(self, index):
        """Đổi form nhập liệu tương ứng khi chọn Map Type."""
        if index in [0, 3, 4]:  # Square, Pentagon, Hexagon dùng chung trang 1 tham số
            self.ui.stackedMapParam_2.setCurrentIndex(0)
            if index == 0:
                self.ui.label_26.setText("Side Length (m)")
            else:
                self.ui.label_26.setText("Radius (m)")
        elif index == 1:  # Rectangle
            self.ui.stackedMapParam_2.setCurrentIndex(1)
        elif index == 2:  # Circle
            self.ui.stackedMapParam_2.setCurrentIndex(2)
        elif index == 5:  # Rescue
            self.ui.stackedMapParam_2.setCurrentIndex(3)

    # ------------------------------------< Simulation Functions >-----------------------------
    async def run_simulation_callback(self):
        """
        Handles the 'Run Simulation' button click event.
        - Generates a mission area based on UI settings.
        - Creates grid points within that area.
        - Runs selected pathfinding algorithms.
        - Displays results in the comparison table and on the simulation map.
        """
        
        # Kiểm tra kết nối UAV 1 ngay từ đầu trước khi tính toán
        if not self._check_uav_connection(1, strictly=True):
            self.update_terminal("[SIM] UAV 1 is not connected. Simulation aborted.", 0)
            self.popup_msg("Vui lòng Connect UAV 1 trước khi chạy Simulation!", "Simulation", "Warning")
            return
            
        self.update_terminal("[SIM] Starting simulation...", 0)

        # 1. Clear previous results
        self.sim_map.runScript("""
            if (typeof map !== 'undefined') {
                map.eachLayer(function (layer) {
                    if (layer instanceof L.Marker || layer instanceof L.Polygon || layer instanceof L.Polyline) {
                        map.removeLayer(layer);
                    }
                });
            }
        """)
        
        # Clear only data columns, keep algorithm names in column 0
        for row in range(self.ui.tableWidgetAlgorithmComparison.rowCount()):
            for col in range(1, self.ui.tableWidgetAlgorithmComparison.columnCount()):
                self.ui.tableWidgetAlgorithmComparison.setItem(row, col, QtWidgets.QTableWidgetItem(""))
                
        self.ui.tableWidgetAlgorithmComparison.setHorizontalHeaderLabels(
            ["Algorithm", "Avg Cost", "Coverage", "Energy", "Time", "Path", "Overlap", "Turns", "Score"]
        )

        # 2. Get parameters from UI
        map_type = self.ui.comboBox_3.currentText()
        num_runs = self.ui.Num_run_2.value()
        if num_runs <= 0: num_runs = 1
        try:
            grid_size = float(self.ui.gridSize_line_edit.text()) # Lấy tạm grid size từ tab map
            if grid_size <= 0: grid_size = 10.0
        except ValueError:
            grid_size = 10.0
        
        # Lấy danh sách thuật toán được chọn
        selected_algos = []
        if self.ui.zigzag_2.isChecked(): selected_algos.append("Zigzag")
        if self.ui.FindPath_2.isChecked(): selected_algos.append("Find_Path")
        if self.ui.NN2Opt_2.isChecked(): selected_algos.append("NN_2opt")
        if self.ui.SA_2.isChecked(): selected_algos.append("SA")
        if self.ui.ACO_2.isChecked(): selected_algos.append("ACO")
        if self.ui.GA_2.isChecked(): selected_algos.append("GA")
        if self.ui.GA_with_1.isChecked(): selected_algos.append("GA_with_turn")
        if self.ui.ABC_2.isChecked(): selected_algos.append("ABC")
        if self.ui.Improve_A_2.isChecked(): selected_algos.append("A*_Improved")

        if not selected_algos:
            self.popup_msg("Please select at least one algorithm.", "Simulation", "Warning")
            return
            
        total_runs = len(selected_algos) * num_runs
        current_run = 0
        self.ui.progressBar.setMaximum(total_runs)
        self.ui.progressBar.setValue(0)
        self.ui.label_32.setText(f"0/{total_runs}")
        await asyncio.sleep(0) # Ép giao diện render ngay lập tức trạng thái 0/x trước khi thuật toán chạy

        # 3. Generate mission area (polygon)
        try:
            # Lấy vị trí UAV 1 làm trung tâm
            center_lat = UAVs[1]["init_params"]["latitude"]
            center_lon = UAVs[1]["init_params"]["longitude"]
            
            # Focus bản đồ mô phỏng vào khu vực này
            self.sim_map.centerAt(center_lat, center_lon)
            self.sim_map.setZoom(18)

            if map_type == "Square":
                side = self.ui.spinBox_5.value()
                if side <= 0: side = 100.0  # Mặc định 100m
                # Tạo 4 góc của hình vuông
                half_side = side / 2.0
                p1 = calculate_new_lat_lon(center_lat, center_lon, half_side, -half_side)  # Top-left
                p2 = calculate_new_lat_lon(center_lat, center_lon, half_side, half_side)   # Top-right
                p3 = calculate_new_lat_lon(center_lat, center_lon, -half_side, half_side)  # Bottom-right
                p4 = calculate_new_lat_lon(center_lat, center_lon, -half_side, -half_side) # Bottom-left
                polygon_vertices = [p1, p2, p3, p4, p1]
            elif map_type == "Rectangle":
                width = self.ui.spinBox_6.value()
                height = self.ui.spinBox_7.value()
                if width <= 0: width = 100.0
                if height <= 0: height = 100.0
                half_w, half_h = width / 2.0, height / 2.0
                p1 = calculate_new_lat_lon(center_lat, center_lon, half_h, -half_w)
                p2 = calculate_new_lat_lon(center_lat, center_lon, half_h, half_w)
                p3 = calculate_new_lat_lon(center_lat, center_lon, -half_h, half_w)
                p4 = calculate_new_lat_lon(center_lat, center_lon, -half_h, -half_w)
                polygon_vertices = [p1, p2, p3, p4, p1]
            else:
                self.popup_msg(f"Map Type '{map_type}' is not implemented yet.", "Simulation", "Warning")
                return

            # Vẽ vùng bay lên bản đồ sim
            self.sim_map.drawPolygon("sim_area", polygon_vertices, options={'color': 'blue', 'fillOpacity': 0.1})

            # 4. Generate grid points
            cartesian_poly = convert_to_cartesian(polygon_vertices)
            grid_points_cartesian = generate_grid(cartesian_poly, grid_size)
            
            # Chuyển đổi điểm grid về lại Lat/Lon
            ref_lat = min(p[0] for p in polygon_vertices)
            ref_lon = min(p[1] for p in polygon_vertices)
            grid_points_latlon = [convert_to_lat_lon((ref_lat, ref_lon), p) for p in grid_points_cartesian]
            
            if not grid_points_latlon or len(grid_points_latlon) < 2:
                self.popup_msg("Vùng sinh quá bé hoặc Grid Size quá lớn, không đủ tạo điểm bay!", "Simulation", "Warning")
                self.update_terminal("[SIM] Simulation aborted: Not enough points.", 0)
                return

            # Vẽ các điểm grid lên bản đồ sim
            marker_options = {
                'icon': str(DOT_ICON_PATH), 
                'iconSize': {'width': 5, 'height': 5},
                'title': ' '  # Truyền khoảng trắng để ép bản đồ ẩn tên điểm (tooltip) đi
            }
            for i, p in enumerate(grid_points_latlon):
                self.sim_map.addMarker(f"sim_pt_{i}", p[0], p[1], **marker_options)

        except Exception as e:
            self.popup_msg(f"Error generating map area/grid: {e}", "Simulation", "Error")
            logger.error(f"[SIM] Error generating map/grid: {e}")
            return

        # 5. Run algorithms and display results
        self.update_terminal(f"[SIM] Running {len(selected_algos)} algorithms, {num_runs} runs each...", 0)
        await asyncio.sleep(0) # Nhường quyền cho event loop in log ra màn hình trước
        
        algo_map = {
            "Zigzag": lambda pts, start: find_zigzag_path(pts, start)[0],
            "Find_Path": find_path_0,
            "NN_2opt": nn_2opt_path,
            "SA": sa_path,
            "ACO": aco_path,
            "GA": ga_path,
            "GA_with_turn": ga_path_with_turns,
            "A*_Improved": astar_path_with_turns,
            "ABC": abc_path
        }
        
        algo_to_row = {
            "Zigzag": 0,
            "Find_Path": 1,
            "NN_2opt": 2,
            "SA": 3,
            "ACO": 4,
            "GA": 5,
            "GA_with_turn": 6,
            "A*_Improved": 7
        }

        best_overall_score = float('inf')
        best_overall_path = None
        best_overall_algo = ""
        
        # Lấy cao độ mặc định và tọa độ ban đầu của UAV 1
        uav_alt = UAVs[1]["init_params"].get("altitude", 10.0)
        start_coord = (center_lat, center_lon)

        for algo_name in selected_algos:
            if algo_name not in algo_map:
                continue
            
            total_cost, total_dist, total_turns = 0, 0, 0
            total_flight_time = 0
            total_battery_drop = 0
            current_best_path_for_algo = []

            try:
                for run_idx in range(num_runs):
                    self.update_terminal(f"\n[SIM] === Đang chạy {algo_name.replace('_', ' ')} - Lượt {run_idx+1}/{num_runs} ===", 0)
                    await asyncio.sleep(0)
                    
                    # 1. TÍNH TOÁN ĐƯỜNG ĐI TOÁN HỌC
                    path = algo_map[algo_name](grid_points_latlon.copy(), start_coord)
                    if path and (abs(path[0][0] - start_coord[0]) > 1e-7 or abs(path[0][1] - start_coord[1]) > 1e-7):
                        path.insert(0, start_coord)
                        
                    cost, dist, turns = calculate_cost_for_path(path)
                    total_cost += cost
                    total_dist += dist
                    total_turns += turns
                    current_best_path_for_algo = path
                    
                    # Vẽ tạm đường đi dự kiến lên bản đồ màu cam nhạt
                    self.sim_map.drawPolyLine("current_run_path", path, options={'color': 'orange', 'weight': 3, 'opacity': 0.5})
                    
                    # 2. XUẤT TỌA ĐỘ RA FILE
                    sim_plan_file = os.path.join(__current_path__, "logs", "points", "simulation_path.txt")
                    os.makedirs(os.path.dirname(sim_plan_file), exist_ok=True)
                    with open(sim_plan_file, "w") as f:
                        for pt in path:
                            f.write(f"{pt[0]},{pt[1]},{uav_alt}\n")
                            
                    # 3. ĐO LƯỜNG VÀ BAY MÔ PHỎNG THỰC TẾ
                    # Ghi nhận thời gian và pin trước khi cất cánh
                    start_time = time.time()
                    batt_str = UAVs[1]["status"].get("battery_status", "100%")
                    start_battery = float(batt_str.replace('%', '')) if '%' in batt_str else 100.0
                    
                    # Kích hoạt chuyến bay khép kín (đợi cho tới khi nó hạ cánh hẳn mới đi tiếp)
                    await self._run_single_sim_flight(1, sim_plan_file)
                    
                    # Ghi nhận thời gian và pin sau khi hạ cánh
                    end_time = time.time()
                    batt_str = UAVs[1]["status"].get("battery_status", "100%")
                    end_battery = float(batt_str.replace('%', '')) if '%' in batt_str else start_battery
                    
                    flight_time = end_time - start_time
                    battery_drop = max(0, start_battery - end_battery)
                    
                    total_flight_time += flight_time
                    total_battery_drop += battery_drop
                    self.update_terminal(f"[SIM] Kết quả lượt {run_idx+1}: Thời gian = {flight_time:.1f}s, Tiêu hao pin = {battery_drop:.1f}%", 0)
                    
                    # 4. CẬP NHẬT GIAO DIỆN
                        
                    current_run += 1
                    self.ui.progressBar.setValue(current_run)
                    self.ui.label_32.setText(f"{current_run}/{total_runs}")
                    await asyncio.sleep(0)
                    
                    if current_run < total_runs:
                        self.update_terminal("[SIM] Đợi 5 giây làm nguội trước khi cất cánh lượt tiếp theo...", 0)
                        await asyncio.sleep(5)

                # 5. TÍNH TRUNG BÌNH & ĐIỀN VÀO BẢNG TỔNG KẾT
                if num_runs > 0:
                    avg_cost = total_cost / num_runs
                    avg_dist = total_dist / num_runs
                    avg_turns = total_turns / num_runs
                    avg_time = total_flight_time / num_runs
                    avg_battery = total_battery_drop / num_runs

                    row = algo_to_row.get(algo_name)
                    if row is not None:
                        self.ui.tableWidgetAlgorithmComparison.setItem(row, 1, QtWidgets.QTableWidgetItem(f"{avg_cost:.2f}"))
                        self.ui.tableWidgetAlgorithmComparison.setItem(row, 3, QtWidgets.QTableWidgetItem(f"{avg_battery:.2f}%"))
                        self.ui.tableWidgetAlgorithmComparison.setItem(row, 4, QtWidgets.QTableWidgetItem(f"{avg_time:.1f} s"))
                        self.ui.tableWidgetAlgorithmComparison.setItem(row, 5, QtWidgets.QTableWidgetItem(f"{avg_dist:.2f} m"))
                        self.ui.tableWidgetAlgorithmComparison.setItem(row, 7, QtWidgets.QTableWidgetItem(f"{avg_turns:.1f}"))
                        
                        # Tính điểm tổng hợp (Score): thời gian và pin tiêu thụ càng thấp càng tốt
                        score = avg_time + (avg_battery * 10)
                        self.ui.tableWidgetAlgorithmComparison.setItem(row, 8, QtWidgets.QTableWidgetItem(f"{score:.1f}"))
                        
                        # Cập nhật xem thuật toán nào đang vô địch về Score
                        if score < best_overall_score:
                            best_overall_score = score
                            best_overall_algo = algo_name
                            best_overall_path = current_best_path_for_algo
            except Exception as e:
                self.update_terminal(f"[SIM] Lỗi thuật toán {algo_name}: {e}", 0)
                logger.error(f"[SIM] Error in algorithm {algo_name}: {e}")
                continue

        # 6. HOÀN TẤT & VẼ KẾT QUẢ TỐT NHẤT LÊN BẢN ĐỒ
        # Xoá đường màu cam nhạt để vẽ đường chính thức
        self.sim_map.runScript("map.eachLayer(function(l){if(l.options&&l.options.color==='orange')map.removeLayer(l);});")
        
        if best_overall_path:
            self.sim_map.drawPolyLine("best_path", best_overall_path, options={'color': 'purple', 'weight': 5})
            self.update_terminal(f"\n[SIM] === HOÀN TẤT MÔ PHỎNG MỌI THUẬT TOÁN ===", 0)
            self.update_terminal(f"[SIM] Thuật toán tốt nhất thực tế: {best_overall_algo.replace('_', ' ')} (Score: {best_overall_score:.1f})", 0)
            self.popup_msg(f"Mô phỏng hoàn tất!\nThuật toán tối ưu nhất: {best_overall_algo.replace('_', ' ')}", "Simulation", "Info")
        else:
            self.update_terminal("[SIM] Hoàn tất mô phỏng nhưng không tìm thấy đường đi hợp lệ.", 0)
        
    async def _run_single_sim_flight(self, uav_index, plan_file):
        """
        Tiến trình thực thi một vòng bay khép kín (Laps):
        Upload -> Cất cánh -> Theo lộ trình -> RTL -> Chờ chạm đất & Disarm.
        Hàm này sẽ khóa (block) các tác vụ sau nó cho tới khi UAV hoàn toàn nằm yên dưới mặt đất.
        """
        try:
            UAVs[uav_index]["status"]["on_mission"] = True
            if self.active_tab_index == uav_index:
                self._set_pause_button_style("Pause")
                
            # Khởi chạy progress bar bay
            progress_task = asyncio.create_task(self.monitor_mission_progress(uav_index))
            
            # Task ngầm: Liên tục kiểm tra xem bay hết điểm chưa để bắn lệnh về
            async def auto_rtl_when_finished():
                while UAVs[uav_index]["status"].get("on_mission", False):
                    try:
                        if await UAVs[uav_index]["system"].mission.is_mission_finished():
                            self.update_terminal(f"[SIM] UAV {uav_index} hoàn tất các điểm, đang tự động quay về (RTL)...", 0)
                            await UAVs[uav_index]["system"].action.return_to_launch()
                            break
                    except Exception:
                        pass
                    await asyncio.sleep(1)
            
            rtl_task = asyncio.create_task(auto_rtl_when_finished())
            
            # Kích hoạt hàm cất cánh và bay (hàm uav_fn_do_mission sẽ tự động block cho đến khi UAV đáp đất hoàn toàn)
            self.update_terminal(f"[SIM] Bắt đầu nạp lộ trình và cất cánh UAV {uav_index}...", 0)
            await uav_fn_do_mission(drone=UAVs[uav_index], mission_plan_file=plan_file)
            
            # Hủy các task ngầm khi chuyến bay đã kết thúc
            if not rtl_task.done(): rtl_task.cancel()
            if not progress_task.done(): progress_task.cancel()
            
            # Khôi phục nút UI
            UAVs[uav_index]["status"]["on_mission"] = False
            if self.active_tab_index == uav_index:
                self._set_pause_button_style("Resume")
                
            # QUAN TRỌNG: Đợi UAV xả động cơ (Disarm) hoàn toàn để reset hệ thống trước vòng lặp sau
            self.update_terminal(f"[SIM] Đợi UAV {uav_index} xả động cơ (Disarm) an toàn...", 0)
            while True:
                is_armed = False
                try:
                    async for armed in UAVs[uav_index]["system"].telemetry.armed():
                        is_armed = armed
                        break
                except Exception:
                    pass
                if not is_armed:
                    break
                await asyncio.sleep(1)
                
        except Exception as e:
            self.update_terminal(f"[SIM] Lỗi trong chuyến bay: {e}", 0)
            raise e
        
    # async def _run_sim_uav_mission(self, uav_index, plan_file):
    #     """
    #     Tiến trình ngầm thực thi mission cho UAV trên tab Simulation.
    #     Gồm các bước: Upload -> Arm -> Takeoff -> Start Mission.
    #     """
    #     try:
    #         UAVs[uav_index]["status"]["on_mission"] = True
    #         if self.active_tab_index == uav_index:
    #             self._set_pause_button_style("Pause")
                
    #         # Chạy hàm mission chuẩn (đã bao gồm cất cánh và làm nhiệm vụ)
    #         await uav_fn_do_mission(drone=UAVs[uav_index], mission_plan_file=plan_file)
            
    #         # Kiểm tra nếu hoàn thành thì RTL (Quay về)
    #         if await UAVs[uav_index]["system"].mission.is_mission_finished():
    #             self.update_terminal(f"[SIM] UAV {uav_index} finished mission. Returning to launch.", 0)
    #             await UAVs[uav_index]["system"].action.return_to_launch()
                
    #         UAVs[uav_index]["status"]["on_mission"] = False
    #         if self.active_tab_index == uav_index:
    #             self._set_pause_button_style("Resume")
    #     except Exception as e:
    #         logger.error(f"[SIM] Error executing flight simulation: {e}")
    #         self.update_terminal(f"[SIM] Error executing flight simulation: {e}", 0)

    # ------------------------------------< Rescue UAV 6 >-----------------------------
    # ? developing ...
    async def uav_fn_rescue(self) -> None:
        """
        Perform a rescue mission using the specified UAV.
        This function checks the connection status of the rescue UAV, connects to it,
        verifies its health, and retrieves its initial position. It then performs the
        rescue mission if certain conditions are met.
        The rescue mission involves:
        1. Checking if rescue position logs are available.
        2. Selecting a mission plan from the available logs.
        3. Performing the rescue mission and suspending detected UAVs.
        4. Removing the rescue log file after the mission is completed.
        If any error occurs during the mission, it logs the error and displays a popup message.
        Returns:
            None
        """

        global UAVs
        if not (
            UAVs[RESCUE_UAV_INDEX]["status"]["connection_status"]
            and UAVs[RESCUE_UAV_INDEX]["connection_allow"]
        ):
            return

        self.update_terminal(f"[INFO] Sent RESCUE command to UAV {RESCUE_UAV_INDEX}")

        await UAVs[RESCUE_UAV_INDEX]["system"].connect(
            system_address=UAVs[RESCUE_UAV_INDEX]["system_address"]
        )
        # check health 
        # TODO: check battery level here
        async for health in UAVs[RESCUE_UAV_INDEX]["system"].telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                logger.log(
                    f"UAV-{RESCUE_UAV_INDEX} -- Global position for estimate OK", level="info"
                )
                break
            
        logger.log(f"UAV-{RESCUE_UAV_INDEX} -- Arming and taking off", level="info")

        # await asyncio.sleep(1)
        # await UAVs[RESCUE_UAV_INDEX]["system"].action.arm()
        # await asyncio.sleep(3)
        # await UAVs[RESCUE_UAV_INDEX]["system"].action.takeoff()
        # await asyncio.sleep(15)
        
        try:
            # do the rescue mission loop
            # 1. check if the rescue position is available
            # 2. get the detected UAVs
            # 3. do the rescue mission

            while True:
                # 1 check if rescue position is available
                rescue_filepaths = glob.glob(
                    f"{__current_path__}/logs/rescue_pos/rescue_pos_uav_*.log"
                )

                if len(rescue_filepaths) == 0:
                    # logger.log(
                    #     f"No rescue position found, re-check rescue directory...", level="info"
                    # )
                    await asyncio.sleep(1)
                    continue

                # NOTE: you can implement your own logic here
                # get the detected UAVs
                #detected_uav_list = []
                for rescue_filepath in rescue_filepaths:
                    uav_index = int(str(Path(rescue_filepath).stem).split("_")[-1])
                    print(f"Detected UAV: {uav_index}")
                    #detected_uav_list.append(UAVs[uav_index])
                
                # get the rescue filepath
                
                logger.log(
                    f"Found {len(rescue_filepaths)} rescue files",
                    level="info",
                )
                rescue_filepath = select_mission_plan(rescue_filepaths)
                logger.log(
                    f"Selected rescue file: {rescue_filepath}",
                    level="info",
                )
                
                # 
                logger.log("Rescue mission started...", level="info")

                await asyncio.sleep(1)
                #asyncio.create_task(self.send_coordinate()) #tu dong gui tin nhan
                await UAVs[RESCUE_UAV_INDEX]["system"].action.arm()
                await asyncio.sleep(3)
                await UAVs[RESCUE_UAV_INDEX]["system"].action.takeoff()
                await asyncio.sleep(10)
                               
                logger.log(
                    f"UAV-{RESCUE_UAV_INDEX} -- Takeoff completed, ready to start rescue mission", level="info"
                )

                # get initial position
                async for position in UAVs[RESCUE_UAV_INDEX]["system"].telemetry.position():
                    UAVs[RESCUE_UAV_INDEX]["init_params"]["latitude"] = round(position.latitude_deg, 12)
                    UAVs[RESCUE_UAV_INDEX]["init_params"]["longitude"] = round(position.longitude_deg, 12)
                    break
                
                # 2 UAV Rescue do the rescue mission and the detected drones goes into suspend mode
                UAVs[RESCUE_UAV_INDEX]["status"]["on_mission"] = True
                UAVs[RESCUE_UAV_INDEX]["status"]["mission_start_time"] = datetime.now().strftime("%Y%m%d_%H%M%S")
                if self.active_tab_index == RESCUE_UAV_INDEX:
                    self._set_pause_button_style("Pause")
                await asyncio.gather(
                    #uav_suspend_missions(drones=detected_uav_list, suspend_time=30),
                    uav_rescue_process(UAVs[RESCUE_UAV_INDEX], rescue_filepath, self)
                    # uav_rescue_process(
                    #     drone=UAVs[RESCUE_UAV_INDEX], rescue_filepath=rescue_filepath, self
                    # ),
                    # uav_rescue_process(
                    #     drone=UAVs[RESCUE_UAV_INDEX], rescue_filepath=rescue_filepath
                    # ),
                )
                UAVs[RESCUE_UAV_INDEX]["status"]["on_mission"] = False
                if self.active_tab_index == RESCUE_UAV_INDEX:
                    self._set_pause_button_style("Resume")
                UAVs[RESCUE_UAV_INDEX]["rescue_first_time"] = False
                await asyncio.sleep(15)
                
                # 3 remove the rescue file
                if os.path.exists(rescue_filepath):
                    os.remove(rescue_filepath)  # remove the rescue file
                    logger.log(f"Rescue file {rescue_filepath} removed", level="info")
                
                # 4 remove the detected UAVs from the list
                # self.detected_uav_list.remove(uav_index)
                # self.detected_uav_list = []
                break  # remove this line if you want to do the rescue mission continuously

            logger.log(f"Rescue mission completed", level="info")
            # start rescue mission again
            #await self.uav_fn_rescue()
        except Exception as e:
            logger.log(f"Error: {repr(e)}", level="error")
            self.popup_msg(f"Error: {repr(e)}", src_msg="uav_fn_rescue", type_msg="Error")

# ------------------------------------< Main Application Class >-----------------------------
def run():
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Oxygen")  # ['Breeze', 'Oxygen', 'QtCurve', 'Windows', 'Fusion']
    loop = QEventLoop(app)
    asyncio.set_event_loop(loop)
    MainWindow = App()
    MainWindow.show()

    with loop:
        pending = asyncio.all_tasks(loop=loop)
        for task in pending:
            task.cancel()

        sys.exit(loop.run_forever())


if __name__ == "__main__":
    run()
