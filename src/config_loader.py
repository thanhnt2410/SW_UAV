import os
import yaml
from pathlib import Path
from datetime import datetime

class ConfigLoader:
    """
    A centralized class to load, process, and provide access to all YAML configurations.
    It handles path replacements and generates dynamic values needed by the application.
    """
    def __init__(self, config_dir_rel_to_src='../config'):
        # --- 1. Define Base Paths ---
        self.SRC_DIR = Path(__file__).parent.resolve()
        self.ROOT_DIR = self.SRC_DIR.parent
        self.NOW = datetime.now().strftime("%y-%m-%d_%H-%M-%S")
        self.config_dir = (self.SRC_DIR / config_dir_rel_to_src).resolve()

        # --- 2. Load All YAML Files ---
        self.uav = self._load_yaml('uav_config.yaml')
        self.stream = self._load_yaml('stream_config.yaml')
        self.interface = self._load_yaml('interface_config.yaml')
        self.init_pos = self._load_yaml('init_pos_uavs.yaml')

        # --- 3. Process and Generate Dynamic Values ---
        self._process_paths()
        self._generate_derived_values()

    def _load_yaml(self, filename: str):
        """Loads a single YAML file from the config directory."""
        file_path = self.config_dir / filename
        try:
            with open(file_path, 'r') as f:
                return yaml.safe_load(f)
        except FileNotFoundError:
            raise FileNotFoundError(f"Configuration file not found: {file_path}")
        except yaml.YAMLError as e:
            raise ValueError(f"Error parsing YAML file {file_path}: {e}")

    def _process_paths_recursive(self, data):
        """Recursively replaces {ROOT_DIR} and {SRC_DIR} placeholders."""
        if isinstance(data, dict):
            for key, value in data.items():
                data[key] = self._process_paths_recursive(value)

        elif isinstance(data, list):
            for i, item in enumerate(data):
                data[i] = self._process_paths_recursive(item)

        elif isinstance(data, str):
            return (
                data.replace("{ROOT_DIR}", str(self.ROOT_DIR))
                    .replace("{SRC_DIR}", str(self.SRC_DIR))
            )

        return data

    def _process_paths(self):
        """Process all loaded configurations to resolve path placeholders."""
        self.interface = self._process_paths_recursive(self.interface)
        self.stream = self._process_paths_recursive(self.stream)

    def _generate_derived_values(self):
        """
        Generates values that were previously computed in the old .py config files.
        This makes them directly accessible from the config object.
        """
        # General values
        self.MODE = self.uav['operation_mode']
        self.MAX_UAV_COUNT = self.uav['general']['max_uav_count']
        self.RESCUE_UAV_INDEX = self.uav['general']['rescue_uav_index']

        # Connection settings based on mode
        conn_conf = self.uav['connection'][self.MODE]
        self.PROTOCOLS = []
        self.SERVER_HOSTS = []
        self.SERVER_PORTS = []
        self.CLIENT_PORTS = []

        if self.MODE == "simulation":
            self.PROTOCOLS = [conn_conf['protocol']] * self.MAX_UAV_COUNT
            self.SERVER_HOSTS = [conn_conf['server_host']] * self.MAX_UAV_COUNT
            self.SERVER_PORTS = [conn_conf['base_server_port'] + i for i in range(self.MAX_UAV_COUNT)]
            self.CLIENT_PORTS = [conn_conf['base_client_port'] + i for i in range(self.MAX_UAV_COUNT)]
        else: # real
            self.PROTOCOLS = [conn_conf['protocol']] * self.MAX_UAV_COUNT
            self.SERVER_HOSTS = conn_conf['server_hosts']
            self.SERVER_PORTS = [conn_conf['baudrate']] * self.MAX_UAV_COUNT
            self.CLIENT_PORTS = [conn_conf['base_client_port'] + i for i in range(self.MAX_UAV_COUNT)]

        self.SYSTEMS_ADDRESSES = [
            f"{proto}://{host}:{port}" if proto != 'serial' else f"{proto}://{host}"
            for proto, host, port in zip(self.PROTOCOLS, self.SERVER_HOSTS, self.SERVER_PORTS)
        ]

        # Stream paths
        self.DEFAULT_STREAM_VIDEO_PATHS = self._get_stream_paths()

        # Log paths
        self.DEFAULT_STREAM_VIDEO_LOG_PATHS = [
            f"{self.SRC_DIR}/logs/recordings/stream_log_uav_{i}_{self.NOW}.avi"
            for i in range(1, self.MAX_UAV_COUNT + 1)
        ]
        self.parameter_data_files = [
            f"{self.SRC_DIR}/data/parameters/param_uav_{i}.txt" for i in range(1, self.MAX_UAV_COUNT + 1)
        ]
        self.drone_current_pos_files = [
            f"{self.SRC_DIR}/logs/drone_current_pos/uav_{i}.txt" for i in range(1, self.MAX_UAV_COUNT + 1)
        ]

        # Model paths
        self.model_uav_paths = {
            i: self.stream['model']['yolo_path_template']
            for i in range(1, self.MAX_UAV_COUNT + 1)
        }

        # Screen sizes and default images
        self.screen_sizes = self.stream['display']['screen_sizes']
        self.noSignal_img_paths = {
            k: self.interface['assets']['default_images']['nosignal_template'].format(w=v['width'], h=v['height'])
            for k, v in self.screen_sizes.items()
        }
        self.pause_img_paths = {
            k: self.interface['assets']['default_images']['pause_template'].format(w=v['width'], h=v['height'])
            for k, v in self.screen_sizes.items()
        }
        self.pause_img_paths["all"] = self.interface['assets']['default_images']['pause_all']


    def _get_stream_paths(self):
        """Generates the list of stream video paths based on the default source type."""
        source_type = self.stream['source']['default_type']
        paths_config = self.stream['source']['paths']

        if source_type in ["streams", "videos"]:
            template = paths_config[source_type]
            return [template.format(i=i) for i in range(1, self.MAX_UAV_COUNT + 1)]
        elif source_type == "webcam":
            template = paths_config[source_type]
            return [template.format(i=i) for i in range(1, self.MAX_UAV_COUNT + 1)]
        elif source_type == "rtsp":
            return paths_config['rtsp']
        return []