#!/usr/bin/env python3
# filepath: /workspace/src/utils/mavsdk_server_utils.py
"""
MAVSDK Server Utilities

This module provides utilities for managing MAVSDK server processes,
including starting servers in new terminal windows and managing process lifecycle.
"""

import os
import shlex
import shutil
import signal
import subprocess
from pathlib import Path
from typing import List, Optional

import psutil

# Path to the MAVSDK server executable
ROOT_DIR = Path(__file__).resolve().parents[2]
MAVSDK_SERVER_PATH = Path(f"{ROOT_DIR}/dependencies/MAVSDK-Python/mavsdk/bin/mavsdk_server")


def is_running_in_docker() -> bool:
    """Check if the current process is running inside a Docker container."""
    return (
        os.environ.get("SW_UAV_DOCKER") == "1"
        or os.environ.get("SW_UAV_IN_DOCKER") == "1"
        or Path("/.dockerenv").exists()
    )


def detect_terminal_emulator(preferred: Optional[str] = None) -> str:
    """
    Detect the appropriate terminal emulator.
    - If running directly on host machine: prefer gnome-terminal (fallback to xfce4-terminal, xterm).
    - If running inside Docker: prefer xfce4-terminal (fallback to xterm, gnome-terminal).
    """
    if preferred and shutil.which(preferred):
        return preferred

    in_docker = is_running_in_docker()
    candidates = (
        ["xfce4-terminal", "xterm", "gnome-terminal", "tilix", "alacritty", "konsole"]
        if in_docker
        else ["gnome-terminal", "xfce4-terminal", "xterm", "tilix", "alacritty", "konsole"]
    )

    for term in candidates:
        if shutil.which(term):
            return term

    return "xfce4-terminal" if in_docker else "gnome-terminal"


def build_terminal_args(terminal_type: str, title: str, inner_command: str) -> List[str]:
    """
    Build argument list to launch inner_command in the specified terminal emulator.
    Using a list avoids shell quote escaping issues.
    """
    bash_script = f"cd '{ROOT_DIR}' && {inner_command}; exec /bin/bash"

    if "gnome-terminal" in terminal_type or "tilix" in terminal_type:
        return [terminal_type, f"--title={title}", "--", "/bin/bash", "-c", bash_script]
    elif "xfce4-terminal" in terminal_type:
        return [terminal_type, f"--title={title}", "-x", "/bin/bash", "-c", bash_script]
    elif "xterm" in terminal_type:
        return [terminal_type, "-T", title, "-e", "/bin/bash", "-c", bash_script]
    elif "konsole" in terminal_type:
        return [terminal_type, "--title", title, "-e", "/bin/bash", "-c", bash_script]
    else:
        return [terminal_type, "-e", "/bin/bash", "-c", bash_script]


def kill_process_tree(pid: int) -> None:
    """
    Kill a process and all its children recursively.
    
    Args:
        pid: Process ID to kill
    """
    try:
        parent = psutil.Process(pid)
        for child in parent.children(recursive=True):
            try:
                child.kill()
            except psutil.NoSuchProcess:
                pass
        parent.kill()
    except psutil.NoSuchProcess:
        pass  # Process already terminated
    except Exception as e:
        print(f"Error killing process {pid}: {e}")


def get_pids_by_cmdline(cmdline_args: List[str]) -> List[int]:
    """
    Find process IDs matching the given command line arguments.
    
    Args:
        cmdline_args: List of command line arguments to match
    
    Returns:
        List of matching process IDs
    """
    matching_pids = []
    
    for proc in psutil.process_iter(['pid', 'cmdline']):
        try:
            proc_cmdline = proc.info['cmdline']
            # Skip processes with no command line
            if not proc_cmdline:
                continue
                
            # Check if cmdline matches
            if proc_cmdline == cmdline_args:
                matching_pids.append(proc.info['pid'])
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
            
    return matching_pids


class MAVSDKServer:
    """
    Manages a MAVSDK server process in a separate terminal window.
    
    This class handles starting and stopping MAVSDK server instances,
    with appropriate connection settings.
    """
    # cspell: ignore MAVSDK
    
    def __init__(
        self, 
        id: int, 
        protocol: str, 
        server_host: str, 
        port: int, 
        bind_port: int,
        use_terminal: bool = True,
        terminal_type: Optional[str] = None
    ):
        """
        Initialize a MAVSDK server manager.
        
        Args:
            id: Server identifier
            protocol: Connection protocol ('udp', 'tcp', or 'serial')
            server_host: Host address to connect to
            port: MAVSDK server port
            bind_port: Vehicle connection port
            use_terminal: Whether to launch in a separate terminal window
            terminal_type: Terminal emulator to use (auto-detected if None)
        """
        self.id = id
        self.protocol = protocol
        self.server_host = server_host
        self.port = port
        self.bind_port = bind_port
        self.use_terminal = use_terminal
        self.terminal_type = detect_terminal_emulator(terminal_type)
        self.process = None
        
        # Build command based on whether MAVSDK server binary exists
        connection_url = f"{protocol}://{server_host}:{bind_port}"
        
        if MAVSDK_SERVER_PATH.exists():
            self.command = f"{MAVSDK_SERVER_PATH} {connection_url} -p {port}"
        else:
            self.command = (
                f"python3 src/mavsdk_server/mavsdk_server_shell.py {connection_url} "
                f"-p {port} -i {id}"
            )
            
        # Informational message
        self.init_msg = (
            f"[INFO] Starting MAVSDK server {id} with {connection_url} -p {port}"
        )
        
        # Build command args
        if use_terminal:
            title = f"MAVSDK Server UAV {id}"
            inner_cmd = f'echo "{self.init_msg}" && sleep 1 && {self.command}'
            self.cmd_args = build_terminal_args(self.terminal_type, title, inner_cmd)
        else:
            self.cmd_args = shlex.split(self.command)

    def start(self) -> None:
        """
        Start the MAVSDK server process.
        
        Launches the server either in a new terminal window or as a background process.
        """
        try:
            print(f"{self.init_msg} (terminal: {self.terminal_type if self.use_terminal else 'headless'})")
            
            # Start process directly with argument list
            self.process = subprocess.Popen(
                self.cmd_args,
                stdin=subprocess.PIPE if not self.use_terminal else None,
                stdout=subprocess.PIPE if not self.use_terminal else None,
                stderr=subprocess.PIPE if not self.use_terminal else None,
                start_new_session=True  # Prevent SIGINT from parent process
            )
            
        except Exception as e:
            print(f"Failed to start MAVSDK server {self.id}: {e}")
            print(f"Command was: {self.cmd_args}")

    def stop(self) -> None:
        """
        Stop the MAVSDK server process.
        
        Terminates the server process and any associated terminal window.
        """
        try:
            # If we have a direct process reference
            if self.process and self.process.pid:
                # Kill the process group to ensure all child processes are terminated
                try:
                    os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
                except ProcessLookupError:
                    pass
                
                # Wait briefly for graceful shutdown
                try:
                    self.process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
                    except ProcessLookupError:
                        pass
            
            # If using terminal, also find and kill any matching processes
            if self.use_terminal:
                for proc in psutil.process_iter(['pid', 'cmdline']):
                    try:
                        cmdline = proc.info.get('cmdline') or []
                        # Check if process relates to this UAV's MAVSDK server or terminal
                        if any(f"mavsdk_server_shell.py" in arg for arg in cmdline) and any(f"-p {self.port}" in " ".join(cmdline) or f"-i {self.id}" in " ".join(cmdline) for arg in cmdline):
                            kill_process_tree(proc.info['pid'])
                        elif any("mavsdk_server" in arg for arg in cmdline) and any(f"-p {self.port}" in " ".join(cmdline) for arg in cmdline):
                            kill_process_tree(proc.info['pid'])
                    except (psutil.NoSuchProcess, psutil.AccessDenied):
                        continue
                    
        except Exception as e:
            print(f"Error stopping MAVSDK server {self.id}: {e}")
        finally:
            self.process = None


def find_mavsdk_servers() -> List[int]:
    """
    Find all running MAVSDK server processes.
    
    Returns:
        List of process IDs for running MAVSDK servers
    """
    server_pids = []
    
    for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
        try:
            # Check process name and command line
            if proc.info['name'] == 'mavsdk_server' or (
                proc.info['cmdline'] and
                any('mavsdk_server' in arg for arg in proc.info['cmdline'])
            ):
                server_pids.append(proc.info['pid'])
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
            
    return server_pids


def terminate_all_mavsdk_servers() -> int:
    """
    Terminate all running MAVSDK server processes.
    
    Returns:
        Number of processes terminated
    """
    server_pids = find_mavsdk_servers()
    
    for pid in server_pids:
        kill_process_tree(pid)
        
    return len(server_pids)


# Example usage
if __name__ == "__main__":
    import time

    # Check for running servers
    existing_servers = find_mavsdk_servers()
    if existing_servers:
        print(f"Found {len(existing_servers)} running MAVSDK servers.")
        choice = input("Terminate them? (y/n): ")
        if choice.lower() == 'y':
            count = terminate_all_mavsdk_servers()
            print(f"Terminated {count} MAVSDK server processes.")
    
    # Start a test server
    test_server = MAVSDKServer(
        id=1,
        protocol="udpin",
        server_host="127.0.0.1",
        port=50051,
        bind_port=14540
    )
    
    try:
        print("Starting test server...")
        test_server.start()
        print("Server started. Press Ctrl+C to stop.")
        
        # Keep alive for testing
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nStopping server...")
    finally:
        test_server.stop()
        print("Server stopped.")
