#Đảm nhiệm trách nhiệm thiết lập RTSP/UDP stream, YOLO inference và render ảnh (convert_cv2qt) lên màn hình giao diện.

import asyncio
import cv2
import os
import numpy as np
from PyQt5.QtCore import pyqtSlot
from utils.qt_utils import convert_cv2qt

__current_path__ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

class StreamController:
    def __init__(self, app):
        self.app = app

    def _create_streaming_threads(self, uav_indexes=None) -> None:
        
        try:
            # If no specific indexes provided, use all available UAVs
            uav_indexes = range(1, self.app.config.MAX_UAV_COUNT + 1) if uav_indexes is None else uav_indexes
            
            for uav_index in uav_indexes:
                # Skip UAVs that aren't eligible for streaming
                if not self._can_stream(uav_index):
                    continue
                    
                # Configure stream settings
                stream_config = self._create_stream_config(uav_index)
                
                # Determine detection model if enabled
                detection_model = (
                    self.app.uav_detection_models[uav_index - 1]
                    if self.app.UAVs[uav_index]["detection_enable"]
                    else None
                )
                
                # Create the streaming thread
                self.app.uav_stream_threads[uav_index - 1] = StreamQtThread(
                    uav_index=uav_index,
                    stream_config=stream_config,
                    detection_model=detection_model
                )
                
                # Log the stream configuration
                self._log_stream_creation(uav_index)
                
                # Safely connect signal to slot (disconnect first to prevent duplicate connections)
                asyncio.create_task(self.app._connect_stream_signal(uav_index))
                
        except Exception as e:
            self.app.logger.log(repr(e), level="error")
            self.app.popup_msg(
                type_msg="Error", 
                msg=repr(e), 
                src_msg="_create_streaming_threads()"
            )

    def _create_stream_config(self, uav_index):
        """Create stream configuration dictionary for a UAV"""
        
        # Capture settings
        capture = {
            "index": uav_index,
            "address": self.app.UAVs[uav_index]["streaming_address"],
            "width": self.app.config.stream['source']['default_size']['width'],
            "height": self.app.config.stream['source']['default_size']['height'],
            "fps": self.app.config.stream['source']['default_fps'],
        }
        
        # Recording settings
        writer = {
            "index": uav_index,
            "enable": self.app.UAVs[uav_index]["recording_enable"],
            "filename": self.app.config.DEFAULT_STREAM_VIDEO_LOG_PATHS[uav_index - 1],
            "fourcc": self.app.config.stream['source']['fourcc'],
            "frameSize": (self.app.config.stream['source']['default_size']['width'],
                          self.app.config.stream['source']['default_size']['height']),
        }
        
        return {
            "capture": capture,
            "writer": writer,
        }

    def _log_stream_creation(self, uav_index):
        """Log the creation of a streaming thread"""
        recording_path = (
            os.path.relpath(self.app.config.DEFAULT_STREAM_VIDEO_LOG_PATHS[uav_index - 1], __current_path__)
            if self.app.UAVs[uav_index]["recording_enable"]
            else 'None'
        )
        
        self.app.logger.log(
            f"UAV-{uav_index} stream started:\n"
            f"  -- Capture stream from {os.path.relpath(self.app.UAVs[uav_index]['streaming_address'], __current_path__)}\n"
            f"  -- Save recording to {recording_path}",
            level="info",
        )
        
        self.app.logger.log(f"UAV-{uav_index} streaming thread created!", level="info")

    @pyqtSlot(np.ndarray, list)
    def stream_on_uav_screen(self, annotated_frame=None, results=None) -> None:
    
        if not results:
            self.app.logger.log("Received empty results in stream handler", level="warning")
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
            asyncio.create_task(self.app.update_uav_screen_view(
                uav_index, streaming_frame, screen_name=self.app.config.stream['display']['default_screen']
            ))
            
            # Process detection results if available and detection is enabled
            if self.app.UAVs[uav_index]["detection_enable"] and detected_results:
                asyncio.create_task(self.app._process_detection_results(uav_index, annotated_frame, detected_results))
    
                
        except Exception as e:
            # Update status and show error message
            self.app.UAVs[uav_index]["status"]["streaming_status"] = False
            self.app.logger.log(f"Stream display error for UAV {uav_index}: {repr(e)}", level="error")
            self.app.popup_msg(
                f"Stream display error: {repr(e)}",
                src_msg="stream_on_uav_screen",
                type_msg="error",
            )

    def _can_stream(self, uav_index):
        """Check if UAV is eligible for stream display."""
        return (
            self.app._check_uav_connection(uav_index=uav_index, strictly=False) and 
            self.app.UAVs[uav_index]["streaming_enable"]
        )

    def _should_process_frame(self, uav_index, current_fps):
        """Apply frame rate limiting to avoid overloading the UI."""
        # Calculate the frame skip rate to achieve target FPS
        max_frame_cnt = max(1, current_fps // self.app.config.stream['source']['default_fps'])
        
        # Increment the frame counter for this UAV
        self.app.uav_stream_frame_cnt[uav_index - 1] += 1
        
        # Process frame if it's time to display based on our rate limiting
        return self.app.uav_stream_frame_cnt[uav_index - 1] % max_frame_cnt == 0

    def _select_frame_type(self, uav_index, frame, annotated_frame):
        """Select which frame to display based on detection settings."""
        # Use annotated frame if detection is enabled, otherwise use raw frame
        return annotated_frame if self.app.UAVs[uav_index]["detection_enable"] else frame

    def uav_toggle_camera_callback(self, uav_index) -> None:
        # Handle the case of toggling all UAVs
        if uav_index not in range(1, self.app.config.MAX_UAV_COUNT + 1):
            for i in range(1, self.app.config.MAX_UAV_COUNT + 1):
                # Only toggle UAVs that are eligible for streaming
                if self._can_stream(i):
                    self.uav_toggle_camera_callback(i)
            return
        
        # Skip if UAV is not eligible for streaming
        if not self._can_stream(uav_index):
            self.app.logger.log(
                f"Camera toggle skipped for UAV {uav_index}: not eligible for streaming",
                level="warning"
            )
            return
            
        try:
            # Determine current streaming state
            is_streaming = self.app.UAVs[uav_index]["status"]["streaming_status"]
            
            if not is_streaming:
                # Start streaming
                if self.app.uav_stream_threads[uav_index - 1] is None:
                    self._create_streaming_threads(uav_indexes=[uav_index])
                    
                self.app.uav_stream_threads[uav_index - 1].start()
                self.app.UAVs[uav_index]["status"]["streaming_status"] = True
                
                self.app.logger.log(f"UAV-{uav_index} streaming started", level="info")
                self.app.ui.btn_toggle_camera.setStyleSheet("background-color: green")
            else:
                # Stop streaming
                self.app.uav_stream_threads[uav_index - 1].stop()
                self.app.UAVs[uav_index]["status"]["streaming_status"] = False
                
                self.app.logger.log(f"UAV-{uav_index} streaming stopped", level="info")
                self.app.ui.btn_toggle_camera.setStyleSheet("background-color: red")
            
            # Update thread status
            self.app.uav_stream_threads[uav_index - 1].isRunning = self.app.UAVs[uav_index]["status"][
                "streaming_status"
            ]
            
        except Exception as e:
            self.app.logger.log(f"Camera toggle error: {repr(e)}", level="error")
            self.app.popup_msg(
                f"Error toggling camera: {repr(e)}", 
                src_msg="uav_toggle_camera_callback",
                type_msg="Error"
            )

