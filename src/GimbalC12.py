"""
C12 Qt5 Dashboard (compact UI)
- Giao diện gọn hơn, dark theme
- Dual RTSP streams (optical + thermal) bằng OpenCV
- PTZ / góc / zoom / record + joystick yaw/pitch
"""

import socket
import sys
import math
from PyQt5 import QtWidgets, QtGui, QtCore
import cv2
import numpy as np
from typing import Optional


# ====================== MINI SDK C12 ======================
class C12Camera:
    def __init__(self, ip: str, port: int = 5000):
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.3)

    def set_ip(self, ip: str, port: Optional[int] = None):
        self.ip = ip
        if port is not None:
            self.port = port

    @staticmethod
    def _crc(frame_wo_crc: str) -> str:
        total = sum(frame_wo_crc.encode("ascii")) & 0xFF
        return f"{total:02X}"

    def _build_cmd(self, src: str, dst: str, rw: str, flag: str, data_hex: str) -> str:
        data_hex = data_hex.upper().replace(" ", "")
        length_hex = f"{len(data_hex):X}"
        frame_wo_crc = f"#TP{src}{dst}{length_hex}{rw}{flag}{data_hex}"
        crc = self._crc(frame_wo_crc)
        return frame_wo_crc + crc

    def _send(self, cmd: str):
        try:
            self.sock.sendto(cmd.encode("ascii"), (self.ip, self.port))
        except Exception as e:
            # socket errors should not crash GUI
            print("Socket send error:", e)

    # PTZ basic
    def _ptz(self, code: int) -> str:
        data_hex = f"{code:02X}"
        cmd = self._build_cmd("U", "G", "w", "PTZ", data_hex)
        self._send(cmd)
        return cmd

    def move_forward(self) -> str: return self._ptz(0x01)
    def move_backward(self) -> str: return self._ptz(0x02)
    def move_left(self) -> str: return self._ptz(0x03)
    def move_right(self) -> str: return self._ptz(0x04)
    def center_ptz(self) -> str: return self._ptz(0x05)
    def stop(self) -> str: return self._ptz(0x00)

    # Angle control
    @staticmethod
    def _angle_to_hex(deg: float) -> str:
        val = int(deg * 100)
        if val < 0:
            val = (1 << 16) + val
        return f"{val & 0xFFFF:04X}"

    @staticmethod
    def _speed_angle_to_hex(speed_deg_s: float) -> str:
        v = max(0, min(99, int(speed_deg_s * 10)))
        return f"{v & 0xFF:02X}"

    def set_yaw_pitch(self, yaw_deg: float, pitch_deg: float, speed_deg_s: float = 10.0) -> str:
        yaw_hex = self._angle_to_hex(yaw_deg)
        pitch_hex = self._angle_to_hex(pitch_deg)
        spd_hex = self._speed_angle_to_hex(speed_deg_s)
        data_hex = yaw_hex + spd_hex + pitch_hex + spd_hex
        cmd = self._build_cmd("U", "G", "w", "GAM", data_hex)
        self._send(cmd)
        return cmd

    def align_center_angle(self) -> str:
        return self.set_yaw_pitch(0.0, 0.0, 15.0)

    def look_down_90(self) -> str:
        return self.set_yaw_pitch(0.0, -90.0, 15.0)

    # Record / Photo
    def start_record(self) -> str:
        cmd = self._build_cmd("U", "D", "w", "REC", "01")
        self._send(cmd)
        return cmd

    def stop_record(self) -> str:
        cmd = self._build_cmd("U", "D", "w", "REC", "00")
        self._send(cmd)
        return cmd

    def toggle_record(self) -> str:
        cmd = self._build_cmd("U", "D", "w", "REC", "0A")
        self._send(cmd)
        return cmd

    def take_photo(self) -> str:
        cmd = self._build_cmd("U", "D", "w", "CAP", "01")
        self._send(cmd)
        return cmd

    # Digital zoom
    def set_zoom_level(self, level: int) -> str:
        if level < 1:
            level = 1
        if level > 4:
            level = 4
        data_hex = f"0{level:X}"
        cmd = self._build_cmd("U", "D", "w", "DZM", data_hex)
        self._send(cmd)
        return cmd

    def zoom_in(self) -> str:
        cmd = self._build_cmd("U", "D", "w", "DZM", "0A")
        self._send(cmd)
        return cmd

    def zoom_out(self) -> str:
        cmd = self._build_cmd("U", "D", "w", "DZM", "0B")
        self._send(cmd)
        return cmd


# ====================== Simple Joystick Widget ======================
class Joystick(QtWidgets.QWidget):
    moved = QtCore.pyqtSignal(float, float)  # x, y from -1..1

    def __init__(self, size=150, parent=None):
        super().__init__(parent)
        self.setFixedSize(size, size)
        self._radius = size // 2
        self._knob_radius = max(12, size // 9)
        self._knob_pos = QtCore.QPoint(self._radius, self._radius)
        self._active = False

    def paintEvent(self, event):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        center = QtCore.QPoint(self._radius, self._radius)

        # outer circle
        p.setPen(QtGui.QPen(QtGui.QColor('#444'), 2))
        p.setBrush(QtGui.QBrush(QtGui.QColor('#202020')))
        p.drawEllipse(center, self._radius - 2, self._radius - 2)

        # cross lines
        p.setPen(QtGui.QPen(QtGui.QColor('#555'), 1, QtCore.Qt.DashLine))
        p.drawLine(center.x() - self._radius + 8, center.y(),
                   center.x() + self._radius - 8, center.y())
        p.drawLine(center.x(), center.y() - self._radius + 8,
                   center.x(), center.y() + self._radius - 8)

        # knob
        p.setBrush(QtGui.QBrush(QtGui.QColor('#66aaff' if self._active else '#aaaaaa')))
        p.setPen(QtGui.QPen(QtGui.QColor('#222')))
        p.drawEllipse(self._knob_pos, self._knob_radius, self._knob_radius)

    def mousePressEvent(self, e):
        self._active = True
        self._update_knob(e.pos())

    def mouseMoveEvent(self, e):
        if self._active:
            self._update_knob(e.pos())

    def mouseReleaseEvent(self, e):
        self._active = False
        # return to center
        self._knob_pos = QtCore.QPoint(self._radius, self._radius)
        self.moved.emit(0.0, 0.0)
        self.update()

    def _update_knob(self, pos: QtCore.QPoint):
        dx = pos.x() - self._radius
        dy = pos.y() - self._radius
        dist = math.hypot(dx, dy)
        maxd = self._radius - self._knob_radius - 4
        if dist > maxd and dist > 0:
            scale = maxd / dist
            dx *= scale
            dy *= scale
        self._knob_pos = QtCore.QPoint(int(self._radius + dx), int(self._radius + dy))
        nx = dx / maxd if maxd else 0.0
        ny = -dy / maxd if maxd else 0.0  # invert Y so up is positive
        self.moved.emit(nx, ny)
        self.update()


# ====================== Main Window ======================
class C12MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowFlags(QtCore.Qt.FramelessWindowHint | QtCore.Qt.Window)

        self.resize(900, 550)

        # biến phục vụ kéo cửa sổ
        self._drag_pos = None
        # defaults (customize these)
        self.default_ip = '192.168.144.110'
        self.default_port = 5000
        self._update_rtsp_urls()

        self.camera = C12Camera(self.default_ip, self.default_port)

        self.cap_opt = None
        self.cap_thm = None

        self._apply_style()
        self._build_ui()

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._on_timer)
        self.timer_interval_ms = 40

        # For joystick-based angle control
        self.joystick_yaw = 0.0
        self.joystick_pitch = 0.0

        # giảm spam log cho joystick
        self._gam_log_counter = 0
    
    def _update_rtsp_urls(self):
        """Cập nhật URL RTSP theo IP hiện tại."""
        self.rtsp_optical = f'rtsp://{self.default_ip}:554/stream=1'
        self.rtsp_thermal = f'rtsp://{self.default_ip}:555/stream=2'

    def _apply_style(self):
        style = """
        /* Nền chính */
        QMainWindow {
            background-color: #1565C0;          /* blue 800 */
        }

        /* Label chữ */
        QLabel {
            color: #E3F2FD;                     /* very light blue */
        }

        /* Khung group box */
        QGroupBox {
            background-color: rgba(255,255,255,0.05);
            border: 1px solid #90CAF9;          /* CHÚ Ý: có 'solid' */
            border-radius: 6px;
            margin-top: 10px;
            padding-top: 12px;
            font-weight: bold;
            color: #E3F2FD;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 8px;
            padding: 0 3px;
        }

        /* Nút bấm */
        QPushButton {
            background-color: #64B5F6;          /* blue 300 */
            border-radius: 4px;
            border: 1px solid #0D47A1;
            padding: 4px 8px;
            color: #0B2540;
            font-weight: 500;
        }
        QPushButton:hover {
            background-color: #42A5F5;
        }
        QPushButton:pressed {
            background-color: #1E88E5;
            color: #E3F2FD;
        }

        /* Ô nhập & log */
        QLineEdit, QTextEdit {
            background-color: #E3F2FD;
            color: #0B2540;
            border-radius: 4px;
            border: 1px solid #90CAF9;
        }

        /* Slider */
        QSlider::groove:horizontal {
            background: #90CAF9;
            height: 6px;
            border-radius: 3px;
        }
        QSlider::handle:horizontal {
            background: #0D47A1;
            width: 14px;
            margin: -4px 0px;
            border-radius: 7px;
        }

        /* Tab video */
        QTabWidget::pane {
            border: 1px solid #0D47A1;
            border-radius: 4px;
            background-color: #E3F2FD;
        }
        QTabBar::tab {
            background: #1976D2;
            color: #E3F2FD;
            padding: 4px 10px;
            border-top-left-radius: 4px;
            border-top-right-radius: 4px;
            min-width: 80px;
        }
        QTabBar::tab:selected {
            background: #E3F2FD;
            color: #0B2540;
        }

        /* Title bar custom (frameless) */
        QFrame#TitleBar {
            background-color: #0D47A1;
        }
        QFrame#TitleBar QLabel {
            color: #E3F2FD;
            font-weight: bold;
        }
        QFrame#TitleBar QPushButton {
            background-color: transparent;
            border: none;
            color: #E3F2FD;
            font-size: 14px;
        }
        QFrame#TitleBar QPushButton:hover {
            background-color: rgba(255,255,255,0.15);
        }
        QFrame#TitleBar QPushButton:pressed {
            background-color: rgba(255,255,255,0.25);
        }
        """
        self.setStyleSheet(style)

    def mousePressEvent(self, event: QtGui.QMouseEvent):
        # Nếu click chuột trái trong vùng title bar (cao 32px đầu tiên)
        if event.button() == QtCore.Qt.LeftButton and event.pos().y() <= 32:
            self._drag_pos = event.globalPos() - self.frameGeometry().topLeft()
            event.accept()
        else:
            super().mousePressEvent(event)

    def mouseMoveEvent(self, event: QtGui.QMouseEvent):
        if self._drag_pos is not None and event.buttons() & QtCore.Qt.LeftButton:
            self.move(event.globalPos() - self._drag_pos)
            event.accept()
        else:
            super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event: QtGui.QMouseEvent):
        self._drag_pos = None
        super().mouseReleaseEvent(event)
    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        main_v = QtWidgets.QVBoxLayout(central)
        main_v.setContentsMargins(0, 0, 0, 0)
        main_v.setSpacing(0)

        # ===== TITLE BAR CUSTOM =====
        title_bar = QtWidgets.QFrame()
        title_bar.setObjectName("TitleBar")
        title_bar.setFixedHeight(32)

        title_layout = QtWidgets.QHBoxLayout(title_bar)
        title_layout.setContentsMargins(10, 0, 10, 0)
        title_layout.setSpacing(8)

        self.title_label = QtWidgets.QLabel("Gimbal C12")
        self.title_label.setAlignment(QtCore.Qt.AlignCenter)
        title_layout.addStretch()
        title_layout.addWidget(self.title_label)
        title_layout.addStretch()

        btn_min = QtWidgets.QPushButton("—")
        btn_min.setFixedSize(28, 22)
        btn_min.clicked.connect(self.showMinimized)
        title_layout.addWidget(btn_min)

        btn_close = QtWidgets.QPushButton("×")
        btn_close.setFixedSize(28, 22)
        btn_close.clicked.connect(self.close)
        title_layout.addWidget(btn_close)

        main_v.addWidget(title_bar)
        # -------- Connection group --------
        conn_group = QtWidgets.QGroupBox("Kết nối Video")
        main_v.addWidget(conn_group)
        conn_layout = QtWidgets.QHBoxLayout(conn_group)
        conn_layout.setContentsMargins(10, 6, 10, 6)
        conn_layout.setSpacing(8)

        # IP
        conn_layout.addWidget(QtWidgets.QLabel("IP:"))
        self.ip_edit = QtWidgets.QLineEdit(self.default_ip)
        self.ip_edit.setFixedWidth(150)
        conn_layout.addWidget(self.ip_edit)

        # Port
        conn_layout.addSpacing(10)
        conn_layout.addWidget(QtWidgets.QLabel("UDP port:"))
        self.port_edit = QtWidgets.QLineEdit(str(self.default_port))
        self.port_edit.setFixedWidth(80)
        conn_layout.addWidget(self.port_edit)

        # Buttons trên cùng 1 hàng
        conn_layout.addSpacing(20)
        btn_apply = QtWidgets.QPushButton("Áp dụng IP/Port")
        btn_apply.clicked.connect(self.apply_connection)
        conn_layout.addWidget(btn_apply)

        btn_start = QtWidgets.QPushButton("Start 2 video")
        btn_start.clicked.connect(self.start_video)
        conn_layout.addWidget(btn_start)

        btn_stop = QtWidgets.QPushButton("Stop video")
        btn_stop.clicked.connect(self.stop_video)
        conn_layout.addWidget(btn_stop)

        # đẩy mọi thứ sát trái, chừa khoảng trống bên phải
        conn_layout.addStretch()

        # -------- Middle: Video + Controls --------
        mid_h = QtWidgets.QHBoxLayout()
        mid_h.setSpacing(10)
        main_v.addLayout(mid_h, stretch=1)

        # ---- Video group ----
        video_group = QtWidgets.QGroupBox("Video")
        mid_h.addWidget(video_group, stretch=3)
        video_layout = QtWidgets.QVBoxLayout(video_group)

        self.tab_video = QtWidgets.QTabWidget()
        video_layout.addWidget(self.tab_video)

        # Optical tab
        opt_widget = QtWidgets.QWidget()
        opt_layout = QtWidgets.QVBoxLayout(opt_widget)
        self.lbl_opt = QtWidgets.QLabel("Optical")
        self.lbl_opt.setMinimumSize(320, 240)
        self.lbl_opt.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_opt.setStyleSheet("background:#ffffff; color:#0b2540; border:1px solid #90caf9;")
        opt_layout.addWidget(self.lbl_opt)
        self.tab_video.addTab(opt_widget, "Optical")

        # Thermal tab
        thm_widget = QtWidgets.QWidget()
        thm_layout = QtWidgets.QVBoxLayout(thm_widget)
        self.lbl_thm = QtWidgets.QLabel("Thermal")
        self.lbl_thm.setMinimumSize(320, 240)
        self.lbl_thm.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_thm.setStyleSheet("background:#ffffff; color:#0b2540; border:1px solid #90caf9;")
        thm_layout.addWidget(self.lbl_thm)
        self.tab_video.addTab(thm_widget, "Thermal")

        # ---- Controls group ----
        ctrl_group = QtWidgets.QGroupBox("Gimbal / PTZ Controls")
        mid_h.addWidget(ctrl_group, stretch=2)
        ctrl_v = QtWidgets.QVBoxLayout(ctrl_group)
        ctrl_v.setSpacing(6)

        # PTZ buttons
        ptz_group = QtWidgets.QGroupBox("PTZ")
        ctrl_v.addWidget(ptz_group)
        grid = QtWidgets.QGridLayout(ptz_group)
        grid.setHorizontalSpacing(4)
        grid.setVerticalSpacing(4)

        btn_up = QtWidgets.QPushButton('↑')
        btn_up.clicked.connect(lambda: self._log_cmd(self.camera.move_forward()))
        grid.addWidget(btn_up, 0, 1)

        btn_left = QtWidgets.QPushButton('←')
        btn_left.clicked.connect(lambda: self._log_cmd(self.camera.move_left()))
        grid.addWidget(btn_left, 1, 0)

        btn_stop = QtWidgets.QPushButton('⏹')
        btn_stop.clicked.connect(lambda: self._log_cmd(self.camera.stop()))
        grid.addWidget(btn_stop, 1, 1)

        btn_right = QtWidgets.QPushButton('→')
        btn_right.clicked.connect(lambda: self._log_cmd(self.camera.move_right()))
        grid.addWidget(btn_right, 1, 2)

        btn_down = QtWidgets.QPushButton('↓')
        btn_down.clicked.connect(lambda: self._log_cmd(self.camera.move_backward()))
        grid.addWidget(btn_down, 2, 1)

        # Angle presets
        angle_group = QtWidgets.QGroupBox("Góc nhanh")
        ctrl_v.addWidget(angle_group)
        angle_h = QtWidgets.QHBoxLayout(angle_group)
        btn_center = QtWidgets.QPushButton('Về tâm PTZ')
        btn_center.clicked.connect(lambda: self._log_cmd(self.camera.center_ptz()))
        btn_center_angle = QtWidgets.QPushButton('Góc 0° / 0°')
        btn_center_angle.clicked.connect(lambda: self._log_cmd(self.camera.align_center_angle()))
        btn_lookdown = QtWidgets.QPushButton('Nhìn xuống -90°')
        btn_lookdown.clicked.connect(lambda: self._log_cmd(self.camera.look_down_90()))
        angle_h.addWidget(btn_center)
        angle_h.addWidget(btn_center_angle)
        angle_h.addWidget(btn_lookdown)

        # Zoom
        zoom_group = QtWidgets.QGroupBox("Zoom")
        ctrl_v.addWidget(zoom_group)
        zoom_h = QtWidgets.QHBoxLayout(zoom_group)
        zoom_h.addWidget(QtWidgets.QPushButton('Zoom +', clicked=lambda: self._log_cmd(self.camera.zoom_in())))
        zoom_h.addWidget(QtWidgets.QPushButton('Zoom -', clicked=lambda: self._log_cmd(self.camera.zoom_out())))
        zoom_h.addWidget(QtWidgets.QLabel('Level (1–4):'))
        self.zoom_edit = QtWidgets.QLineEdit('1')
        self.zoom_edit.setFixedWidth(40)
        zoom_h.addWidget(self.zoom_edit)
        zoom_h.addWidget(QtWidgets.QPushButton('Set', clicked=self.on_set_zoom))
        zoom_h.addStretch()

        # Record / photo
        rec_group = QtWidgets.QGroupBox("Record / Ảnh")
        ctrl_v.addWidget(rec_group)
        rec_h = QtWidgets.QHBoxLayout(rec_group)
        rec_h.addWidget(QtWidgets.QPushButton('Start REC', clicked=lambda: self._log_cmd(self.camera.start_record())))
        rec_h.addWidget(QtWidgets.QPushButton('Stop REC', clicked=lambda: self._log_cmd(self.camera.stop_record())))
        rec_h.addWidget(QtWidgets.QPushButton('Toggle', clicked=lambda: self._log_cmd(self.camera.toggle_record())))
        rec_h.addWidget(QtWidgets.QPushButton('Photo', clicked=lambda: self._log_cmd(self.camera.take_photo())))

        # Joystick + speed
        joy_group = QtWidgets.QGroupBox("Joystick yaw/pitch")
        ctrl_v.addWidget(joy_group)
        joy_v = QtWidgets.QVBoxLayout(joy_group)

        self.joystick = Joystick(150)
        self.joystick.moved.connect(self.on_joystick_moved)
        joy_v.addWidget(self.joystick, alignment=QtCore.Qt.AlignCenter)

        spd_h = QtWidgets.QHBoxLayout()
        spd_h.addWidget(QtWidgets.QLabel('Speed:'))
        self.speed_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.speed_slider.setRange(1, 50)
        self.speed_slider.setValue(10)
        spd_h.addWidget(self.speed_slider)
        joy_v.addLayout(spd_h)

        ctrl_v.addStretch()

        # -------- Logs --------
        log_group = QtWidgets.QGroupBox("Log")
        main_v.addWidget(log_group, stretch=1)
        log_layout = QtWidgets.QVBoxLayout(log_group)
        self.log_text = QtWidgets.QTextEdit()
        self.log_text.setReadOnly(True)
        log_layout.addWidget(self.log_text)

    # ---------- Logic ----------
    def apply_connection(self):
        ip = self.ip_edit.text().strip()
        try:
            port = int(self.port_edit.text().strip())
        except Exception:
            self._append_log('[ERR] Port không hợp lệ')
            return
        self.camera.set_ip(ip, port)
        self._append_log(f'[INFO] Đã áp dụng IP={ip}, port={port}')

    def on_set_zoom(self):
        try:
            lvl = int(self.zoom_edit.text())
        except Exception:
            self._append_log('[ERR] Zoom level không hợp lệ')
            return
        cmd = self.camera.set_zoom_level(lvl)
        self._append_log(f'[CMD] {cmd}')

    def _log_cmd(self, cmd: str):
        self._append_log(f'[CMD] {cmd}')

    def _append_log(self, msg: str):
        self.log_text.append(msg)

    # Video control
    def start_video(self):
        if self.cap_opt:
            try:
                self.cap_opt.release()
            except:
                pass
        if self.cap_thm:
            try:
                self.cap_thm.release()
            except:
                pass
        self.cap_opt = cv2.VideoCapture(self.rtsp_optical)
        self.cap_thm = cv2.VideoCapture(self.rtsp_thermal)
        if not self.cap_opt.isOpened():
            self._append_log('[WARN] Không mở được optical stream')
            self.cap_opt = None
        if not self.cap_thm.isOpened():
            self._append_log('[WARN] Không mở được thermal stream')
            self.cap_thm = None
        if self.cap_opt or self.cap_thm:
            self.timer.start(self.timer_interval_ms)
            self._append_log('[INFO] Start video')

    def stop_video(self):
        self.timer.stop()
        if self.cap_opt:
            self.cap_opt.release()
            self.cap_opt = None
        if self.cap_thm:
            self.cap_thm.release()
            self.cap_thm = None
        self._append_log('[INFO] Stop video')

    def _on_timer(self):
        def draw_frame(cap, label):
            if cap is None:
                return
            ret, frame = cap.read()
            if not ret:
                return
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = frame.shape
            bytes_per_line = ch * w
            qimg = QtGui.QImage(frame.data, w, h, bytes_per_line, QtGui.QImage.Format.Format_RGB888)
            pix = QtGui.QPixmap.fromImage(qimg)
            label.setPixmap(pix.scaled(label.width(), label.height(), QtCore.Qt.KeepAspectRatio))

        draw_frame(self.cap_opt, self.lbl_opt)
        draw_frame(self.cap_thm, self.lbl_thm)

        # Joystick angle control
        if abs(self.joystick_yaw) > 0.001 or abs(self.joystick_pitch) > 0.001:
            speed = self.speed_slider.value() / 1.0
            yaw_deg = self.joystick_yaw * 30.0
            pitch_deg = self.joystick_pitch * 30.0
            cmd = self.camera.set_yaw_pitch(yaw_deg, pitch_deg, speed)

            # tránh spam log: chỉ log mỗi 10 lần
            self._gam_log_counter += 1
            if self._gam_log_counter >= 10:
                self._gam_log_counter = 0
                self._append_log(
                    f'[GAM] Y={yaw_deg:.1f} P={pitch_deg:.1f} S={speed:.1f} -> {cmd}'
                )

    # Joystick handler
    def on_joystick_moved(self, nx, ny):
        # nx, ny in -1..1 (nx -> yaw, ny -> pitch)
        self.joystick_yaw = nx * 1.0
        self.joystick_pitch = ny * 1.0


# ====================== RUN ======================
def main():
    app = QtWidgets.QApplication(sys.argv)
    win = C12MainWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
