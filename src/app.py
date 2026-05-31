import asyncio
import faulthandler
import os
import sys

faulthandler.enable(all_threads=True)

os.environ.setdefault("QTWEBENGINE_DISABLE_SANDBOX", "1")
os.environ.setdefault(
    "QTWEBENGINE_CHROMIUM_FLAGS",
    "--no-sandbox --disable-gpu --disable-gpu-sandbox --disable-seccomp-filter-sandbox",
)
os.environ.setdefault("QT_OPENGL", "software")
os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")
os.environ.setdefault("QT_XCB_GL_INTEGRATION", "none")
os.environ.setdefault("QT_AUTO_SCREEN_SCALE_FACTOR", "1")
os.environ.setdefault("QT_SCALE_FACTOR_ROUNDING_POLICY", "PassThrough")
if "UAV_QT_SCALE_FACTOR" in os.environ:
    os.environ.setdefault("QT_SCALE_FACTOR", os.environ["UAV_QT_SCALE_FACTOR"])

from PyQt5 import QtCore, QtWebEngine, QtWidgets
from asyncqt import QEventLoop

QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling)
QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps)
QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_ShareOpenGLContexts)
QtWebEngine.QtWebEngine.initialize()


def startup_trace(message: str) -> None:
    print(f"[startup] {message}", file=sys.stderr, flush=True)

os.environ.setdefault("QT_XCB_GL_INTEGRATION", "none")

class Runner:
    def __init__(self):
        startup_trace("creating QApplication")
        self.app = QtWidgets.QApplication(sys.argv)
        startup_trace("setting style")
        self.app.setStyle("Oxygen")
        startup_trace("creating async event loop")
        self.loop = QEventLoop(self.app)
        asyncio.set_event_loop(self.loop)
        startup_trace("importing main window")
        from interface_wrapper import App

        startup_trace("creating main window")
        self.window = App()
        startup_trace("main window created")

    def run(self):
        startup_trace("showing main window")
        self.window.show()
        with self.loop:
            pending = asyncio.all_tasks(loop=self.loop)
            for task in pending:
                task.cancel()
            sys.exit(self.loop.run_forever())


if __name__ == "__main__":
    runner = Runner()
    runner.run()
