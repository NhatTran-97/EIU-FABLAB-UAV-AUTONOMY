import sys
import os
import subprocess
from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget, QHBoxLayout
from PyQt6 import uic
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtCore import QUrl, QTimer
from PyQt6.QtWebChannel import QWebChannel
from PyQt6.QtCore import QUrl, QTimer, Qt, QEvent


from lora_bridge import LoraBridge
from control import GroundController


os.environ["QTWEBENGINE_DICTIONARIES_PATH"] = "/dev/null"

os.environ.setdefault(
    "QTWEBENGINE_CHROMIUM_FLAGS",
    "--disable-features=VaapiVideoDecoder,VaapiVideoEncoder",
)

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        base_dir = os.path.dirname(os.path.abspath(__file__))
        index_path = os.path.join(base_dir, "index")
        ui_path = os.path.join(base_dir, "ui", "main.ui")

        if not os.path.exists(os.path.join(index_path, "map.html")):
            print(f"❌ Warning: map.html not found at {index_path}")

        self.http_port = int(os.environ.get("GROUND_GUI_HTTP_PORT", "8000"))
        self.http_process = subprocess.Popen(
            ["python3", "-m", "http.server", str(self.http_port)],
            cwd=index_path,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        print(f"🚀 Server running at: {index_path} (http://localhost:{self.http_port})")

        uic.loadUi(ui_path, self)
        self.setWindowTitle("Ground Control Station") 
        
        if cw := self.centralWidget():
            cw.setContentsMargins(0, 0, 0, 0)
            if cw.layout():
                cw.layout().setContentsMargins(0, 0, 0, 0)
                cw.layout().setSpacing(0)

        self.bridge = LoraBridge()
        self.browser = QWebEngineView(self)
        self.channel = QWebChannel()
        self.channel.registerObject("bridge", self.bridge)  
        self.browser.page().setWebChannel(self.channel)

        QTimer.singleShot(1000, lambda: self.browser.load(QUrl(f"http://localhost:{self.http_port}/map.html")))

        placeholder = self.findChild(QWidget, "load_map_widget")
        if placeholder:
            parent = placeholder.parent()
            if parent and parent.layout():
                layout = parent.layout()
                layout.replaceWidget(placeholder, self.browser)
                placeholder.deleteLater()

        main_layout = self.centralWidget().layout()
        if isinstance(main_layout, QHBoxLayout):
            main_layout.setStretch(0, 0)
            main_layout.setStretch(1, 10)

        serial_port = os.environ.get("GROUND_GUI_SERIAL_PORT", "/dev/lora_drone")
        baudrate = int(os.environ.get("GROUND_GUI_BAUDRATE", "9600"))
        self.controller = GroundController(port=serial_port, baudrate=baudrate, gui_bridge=self.bridge)
        self.bridge.set_controller(self.controller)

    def closeEvent(self, event):
        try:
            if getattr(self, "controller", None):
                self.controller.stop()
        except Exception as e:
            print(f"⚠️ controller stop error: {e}")
        try:
            if getattr(self, "http_process", None):
                self.http_process.terminate()
        except Exception as e:
            print(f"⚠️ http server terminate error: {e}")
        super().closeEvent(event)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    try:
        sys.exit(app.exec())
    except KeyboardInterrupt:
        print("⛔ Process terminated by user (Ctrl + C)")
