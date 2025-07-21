import ctypes
import sys
import os
import subprocess

from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QSizePolicy, QHBoxLayout)
from PyQt6 import uic
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtCore import QUrl, QTimer
from PyQt6.QtGui import QGuiApplication

# Khắc phục lỗi dictionary của QtWebEngine
os.environ["QTWEBENGINE_DICTIONARIES_PATH"] = "/dev/null"


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi("ui/main.ui", self)

        self.http_process = subprocess.Popen(
            ["python3", "-m", "http.server", "8000"],cwd=os.path.abspath("index"),stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        print("🚀 Đã khởi động HTTP server tại http://localhost:8000")

        placeholder = self.findChild(QWidget, "load_map_widget")

        if placeholder is None:
            print("❌ Không tìm thấy widget 'web'")
            return

        self.browser = QWebEngineView(self)
        QTimer.singleShot(300, lambda: self.browser.load( QUrl("http://localhost:8000/map.html")))

        parent = placeholder.parent()
        if parent is not None and parent.layout() is not None:
            layout = parent.layout()
            layout.replaceWidget(placeholder, self.browser)
            placeholder.deleteLater()
        
  
        main_layout = self.centralWidget().layout()
        if isinstance(main_layout, QHBoxLayout):
            main_layout.setStretch(0, 2)  # frame trái
            main_layout.setStretch(1, 8)  # frame phải (bản đồ)
            print("✅ Đã đặt layout 20:80")

    def closeEvent(self, event):
        if hasattr(self, 'http_process'):
            print("🛑 Đang tắt HTTP server...")
            self.http_process.terminate()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    try:
        sys.exit(app.exec())
    except KeyboardInterrupt:
        print("⛔ Dừng chương trình thủ công.")
