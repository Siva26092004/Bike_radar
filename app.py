import sys
from PySide6.QtWidgets import QApplication
from backend.grid_backend import GridBackend
from frontend.main_window import MainWindow
from pathlib import Path

def main():
    app = QApplication(sys.argv)

    backend = GridBackend()
    window = MainWindow(backend)

    window.resize(900, 600)
    window.show()
  

    #hi
    # After window.show()
    CONFIG_PORT = "COM19"  # Replace with your actual config port
    DATA_PORT = "COM20"    # Replace with your actual data port
    CONFIG_FILE = Path("AOP_6m_default.cfg")  # Replace with actual path

    # Send configuration
    backend.send_config(CONFIG_PORT, CONFIG_FILE)

    # Start reading radar data
    backend.start_reading(DATA_PORT)

    sys.exit(app.exec())


if __name__ == "__main__":
    main()