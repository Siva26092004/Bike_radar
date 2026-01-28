from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QRadioButton, QGroupBox,
    QLabel, QDoubleSpinBox, QSizePolicy
)
import pyqtgraph as pg
import numpy as np


class MainWindow(QWidget):
    def __init__(self, backend):
        super().__init__()

        self.backend = backend
        self.setWindowTitle("Bike Radar Grid Setup")

        self._build_ui()
        self.backend.grid_ready.connect(self.update_grid)
        self.backend.radar_points_ready.connect(self.update_radar_points)

    # -------------------------------------------------
    # UI BUILD (SPLIT SCREEN)
    # -------------------------------------------------
    def _build_ui(self):
        root = QHBoxLayout(self)
        root.setSpacing(6)

        # ================= LEFT PANEL =================
        left_panel = QVBoxLayout()
        left_panel.setSpacing(8)

        # -------- MODE (INDEPENDENT) --------
        mode_box = QGroupBox("Mode")
        mode_layout = QHBoxLayout()

        self.auto_btn = QRadioButton("Auto")
        self.manual_btn = QRadioButton("Manual")
        self.manual_btn.setChecked(True)

        mode_layout.addWidget(self.auto_btn)
        mode_layout.addWidget(self.manual_btn)
        mode_layout.addStretch(1)

        mode_box.setLayout(mode_layout)
        left_panel.addWidget(mode_box)

        # -------- GRID PARAMETERS --------
        self.param_box = QGroupBox("Grid Parameters")
        param_layout = QVBoxLayout()
        param_layout.setSpacing(6)

        self.xmin = self._spin("X Min(m)", param_layout, -12.0)
        self.xmax = self._spin("X Max(m)", param_layout, 12.0)
        self.ymin = self._spin("Y Min (m)", param_layout, 0.0)
        self.ymax = self._spin("Y Max (m)", param_layout, 120.0)
        self.dx   = self._spin("Cell Size X (m)", param_layout, 1.0)
        self.dy   = self._spin("Cell Size Y (m)", param_layout, 5.0)

        self.param_box.setLayout(param_layout)
        left_panel.addWidget(self.param_box)

        # -------- CREATE GRID --------
        self.create_btn = QPushButton("Create Grid")
        self.create_btn.setFixedHeight(30)
        self.create_btn.clicked.connect(self.on_create_grid)
        left_panel.addWidget(self.create_btn)

        left_panel.addStretch(1)

        left_widget = QWidget()
        left_widget.setLayout(left_panel)
        left_widget.setFixedWidth(320)

        root.addWidget(left_widget)

        # ================= RIGHT PANEL (PLOT) =================
        self.plot = pg.PlotWidget()
        self.plot.setBackground('k')
        self.plot.setAspectLocked(False)
        self.plot.showGrid(x=True, y=True, alpha=0.25)

        # Axis behavior controlled ONLY by Create Grid
        self.plot.enableAutoRange(False, False)
        self.plot.setMouseEnabled(False, False)

        self.plot.getPlotItem().layout.setContentsMargins(0, 0, 0, 0)

        self.image = pg.ImageItem()
        self.plot.addItem(self.image)

        # Scatter plot for radar points
        self.scatter = pg.ScatterPlotItem(
            size=10,
            pen=pg.mkPen(None),
            brush=pg.mkBrush(255, 0, 0, 200)  # Red with alpha
        )
        self.plot.addItem(self.scatter)

        self.plot.setLabel("bottom", "X Position (m)")
        self.plot.setLabel("left", "Y Position (m)")

        self.plot.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        root.addWidget(self.plot, stretch=1)

    # -------------------------------------------------
    # SPIN BOX
    # -------------------------------------------------
    def _spin(self, label, layout, default):
        row = QHBoxLayout()
        row.addWidget(QLabel(label))

        spin = QDoubleSpinBox()
        spin.setDecimals(2)
        spin.setRange(-1000, 1000)
        spin.setValue(default)
        spin.setFixedWidth(110)

        row.addStretch(1)
        row.addWidget(spin)
        layout.addLayout(row)

        return spin

    # -------------------------------------------------
    # CREATE GRID (ALWAYS RESCALE)
    # -------------------------------------------------
    def on_create_grid(self):
        cfg = {
            "x_min": self.xmin.value(),
            "x_max": self.xmax.value(),
            "y_min": self.ymin.value(),
            "y_max": self.ymax.value(),
            "dx": self.dx.value(),
            "dy": self.dy.value(),
        }

        # Create grid ALWAYS resets plot
        self.backend.create_grid(cfg)

    # -------------------------------------------------
    # UPDATE GRID + AXES (UNCONDITIONAL)
    # -------------------------------------------------
    def update_grid(self, grid):
        if grid is None or grid.size == 0:
            return

        x_min = self.xmin.value()
        x_max = self.xmax.value()
        y_min = self.ymin.value()
        y_max = self.ymax.value()
        dx = self.dx.value()
        dy = self.dy.value()

        # Update image
        self.image.setImage(grid.T, autoLevels=True)
        self.image.setRect(
            x_min,
            y_min,
            x_max - x_min,
            y_max - y_min
        )

        # ALWAYS rescale on Create Grid
        self.plot.setXRange(x_min, x_max, padding=0)
        self.plot.setYRange(y_min, y_max, padding=0)

        # Explicit ticks
        x_ticks = [(v, f"{v:g}") for v in np.arange(x_min, x_max + dx, dx)]
        y_ticks = [(v, f"{v:g}") for v in np.arange(y_min, y_max + dy, dy)]

        self.plot.getAxis("bottom").setTicks([x_ticks])
        self.plot.getAxis("left").setTicks([y_ticks])
    # -------------------------------------------------
    # UPDATE RADAR POINTS
    # -------------------------------------------------
    def update_radar_points(self, points):
        """Update scatter plot with new radar points."""
        print(f"points {points}")
        if not points:
            self.scatter.setData([], [])
            return

        # Extract real-world coordinates for plotting
        x_coords = [p['y'] for p in points]
        y_coords = [p['x'] for p in points]
        
        # Update scatter plot
        self.scatter.setData(x_coords, y_coords)
        
        print(f"Displaying {len(points)} radar points on grid")