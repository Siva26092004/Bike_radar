from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QRadioButton, QGroupBox,
    QLabel, QDoubleSpinBox, QSizePolicy, QFileDialog
)
from PySide6.QtCore import QTimer
import pyqtgraph as pg
import pyqtgraph.exporters as pg_exporters
import numpy as np


class MainWindow(QWidget):
    def __init__(self, backend):
        super().__init__()

        self.backend = backend
        self.setWindowTitle("Bike Radar Grid Setup")

        self.grid = None                 # occupancy grid (0/1)
        self.current_index = 0           # AUTO traversal index

        self._build_ui()

        # Backend connections
        self.backend.grid_ready.connect(self.update_grid)
        self.backend.radar_points_ready.connect(self.update_radar_points)

    # -------------------------------------------------
    # UI BUILD
    # -------------------------------------------------
    def _build_ui(self):
        root = QHBoxLayout(self)
        root.setSpacing(6)

        # ================= LEFT PANEL =================
        left_panel = QVBoxLayout()
        left_panel.setSpacing(8)

        # -------- GRID PARAMETERS --------
        grid_box = QGroupBox("Grid Parameters")
        grid_layout = QVBoxLayout()

        self.xmin = self._dspin("X Min (deg)", grid_layout, -12.0)
        self.xmax = self._dspin("X Max (deg)", grid_layout, 12.0)
        self.ymin = self._dspin("Y Min (m)", grid_layout, 0.0)
        self.ymax = self._dspin("Y Max (m)", grid_layout, 120.0)
        self.dx   = self._dspin("Cell Size X (deg)", grid_layout, 1.0)
        self.dy   = self._dspin("Cell Size Y (m)", grid_layout, 5.0)

        grid_box.setLayout(grid_layout)
        left_panel.addWidget(grid_box)

        # -------- CREATE GRID --------
        self.create_btn = QPushButton("Create Grid")
        self.create_btn.setFixedHeight(30)
        self.create_btn.clicked.connect(self.on_create_grid)
        left_panel.addWidget(self.create_btn)

        # -------- EXPORT --------
        self.export_btn = QPushButton("Export Plot")
        self.export_btn.setFixedHeight(30)
        self.export_btn.clicked.connect(self.export_plot)
        left_panel.addWidget(self.export_btn)

        # -------- MODE --------
        mode_box = QGroupBox("Mode")
        mode_layout = QVBoxLayout()

        self.manual_btn = QRadioButton("Manual")
        self.auto_btn = QRadioButton("Auto")
        self.manual_btn.setChecked(True)

        timing_row = QHBoxLayout()
        timing_row.addWidget(QLabel("Auto step (sec):"))
        self.auto_interval = QDoubleSpinBox()
        self.auto_interval.setRange(0.1, 10.0)
        self.auto_interval.setValue(1.0)
        self.auto_interval.setSingleStep(0.5)
        timing_row.addWidget(self.auto_interval)

        mode_layout.addWidget(self.manual_btn)
        mode_layout.addWidget(self.auto_btn)
        mode_layout.addLayout(timing_row)
        mode_box.setLayout(mode_layout)

        left_panel.addWidget(mode_box)
        left_panel.addStretch(1)

        left_widget = QWidget()
        left_widget.setLayout(left_panel)
        left_widget.setFixedWidth(340)
        root.addWidget(left_widget)

        # ================= RIGHT PANEL =================
        self.plot = pg.PlotWidget()
        self.plot.setBackground('k')
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        self.plot.enableAutoRange(False, False)
        self.plot.setMouseEnabled(False, False)
        self.plot.getPlotItem().layout.setContentsMargins(0, 0, 0, 0)

        self.image = pg.ImageItem()
        self.plot.addItem(self.image)

        # ---- OCCUPANCY LUT ----
        self.occ_lut = np.array([
            [255,   0,   0, 255],   # 0 → red
            [  0, 255,   0, 255],   # 1 → green
        ], dtype=np.uint8)

        # ---- HIGHLIGHT (AUTO / MANUAL) ----
        self.highlight = pg.RectROI(
            [0, 0], [1, 1],
            pen=pg.mkPen('r', width=2)
        )
        self.highlight.setVisible(False)
        self.plot.addItem(self.highlight)

        self.plot.setLabel("bottom", "Angle (deg)")
        self.plot.setLabel("left", "Range (m)")
        self.plot.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        root.addWidget(self.plot, stretch=1)

        # -------- AUTO TIMER --------
        self.timer = QTimer()
        self.timer.timeout.connect(self.auto_step)

        self.auto_btn.toggled.connect(self.on_mode_change)
        self.plot.scene().sigMouseClicked.connect(self.on_plot_click)

    # -------------------------------------------------
    def _dspin(self, label, layout, default):
        row = QHBoxLayout()
        row.addWidget(QLabel(label))

        spin = QDoubleSpinBox()
        spin.setRange(-1000, 1000)
        spin.setValue(default)
        spin.setFixedWidth(110)

        row.addStretch(1)
        row.addWidget(spin)
        layout.addLayout(row)
        return spin

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
        self.backend.create_grid(cfg)

    # -------------------------------------------------
    # GRID INITIALIZATION (ON CREATE GRID)
    # -------------------------------------------------
    def update_grid(self, grid):
        self.current_index = 0

        # Occupancy grid (0/1 only)
        self.grid = np.zeros_like(grid, dtype=np.uint8)

        x_min, x_max = self.xmin.value(), self.xmax.value()
        y_min, y_max = self.ymin.value(), self.ymax.value()
        dx, dy = self.dx.value(), self.dy.value()

        # Reset image (reshape-safe)
        self.plot.removeItem(self.image)
        self.image = pg.ImageItem()
        self.plot.addItem(self.image)

        self.image.setImage(
            self.grid.T,
            autoLevels=False,
            levels=(0, 1)
        )
        self.image.setLookupTable(self.occ_lut)

        self.image.setRect(
            x_min,
            y_min,
            x_max - x_min,
            y_max - y_min
        )

        self.plot.setXRange(x_min, x_max, padding=0)
        self.plot.setYRange(y_min, y_max, padding=0)

        x_ticks = [(v, f"{v:g}") for v in np.arange(x_min, x_max + dx, dx)]
        y_ticks = [(v, f"{v:g}") for v in np.arange(y_min, y_max + dy, dy)]
        self.plot.getAxis("bottom").setTicks([x_ticks])
        self.plot.getAxis("left").setTicks([y_ticks])

        self.highlight.setVisible(False)

    # -------------------------------------------------
    # BACKEND RADAR UPDATE (OCCUPANCY ONLY)
    # -------------------------------------------------
    def update_radar_points(self, points):
        if self.grid is None:
            return

        # Clear grid (all red)
        self.grid.fill(0)

        # Mark detected bins green
        for p in points:
            ix = int(p['x'])
            iy = int(p['y'])
            if 0 <= iy < self.grid.shape[0] and 0 <= ix < self.grid.shape[1]:
                self.grid[iy, ix] = 1

        # Update image only (NO reshape, NO axis change)
        self.image.setImage(
            self.grid.T,
            autoLevels=False,
            levels=(0, 1)
        )

    # -------------------------------------------------
    def on_mode_change(self):
        if self.auto_btn.isChecked():
            self.timer.start(int(self.auto_interval.value() * 1000))
        else:
            self.timer.stop()

    # -------------------------------------------------
    # AUTO BIN TRAVERSAL (UNCHANGED)
    # -------------------------------------------------
    def auto_step(self):
        if self.grid is None:
            return

        ny, nx = self.grid.shape
        idx = self.current_index % (nx * ny)
        ix = idx % nx
        iy = idx // nx

        self.highlight_bin(ix, iy)
        self.current_index += 1

    # -------------------------------------------------
    # MANUAL CLICK
    # -------------------------------------------------
    def on_plot_click(self, event):
        if not self.manual_btn.isChecked() or self.grid is None:
            return

        pos = event.scenePos()
        vb = self.plot.getViewBox()
        pt = vb.mapSceneToView(pos)

        ix = int((pt.x() - self.xmin.value()) / self.dx.value())
        iy = int((pt.y() - self.ymin.value()) / self.dy.value())

        self.highlight_bin(ix, iy)

    # -------------------------------------------------
    def highlight_bin(self, ix, iy):
        if self.grid is None:
            return

        ny, nx = self.grid.shape
        if ix < 0 or iy < 0 or ix >= nx or iy >= ny:
            return

        x = self.xmin.value() + ix * self.dx.value()
        y = self.ymin.value() + iy * self.dy.value()

        self.highlight.setPos([x, y])
        self.highlight.setSize([self.dx.value(), self.dy.value()])
        self.highlight.setVisible(True)

    # -------------------------------------------------
    def export_plot(self):
        if self.grid is None:
            return

        file_path, _ = QFileDialog.getSaveFileName(
            self,
            "Save Plot Image",
            "bike_radar_grid.png",
            "PNG Images (*.png);;JPEG Images (*.jpg)"
        )
        if not file_path:
            return

        exporter = pg_exporters.ImageExporter(self.plot.getPlotItem())
        exporter.parameters()['width'] = 1920
        exporter.export(file_path)
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