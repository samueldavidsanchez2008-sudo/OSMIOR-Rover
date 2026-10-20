import datetime
import math
import queue
import socket
import sys
import time

import pyqtgraph as pg
from PyQt6.QtCore import QPointF, Qt, QThread, QTimer, pyqtSignal, pyqtSlot
from PyQt6.QtGui import QBrush, QFont, QPainter, QPen, QColor, QPolygonF
from PyQt6.QtWidgets import (
    QApplication,
    QComboBox,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QProgressBar,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

NICLA_IP = "192.168.4.1"
TCP_PORT = 8080
MAX_SAMPLES = 180
HEARTBEAT_MS = 500
RADAR_MIRROR = False       # True si el radar sale invertido izquierda/derecha
RADAR_MAX_CM = 200
RADAR_FADE_S = 8.0

# Bash-like palette.
BG = "#070b0d"
CARD = "#0c1215"
GRID = "#193037"
TEXT = "#d8fbe1"
GREEN = "#39ff88"
CYAN = "#43e5ff"
YELLOW = "#ffd166"
ORANGE = "#ff9f43"
RED = "#ff4d6d"
MAGENTA = "#f58cff"
BLUE = "#7aa2ff"
GRAY = "#66837a"


# ============================================================
# NETWORK WORKER
# ============================================================

class NetworkWorker(QThread):
    telemetry = pyqtSignal(dict)
    status = pyqtSignal(str)
    event = pyqtSignal(str)
    stats = pyqtSignal(dict)
    radar = pyqtSignal(list)

    def __init__(self):
        super().__init__()
        self.running = True
        self.tx_queue = queue.Queue()
        self.sock = None
        self.sequence = 0
        self.pending_times = {}

    def stop(self):
        self.running = False
        try:
            if self.sock:
                self.sock.sendall(b"CMD,S\n")
        except Exception:
            pass
        try:
            self.tx_queue.put_nowait("STOP_LOCAL")
        except Exception:
            pass
        try:
            if self.sock:
                self.sock.shutdown(socket.SHUT_RDWR)
        except Exception:
            pass
        try:
            if self.sock:
                self.sock.close()
        except Exception:
            pass

    def send_line(self, line):
        if not isinstance(line, str):
            return
        try:
            self.tx_queue.put_nowait(line)
        except queue.Full:
            pass

    @staticmethod
    def parse_telemetry(line):
        if not line.startswith("TEL;"):
            return None

        data = {}
        for token in line[4:].split(";"):
            if "=" not in token:
                continue
            key, value = token.split("=", 1)
            try:
                if key in {"PERSON", "MODEL", "DF", "PWM", "RADAR"}:
                    data[key] = int(float(value))
                elif key in {"STATE", "COL", "PRAW", "MODE"}:
                    data[key] = value
                else:
                    data[key] = float(value)
            except ValueError:
                data[key] = value

        return data

    @staticmethod
    def parse_radar(line):
        # RAD;PTS=90:123,96:-1,...
        if not line.startswith("RAD;PTS="):
            return None
        pts = []
        for tok in line[8:].split(","):
            if ":" not in tok:
                continue
            a, d = tok.split(":", 1)
            try:
                pts.append((int(a), int(d)))
            except ValueError:
                pass
        return pts

    @staticmethod
    def parse_stats(line):
        if not line.startswith("STAT;"):
            return None
        result = {}
        for token in line[5:].split(";"):
            if "=" not in token:
                continue
            key, value = token.split("=", 1)
            try:
                result[key] = float(value)
            except ValueError:
                result[key] = value
        return result

    def transmit_queued(self, sock):
        while True:
            try:
                item = self.tx_queue.get_nowait()
            except queue.Empty:
                return

            if item == "STOP_LOCAL":
                continue

            wire_line = item.strip()
            seq = None
            upper = wire_line.upper()

            # Number commands at the HMI so each UNO ACK can be timed.
            if upper.startswith(("CMD,", "AUD,", "COL,", "RAD,")):
                fields = wire_line.split(",")
                if len(fields) == 2:
                    self.sequence = (self.sequence + 1) % 1000000
                    if self.sequence == 0:
                        self.sequence = 1
                    seq = self.sequence
                    wire_line = wire_line + "," + str(seq)
                elif len(fields) >= 3:
                    try:
                        seq = int(fields[2])
                    except ValueError:
                        seq = None

            sock.sendall((wire_line + "\n").encode("ascii"))

            if seq is not None:
                self.pending_times[seq] = (time.monotonic(), item)
                # Bound the map in case a command never receives an ACK.
                if len(self.pending_times) > 32:
                    oldest = next(iter(self.pending_times))
                    self.pending_times.pop(oldest, None)

    def run(self):
        while self.running:
            sock = None
            rx_buffer = ""
            last_hb = 0.0

            try:
                self.status.emit("CONECTANDO")
                sock = socket.create_connection((NICLA_IP, TCP_PORT), timeout=2.0)
                sock.settimeout(0.05)
                self.sock = sock

                sock.sendall(b"HELLO\n")
                sock.sendall(b"CMD,S\n")
                self.status.emit("CONECTADO")
                last_hb = time.monotonic()

                while self.running:
                    # Heartbeat keeps the rover watchdog alive.
                    if (time.monotonic() - last_hb) * 1000.0 >= HEARTBEAT_MS:
                        sock.sendall(b"HB\n")
                        last_hb = time.monotonic()

                    self.transmit_queued(sock)

                    try:
                        chunk = sock.recv(4096)
                        if not chunk:
                            raise ConnectionError

                        rx_buffer += chunk.decode("ascii", "ignore")
                        if len(rx_buffer) > 16000:
                            rx_buffer = ""

                        while "\n" in rx_buffer:
                            line, rx_buffer = rx_buffer.split("\n", 1)
                            line = line.strip()

                            parsed = self.parse_telemetry(line)
                            if parsed is not None:
                                self.telemetry.emit(parsed)
                                continue

                            pts = self.parse_radar(line)
                            if pts is not None:
                                self.radar.emit(pts)
                                continue

                            stats = self.parse_stats(line)
                            if stats is not None:
                                self.stats.emit(stats)
                                continue

                            if line.startswith("EVT;"):
                                if "CMD_SUPERSEDED;" in line:
                                    try:
                                        seq_text = [p.split("=", 1)[1] for p in line.split(";") if p.startswith("SEQ=")][0]
                                        self.pending_times.pop(int(seq_text), None)
                                    except Exception:
                                        pass
                                if "ACK;" in line:
                                    try:
                                        seq_text = [p.split("=", 1)[1] for p in line.split(";") if p.startswith("SEQ=")][0]
                                        seq = int(seq_text)
                                        pending = self.pending_times.pop(seq, None)
                                        if pending is not None:
                                            rtt_ms = (time.monotonic() - pending[0]) * 1000.0
                                            line += ";RTT_MS={:.1f}".format(rtt_ms)
                                    except Exception:
                                        pass
                                elif "ACK_TIMEOUT;" in line:
                                    try:
                                        seq_text = [p.split("=", 1)[1] for p in line.split(";") if p.startswith("SEQ=")][0]
                                        self.pending_times.pop(int(seq_text), None)
                                    except Exception:
                                        pass
                                self.event.emit(line[4:])
                            elif line.startswith("READY;"):
                                self.event.emit(line)
                            elif line == "PONG":
                                self.event.emit("PONG")

                    except socket.timeout:
                        pass

            except Exception:
                self.status.emit("DESCONECTADO")
                try:
                    if sock:
                        sock.close()
                except Exception:
                    pass

                self.sock = None

                # Nunca arrastamos comandos viejos de movimiento a un reconnect.
                try:
                    while True:
                        self.tx_queue.get_nowait()
                except queue.Empty:
                    pass

                time.sleep(1.0)


# ============================================================
# GAUGE
# ============================================================

class AngleGauge(QWidget):
    def __init__(self, title):
        super().__init__()
        self.title = title
        self.value = 0.0
        self.critical = 35.0
        self.red = 30.0
        self.minimum = -45.0
        self.maximum = 45.0
        self.setMinimumSize(190, 215)

    def setValue(self, value, critical=35.0, red=30.0):
        self.value = max(self.minimum, min(self.maximum, float(value)))
        self.critical = float(critical)
        self.red = float(red)
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        w = self.width()
        h = self.height()
        cx = w / 2.0
        cy = h * 0.72
        r = min(w * 0.38, h * 0.46)

        p.setPen(QPen(QColor(GRAY), 10))
        p.drawArc(int(cx-r), int(cy-r), int(2*r), int(2*r), 210*16, -240*16)

        # Central zero mark.
        p.setPen(QPen(QColor(TEXT), 2))
        for deg in range(-45, 46, 15):
            a = math.radians(-90 + deg * 240 / 90)
            x1 = cx + math.cos(a) * (r - 2)
            y1 = cy + math.sin(a) * (r - 2)
            x2 = cx + math.cos(a) * (r - 18)
            y2 = cy + math.sin(a) * (r - 18)
            p.drawLine(int(x1), int(y1), int(x2), int(y2))

        norm = (self.value-self.minimum)/(self.maximum-self.minimum)
        needle_angle = math.radians(-120 + norm*240 - 90)
        nx = cx + math.cos(needle_angle) * (r - 26)
        ny = cy + math.sin(needle_angle) * (r - 26)

        needle_color = GREEN
        if abs(self.value) >= self.red:
            needle_color = RED
        elif abs(self.value) >= self.critical - 10:
            needle_color = YELLOW

        p.setPen(QPen(QColor(needle_color), 5))
        p.drawLine(int(cx), int(cy), int(nx), int(ny))

        p.setBrush(QColor(TEXT))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawEllipse(int(cx-6), int(cy-6), 12, 12)

        p.setPen(QColor(TEXT))
        p.setFont(QFont("Consolas", 11, QFont.Weight.Bold))
        p.drawText(0, 12, w, 24, Qt.AlignmentFlag.AlignCenter, self.title)

        p.setFont(QFont("Consolas", 18, QFont.Weight.Bold))
        p.drawText(0, int(cy+34), w, 28, Qt.AlignmentFlag.AlignCenter, f"{self.value:+.1f} deg")


# ============================================================
# RADAR
# ============================================================

class RadarWidget(QWidget):
    """Semicirculo de 180 grados. 90 deg del servo = al frente (arriba)."""

    def __init__(self):
        super().__init__()
        self.setMinimumSize(300, 190)
        self.active = False
        self.bins = {}          # angulo -> (cm, t)
        self.last_angle = None

    def set_active(self, on):
        self.active = bool(on)
        self.update()

    def clear(self):
        self.bins.clear()
        self.last_angle = None
        self.update()

    def add_points(self, pts):
        now = time.monotonic()
        for ang, cm in pts:
            self.bins[ang] = (cm, now)
            self.last_angle = ang
        self.update()

    def _xy(self, cx, cy, radius, ang):
        a = ang if RADAR_MIRROR else 180 - ang
        phi = math.radians(a)
        return cx + math.cos(phi) * radius, cy - math.sin(phi) * radius

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        cx = w / 2.0
        cy = h - 16.0
        R = max(40.0, min(w / 2.0 - 14.0, h - 34.0))

        # Rings + spokes.
        p.setPen(QPen(QColor(GRID), 1))
        for frac in (0.25, 0.5, 0.75, 1.0):
            r = R * frac
            p.drawArc(int(cx - r), int(cy - r), int(2 * r), int(2 * r), 0, 180 * 16)
        for ang in range(0, 181, 30):
            x, y = self._xy(cx, cy, R, ang)
            p.drawLine(int(cx), int(cy), int(x), int(y))
        p.setPen(QColor(GRAY))
        p.setFont(QFont("Consolas", 8))
        for frac in (0.5, 1.0):
            p.drawText(int(cx + 4), int(cy - R * frac - 2), f"{int(RADAR_MAX_CM * frac)} cm")

        now = time.monotonic()
        live = sorted((a, v) for a, v in self.bins.items() if now - v[1] < RADAR_FADE_S)
        profile = []
        for ang, (cm, t) in live:
            r = R if cm < 0 else min(cm, RADAR_MAX_CM) / float(RADAR_MAX_CM) * R
            x, y = self._xy(cx, cy, r, ang)
            profile.append((ang, cm, t, x, y))

        # Free-space silhouette.
        if len(profile) >= 2:
            poly = QPolygonF([QPointF(cx, cy)] + [QPointF(x, y) for _a, _c, _t, x, y in profile])
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(QBrush(QColor(57, 255, 136, 28)))
            p.drawPolygon(poly)

        # Contour (only between neighbouring bins) + echo dots.
        for i in range(len(profile)):
            ang, cm, t, x, y = profile[i]
            fade = max(0.15, 1.0 - (now - t) / RADAR_FADE_S)
            col = QColor(GREEN)
            col.setAlphaF(fade)
            if i > 0 and ang - profile[i - 1][0] <= 9:
                _a0, c0, _t0, x0, y0 = profile[i - 1]
                p.setPen(QPen(col, 2))
                p.drawLine(int(x0), int(y0), int(x), int(y))
            if cm >= 0:
                p.setPen(Qt.PenStyle.NoPen)
                p.setBrush(QBrush(col))
                p.drawEllipse(QPointF(x, y), 3.0, 3.0)

        # Sweep line.
        if self.active and self.last_angle is not None:
            x, y = self._xy(cx, cy, R, self.last_angle)
            p.setPen(QPen(QColor(CYAN), 2))
            p.drawLine(int(cx), int(cy), int(x), int(y))

        p.setPen(QColor(GRAY))
        p.setFont(QFont("Consolas", 10, QFont.Weight.Bold))
        if not self.active:
            p.drawText(0, int(h * 0.35), w, 24, Qt.AlignmentFlag.AlignCenter,
                       "RADAR OFF" if not live else "RADAR OFF (ultimo barrido)")


# ============================================================
# PLOT FACTORY
# ============================================================

def make_plot(title, y_label, pens):
    plot = pg.PlotWidget()
    plot.setBackground(BG)
    plot.showGrid(x=True, y=True, alpha=0.20)
    plot.setTitle(title, color=TEXT, size="10pt")
    plot.getAxis("left").setLabel(y_label, color=GRAY)
    plot.getAxis("bottom").setLabel("t (samples)", color=GRAY)
    plot.getAxis("left").setTextPen(QPen(QColor(GRAY)))
    plot.getAxis("bottom").setTextPen(QPen(QColor(GRAY)))
    plot.setMenuEnabled(False)
    plot.setMouseEnabled(False, False)
    plot.addLegend(offset=(5, 5))

    curves = []
    for name, color in pens:
        curve = plot.plot(pen=pg.mkPen(color=color, width=2), name=name)
        curves.append(curve)

    return plot, curves


class Card(QFrame):
    def __init__(self, title=""):
        super().__init__()
        self.setObjectName("card")
        self.layout = QVBoxLayout(self)
        if title:
            label = QLabel(title)
            label.setObjectName("section")
            self.layout.addWidget(label)


# ============================================================
# MAIN HMI
# ============================================================

class HMI(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("OSMIOR // HMI")
        self.resize(1650, 1000)
        self.setMinimumSize(1400, 850)

        self.worker = NetworkWorker()
        self.worker.telemetry.connect(self.update_telemetry)
        self.worker.status.connect(self.update_connection)
        self.worker.event.connect(self.update_event)
        self.worker.stats.connect(self.update_stats)
        self.worker.radar.connect(self.update_radar)

        self.hist = {key: [] for key in (
            "VTHEORY", "VIMU",
            "TAUREQ", "TAUTOTAL",
            "PITCH", "ROLL", "CRIT", "TILT",
            "NFRONT", "NREAR", "NLEFT", "NRIGHT",
            "DL", "DR",
            "FREQ",
            "AX", "AY", "AZ",
            "GX", "GY", "GZ",
            "TEXT", "TCAB", "HUM",
        )}

        # Mission clock: starts when the HMI first connects to the AP and
        # stops only when the operator presses FIN MISION.
        self.mission_start = None
        self.mission_end = None
        self.mission_ended = False
        self.mission_start_wall = None
        self.connected = False
        self.radar_wanted = False
        self.radar_actual = False

        self.last_data = {}
        self.last_graph_update = 0
        self.build_ui()
        self.apply_style()
        self.worker.start()

        self.mission_timer = QTimer(self)
        self.mission_timer.timeout.connect(self.refresh_mission)
        self.mission_timer.start(250)

    # --------------------------------------------------------
    # UI
    # --------------------------------------------------------

    def build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        outer = QVBoxLayout(root)
        outer.setContentsMargins(10, 8, 10, 8)
        outer.setSpacing(8)

        # Header.
        header = QHBoxLayout()

        self.title = QLabel("OSMIOR // ROVER CONTROL TERMINAL V4")
        self.title.setObjectName("title")

        self.conn_label = QLabel("DESCONECTADO")
        self.conn_label.setObjectName("conn")

        self.mode_label = QLabel("MODE: MANUAL")
        self.mode_label.setObjectName("mode")

        self.model_label = QLabel("ML: --")
        self.model_label.setObjectName("model")

        header.addWidget(self.title)
        header.addStretch()
        header.addWidget(self.model_label)
        header.addSpacing(16)
        header.addWidget(self.mode_label)
        header.addSpacing(16)
        header.addWidget(self.conn_label)

        outer.addLayout(header)

        # Main: stability | graphs | control.
        main = QGridLayout()
        main.setHorizontalSpacing(8)
        main.setVerticalSpacing(8)
        outer.addLayout(main, stretch=1)

        self.build_stability(main)
        self.build_graphs(main)
        self.build_control(main)

        # Bottom status / battery.
        bottom = Card("AUTONOMIA ESTIMADA / EVENTOS")
        bottom_row = QHBoxLayout()

        self.battery_bar = QProgressBar()
        self.battery_bar.setRange(0, 100)
        self.battery_bar.setValue(100)
        self.battery_bar.setTextVisible(True)
        self.battery_bar.setFormat("%p%")

        self.battery_text = QLabel("100% / -- min")
        self.last_event_label = QLabel("EVENT: waiting")
        self.last_event_label.setObjectName("event")

        bottom_row.addWidget(self.battery_bar, stretch=2)
        bottom_row.addWidget(self.battery_text)
        bottom_row.addSpacing(20)
        bottom_row.addWidget(self.last_event_label, stretch=2)
        bottom.layout.addLayout(bottom_row)
        outer.addWidget(bottom)

    def build_stability(self, main):
        card = Card("STABILITY / INCLINATION")
        gauges = QHBoxLayout()

        self.pitch_gauge = AngleGauge("PITCH")
        self.roll_gauge = AngleGauge("ROLL")
        gauges.addWidget(self.pitch_gauge)
        gauges.addWidget(self.roll_gauge)
        card.layout.addLayout(gauges)

        self.tilt_status = QLabel("STATE: GREEN")
        self.tilt_status.setObjectName("greenStatus")
        card.layout.addWidget(self.tilt_status)

        grid = QGridLayout()
        self.critical_label = QLabel("CRIT: -- deg")
        self.margin_label = QLabel("MARGIN: -- deg")
        self.nfront = QLabel("N FRONT: -- N")
        self.nrear = QLabel("N REAR: -- N")
        self.nleft = QLabel("N LEFT: -- N")
        self.nright = QLabel("N RIGHT: -- N")

        grid.addWidget(self.critical_label, 0, 0)
        grid.addWidget(self.margin_label, 0, 1)
        grid.addWidget(self.nfront, 1, 0)
        grid.addWidget(self.nrear, 1, 1)
        grid.addWidget(self.nleft, 2, 0)
        grid.addWidget(self.nright, 2, 1)

        card.layout.addLayout(grid)

        note = QLabel("5 deg before critical -> RED / AUDIO 004")
        note.setObjectName("note")
        card.layout.addWidget(note)

        # ---- MISSION TIME ----
        mission = Card("MISSION TIME")
        self.mission_clock = QLabel("T+ --:--:--")
        self.mission_clock.setObjectName("clock")
        self.mission_info = QLabel("Esperando conexion al AP...")
        self.mission_info.setObjectName("note")
        self.mission_btn = QPushButton("FIN MISION")
        self.mission_btn.clicked.connect(self.toggle_mission)
        mission.layout.addWidget(self.mission_clock)
        mission.layout.addWidget(self.mission_info)
        mission.layout.addWidget(self.mission_btn)
        card.layout.addWidget(mission)

        # ---- ENVIRONMENT ----
        env = Card("ENVIRONMENT")
        self.ext_label = QLabel("EXT   T: -- C | HR: -- %")
        self.cab_label = QLabel("CABINA T: -- C | P: -- hPa")
        self.env_status = QLabel("SENSORES: --")
        self.co2_label = QLabel("CO2: SIN SENSOR")
        self.co2_note = QLabel("T/P/HR no permiten calcular CO2")
        self.co2_note.setObjectName("note")
        for label in (self.ext_label, self.cab_label, self.env_status, self.co2_label):
            label.setObjectName("telemetry")
            env.layout.addWidget(label)
        env.layout.addWidget(self.co2_note)
        card.layout.addWidget(env)
        card.layout.addStretch()

        main.addWidget(card, 0, 0, 4, 1)

    def build_graphs(self, main):
        container = QWidget()
        grid = QGridLayout(container)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(8)

        self.plots = {}

        definitions = [
            ("speed", "VELOCITY", "m/s", [("v theoretical", GREEN), ("v IMU est", CYAN)]),
            ("torque", "TORQUE ESTIMATED / REQUIRED", "N*m", [("tau / motor", ORANGE), ("tau total", MAGENTA)]),
            ("tilt", "ANGLE / CRITICAL", "deg", [("pitch", CYAN), ("roll", MAGENTA), ("critical", RED)]),
            ("normal", "NORMAL LOAD", "N", [("front", GREEN), ("rear", YELLOW), ("left", CYAN), ("right", ORANGE)]),
            ("distance", "HC-SR04", "cm", [("left", CYAN), ("right", YELLOW)]),
            ("traction", "TRACTION DEMAND", "N", [("F required", ORANGE)]),
            ("accel", "ACCELEROMETER", "g", [("AX", GREEN), ("AY", CYAN), ("AZ", YELLOW)]),
            ("gyro", "GYROSCOPE", "deg/s", [("GX", GREEN), ("GY", CYAN), ("GZ", YELLOW)]),
        ]

        for index, (key, title, unit, pens) in enumerate(definitions):
            plot, curves = make_plot(title, unit, pens)
            plot.setMinimumHeight(125)
            self.plots[key] = (plot, curves)
            row = index // 2
            col = index % 2
            grid.addWidget(plot, row, col)

        plot, curves = make_plot(
            "ENVIRONMENT", "C / %HR",
            [("T ext (DHT11)", ORANGE), ("T cabina (BMP280)", CYAN), ("HR ext", BLUE)],
        )
        plot.setMinimumHeight(125)
        self.plots["env"] = (plot, curves)
        grid.addWidget(plot, 4, 0, 1, 2)

        main.addWidget(container, 0, 1, 4, 1)

    def build_control(self, main):
        card = Card("CONTROL / TELEMETRY")

        pad = QGridLayout()
        self.b_forward = QPushButton("W\nFORWARD")
        self.b_left = QPushButton("A\nLEFT")
        self.b_stop = QPushButton("S\nSTOP")
        self.b_right = QPushButton("D\nRIGHT")
        self.b_back = QPushButton("X\nBACK")

        pad.addWidget(self.b_forward, 0, 1)
        pad.addWidget(self.b_left, 1, 0)
        pad.addWidget(self.b_stop, 1, 1)
        pad.addWidget(self.b_right, 1, 2)
        pad.addWidget(self.b_back, 2, 1)
        card.layout.addLayout(pad)

        self.bind_motion(self.b_forward, "F")
        self.bind_motion(self.b_back, "B")
        self.bind_motion(self.b_left, "L")
        self.bind_motion(self.b_right, "R")
        self.bind_motion(self.b_stop, "S")

        self.person_button = QPushButton("ASK PERSON")
        self.person_button.clicked.connect(lambda: self.worker.send_line("AUD,3"))
        card.layout.addWidget(self.person_button)

        audio_row = QHBoxLayout()
        self.audio_combo = QComboBox()
        self.audio_combo.addItems([
            "1 - START",
            "2 - LOW BATTERY",
            "3 - PERSON QUERY",
            "4 - CRITICAL TILT",
            "5 - PERSON DETECTED",
            "6 - INSPECCION ZONA",
        ])
        audio_button = QPushButton("PLAY")
        audio_button.clicked.connect(self.play_selected_audio)
        audio_row.addWidget(self.audio_combo)
        audio_row.addWidget(audio_button)
        card.layout.addLayout(audio_row)

        # ---- recoleccion ----
        col_row = QHBoxLayout()
        self.col_on = QPushButton("RECOLECTAR")
        self.col_rev = QPushButton("REVERSA")
        self.col_off = QPushButton("PARAR")
        self.col_on.clicked.connect(lambda: self.worker.send_line("COL,C"))
        self.col_rev.clicked.connect(lambda: self.worker.send_line("COL,E"))
        self.col_off.clicked.connect(lambda: self.worker.send_line("COL,O"))
        for b in (self.col_on, self.col_rev, self.col_off):
            col_row.addWidget(b)
        card.layout.addLayout(col_row)
        self.col_label = QLabel("COLLECTOR: --")
        self.col_label.setObjectName("telemetry")
        card.layout.addWidget(self.col_label)

        # ---- radar ----
        self.radar_button = QPushButton("RADAR: OFF  (click para activar)")
        self.radar_button.clicked.connect(self.toggle_radar)
        card.layout.addWidget(self.radar_button)
        self.radar_widget = RadarWidget()
        card.layout.addWidget(self.radar_widget)

        self.vision_label = QLabel("VISION: --")
        self.person_label = QLabel("PERSON: --")
        self.distance_label = QLabel("HC-SR04 L: -- cm | R: -- cm")
        self.motion_label = QLabel("UNO CMD: S | ACK RTT: -- ms")
        self.uart_diag_label = QLabel("UART: RX bad=0 | UNO overflow=0")
        self.imu_label = QLabel("PITCH: -- | ROLL: --")
        self.torque_label = QLabel("TAU/MOTOR: -- N*m | F: -- N")
        self.speed_label = QLabel("V THEORY: -- | V IMU: -- m/s")
        self.df_label = QLabel("DFPLAYER: --")
        self.memory_label = QLabel("MEM: --")

        for label in (
            self.person_label,
            self.vision_label,
            self.distance_label,
            self.motion_label,
            self.uart_diag_label,
            self.imu_label,
            self.torque_label,
            self.speed_label,
            self.df_label,
            self.memory_label,
        ):
            label.setObjectName("telemetry")
            card.layout.addWidget(label)

        card.layout.addStretch()
        main.addWidget(card, 0, 2, 4, 1)

    # --------------------------------------------------------
    # STYLE
    # --------------------------------------------------------

    def apply_style(self):
        self.setStyleSheet(f"""
            QMainWindow, QWidget {{
                background: {BG};
                color: {TEXT};
                font-family: Consolas, 'Courier New';
            }}
            #title {{
                color: {GREEN};
                font-size: 22px;
                font-weight: 700;
            }}
            #conn {{
                color: {RED};
                font-weight: 700;
            }}
            #mode {{
                color: {CYAN};
                font-weight: 700;
            }}
            #model {{
                color: {MAGENTA};
                font-weight: 700;
            }}
            #card {{
                background: {CARD};
                border: 1px solid {GRID};
                border-radius: 6px;
            }}
            #section {{
                color: {GREEN};
                font-size: 13px;
                font-weight: 700;
                padding-bottom: 2px;
            }}
            #telemetry {{
                color: {TEXT};
                padding: 3px 0px;
            }}
            #clock {{
                color: {GREEN};
                font-size: 30px;
                font-weight: 700;
            }}
            #note {{
                color: {GRAY};
                font-size: 11px;
            }}
            #event {{
                color: {YELLOW};
                font-weight: 700;
            }}
            QPushButton {{
                background: #0a1618;
                border: 1px solid #24515a;
                color: {TEXT};
                border-radius: 4px;
                padding: 9px;
                font-family: Consolas;
                font-weight: 700;
            }}
            QPushButton:hover {{
                background: #10272b;
                border-color: {CYAN};
            }}
            QPushButton:pressed {{
                background: #143a38;
                border-color: {GREEN};
            }}
            QComboBox {{
                background: #081214;
                color: {TEXT};
                border: 1px solid #24515a;
                padding: 7px;
            }}
            QProgressBar {{
                border: 1px solid #24515a;
                background: #081214;
                text-align: center;
                color: {TEXT};
                border-radius: 4px;
            }}
            QProgressBar::chunk {{
                background: {GREEN};
                border-radius: 3px;
            }}
            #greenStatus {{ color: {GREEN}; font-weight: 700; }}
        """)

    # --------------------------------------------------------
    # COMMANDS
    # --------------------------------------------------------

    def bind_motion(self, button, command):
        button.pressed.connect(lambda cmd=command: self.worker.send_line(f"CMD,{cmd}"))
        button.released.connect(lambda: self.worker.send_line("CMD,S"))

    def play_selected_audio(self):
        track = self.audio_combo.currentIndex() + 1
        self.worker.send_line(f"AUD,{track}")

    # --------------------------------------------------------
    # TELEMETRY
    # --------------------------------------------------------

    # --------------------------------------------------------
    # MISSION / RADAR
    # --------------------------------------------------------

    @staticmethod
    def fmt_clock(seconds):
        seconds = int(max(0, seconds))
        return "T+ {:02d}:{:02d}:{:02d}".format(seconds // 3600, (seconds // 60) % 60, seconds % 60)

    def mission_on_connect(self):
        if self.mission_start is None and not self.mission_ended:
            self.mission_start = time.monotonic()
            self.mission_start_wall = datetime.datetime.now()
            self.mission_btn.setText("FIN MISION")
        self.refresh_mission()

    def toggle_mission(self):
        if self.mission_start is not None and not self.mission_ended:
            self.mission_end = time.monotonic()
            self.mission_ended = True
            self.mission_btn.setText("NUEVA MISION")
            total = self.mission_end - self.mission_start
            self.last_event_label.setText("EVENT: MISION TERMINADA " + self.fmt_clock(total))
        else:
            self.mission_start = None
            self.mission_end = None
            self.mission_ended = False
            self.mission_start_wall = None
            self.mission_btn.setText("FIN MISION")
            if self.connected:
                self.mission_on_connect()
        self.refresh_mission()

    def refresh_mission(self):
        if self.mission_start is None:
            self.mission_clock.setText("T+ --:--:--")
            self.mission_info.setText("Esperando conexion al AP...")
            self.mission_clock.setStyleSheet(f"color:{GRAY};")
            return
        end = self.mission_end if self.mission_ended else time.monotonic()
        self.mission_clock.setText(self.fmt_clock(end - self.mission_start))
        start_txt = self.mission_start_wall.strftime("%H:%M:%S")
        if self.mission_ended:
            self.mission_info.setText(f"Inicio {start_txt} | TERMINADA")
            self.mission_clock.setStyleSheet(f"color:{YELLOW};")
        else:
            link = "" if self.connected else " | SIN ENLACE (el tiempo sigue)"
            self.mission_info.setText(f"Inicio {start_txt}{link}")
            self.mission_clock.setStyleSheet(f"color:{GREEN};")

    def toggle_radar(self):
        self.radar_wanted = not (self.radar_wanted or self.radar_actual)
        if self.radar_wanted:
            self.radar_widget.clear()
        self.worker.send_line("RAD,1" if self.radar_wanted else "RAD,0")
        self.refresh_radar_button()

    def refresh_radar_button(self):
        if self.radar_actual:
            self.radar_button.setText("RADAR: ON  (click para apagar)")
        elif self.radar_wanted:
            self.radar_button.setText("RADAR: activando...")
        else:
            self.radar_button.setText("RADAR: OFF  (click para activar)")

    @pyqtSlot(list)
    def update_radar(self, pts):
        self.radar_widget.add_points(pts)

    @pyqtSlot(str)
    def update_connection(self, state):
        self.connected = (state == "CONECTADO")
        if self.connected:
            self.mission_on_connect()
        else:
            self.radar_wanted = False
            self.radar_actual = False
            self.refresh_radar_button()
            self.radar_widget.set_active(False)
        self.conn_label.setText(state)
        if state == "CONECTADO":
            self.conn_label.setStyleSheet(f"color:{GREEN};font-weight:700;")
        elif state == "CONECTANDO":
            self.conn_label.setStyleSheet(f"color:{YELLOW};font-weight:700;")
        else:
            self.conn_label.setStyleSheet(f"color:{RED};font-weight:700;")

    @pyqtSlot(dict)
    def update_stats(self, stats):
        model_ok = bool(int(stats.get("MODEL", 0)))
        mram = int(stats.get("MRAM", 0))
        mlen = int(stats.get("MLEN", 0))
        free = int(stats.get("FREE", 0))

        self.memory_label.setText(
            f"MEM FREE: {free//1024} KB | MODEL: {mlen//1024} KB / RAM {mram//1024} KB"
        )
        self.model_label.setText("ML: GOOGLE PERSON OK" if model_ok else "ML: NOT LOADED")

        self.model_label.setStyleSheet(
            f"color:{GREEN};font-weight:700;"
            if model_ok
            else f"color:{RED};font-weight:700;"
        )

    @pyqtSlot(str)
    def update_event(self, event):
        self.last_event_label.setText("EVENT: " + event)

        parts = event.split(";")
        fields = {}
        for part in parts:
            if "=" in part:
                key, value = part.split("=", 1)
                fields[key] = value

        if fields.get("TYPE") == "M" and "CMD" in fields:
            rtt = fields.get("RTT_MS", "--")
            self.motion_label.setText(
                "UNO CMD: {} | ACK RTT: {} ms".format(fields["CMD"], rtt)
            )
        elif fields.get("TYPE") == "C" and "CMD" in fields:
            rtt = fields.get("RTT_MS", "--")
            self.motion_label.setText(
                "UNO COLLECT: {} | ACK RTT: {} ms".format(fields["CMD"], rtt)
            )
        elif fields.get("TYPE") == "Z" and "CMD" in fields:
            rtt = fields.get("RTT_MS", "--")
            self.motion_label.setText(
                "UNO RADAR: {} | ACK RTT: {} ms".format(fields["CMD"], rtt)
            )
        elif fields.get("TYPE") == "A" and "CMD" in fields:
            rtt = fields.get("RTT_MS", "--")
            self.motion_label.setText(
                "UNO AUDIO: {} | ACK RTT: {} ms".format(fields["CMD"], rtt)
            )

    @pyqtSlot(dict)
    def update_telemetry(self, data):
        data = dict(data)
        # Invalid sensor values -> NaN (gaps in the plot, "--" in the labels).
        for k in ("TEXT", "TCAB"):
            if float(data.get(k, -99.9)) < -90.0:
                data[k] = math.nan
        if float(data.get("HUM", -1.0)) < 0.0:
            data["HUM"] = math.nan
        if float(data.get("PRESS", -1.0)) <= 0.0:
            data["PRESS"] = math.nan
        self.last_data = data

        pitch = float(data.get("PITCH", 0.0))
        roll = float(data.get("ROLL", 0.0))
        crit = float(data.get("CRIT", 35.0))
        margin = float(data.get("MARGIN", 0.0))
        tilt = float(data.get("TILT", 0.0))
        state = str(data.get("STATE", "GREEN"))

        self.pitch_gauge.setValue(pitch, crit, crit - 5.0)
        self.roll_gauge.setValue(roll, crit, crit - 5.0)

        self.tilt_status.setText(f"STATE: {state}")
        if state == "RED":
            self.tilt_status.setStyleSheet(f"color:{RED};font-weight:700;")
        elif state == "YELLOW":
            self.tilt_status.setStyleSheet(f"color:{YELLOW};font-weight:700;")
        else:
            self.tilt_status.setStyleSheet(f"color:{GREEN};font-weight:700;")

        self.critical_label.setText(f"CRIT: {crit:.1f} deg")
        self.margin_label.setText(f"MARGIN: {margin:+.1f} deg")
        self.nfront.setText(f"N FRONT: {data.get('NFRONT', 0):.2f} N")
        self.nrear.setText(f"N REAR: {data.get('NREAR', 0):.2f} N")
        self.nleft.setText(f"N LEFT: {data.get('NLEFT', 0):.2f} N")
        self.nright.setText(f"N RIGHT: {data.get('NRIGHT', 0):.2f} N")

        person = bool(data.get("PERSON", 0))
        score = float(data.get("PSCORE", 0.0))
        self.person_label.setText(
            f"PERSON: DETECTED ({score:.2f})" if person
            else f"PERSON: NO ({score:.2f})"
        )
        self.person_label.setStyleSheet(
            f"color:{GREEN};font-weight:700;" if person
            else f"color:{GRAY};font-weight:700;"
        )

        self.distance_label.setText(
            f"HC-SR04 L: {data.get('DL', -1):.1f} cm | R: {data.get('DR', -1):.1f} cm"
        )

        def num(key, spec):
            v = data.get(key, math.nan)
            return "--" if (not isinstance(v, (int, float)) or math.isnan(v)) else format(v, spec)

        self.ext_label.setText(f"EXT   T: {num('TEXT', '.1f')} C | HR: {num('HUM', '.0f')} %")
        self.cab_label.setText(f"CABINA T: {num('TCAB', '.1f')} C | P: {num('PRESS', '.1f')} hPa")

        st = int(float(data.get("ENVST", 0)))
        age = float(data.get("TAGE", 0))
        if age > 2500:
            self.env_status.setText(
                "UNO SIN RESPUESTA POR UART" + ("" if age >= 99999 else " ({:.0f} s)".format(age / 1000.0)))
            self.env_status.setStyleSheet(f"color:{RED};font-weight:700;")
        else:
            bmp = "OK" if st & 2 else ("SIN LECTURA" if st & 1 else "NO INICIA")
            dht = "OK" if st & 4 else "SIN LECTURA"
            self.env_status.setText(f"BMP280: {bmp} | DHT11: {dht}")
            ok = (st & 2) and (st & 4)
            self.env_status.setStyleSheet(f"color:{GREEN if ok else YELLOW};font-weight:700;")

        luz = float(data.get("FMEAN", -1.0))
        luz_txt = "?"
        if luz >= 0:
            luz_txt = f"{luz:.0f}" + (" OSCURO" if luz < 25 else " SATURADO" if luz > 230 else "")
        self.vision_label.setText(
            "VISION raw[{}] idx={} luz={} | {} ms".format(
                data.get("PRAW", "--"), int(data.get("PIDX", -1)), luz_txt, int(data.get("VMS", 0)))
        )

        col = str(data.get("COL", "O"))
        self.col_label.setText(
            "COLLECTOR: " + {"C": "ADELANTE", "E": "REVERSA", "O": "OFF"}.get(col, col))

        self.radar_actual = bool(data.get("RADAR", 0))
        if self.radar_actual:
            self.radar_wanted = True
        self.radar_widget.set_active(self.radar_actual)
        self.refresh_radar_button()

        self.uart_diag_label.setText(
            "UART: NICLA bad={} | UNO bad={} | RX overflow={} | malformed={}".format(
                int(data.get("URXBAD", 0)),
                int(data.get("NURXBAD", 0)),
                int(data.get("UOVF", 0)),
                int(data.get("ULINE", 0)),
            )
        )

        self.imu_label.setText(f"PITCH: {pitch:+.1f} | ROLL: {roll:+.1f} deg")

        self.torque_label.setText(
            f"TAU/MOTOR: {data.get('TAUREQ', 0):.4f} N*m | F: {data.get('FREQ', 0):.2f} N"
        )

        self.speed_label.setText(
            f"V THEORY: {data.get('VTHEORY', 0):.3f} | V IMU: {data.get('VIMU', 0):+.3f} m/s"
        )

        self.df_label.setText(
            "DFPLAYER: OK" if bool(data.get("DF", 0)) else "DFPLAYER: ERROR"
        )
        self.df_label.setStyleSheet(
            f"color:{GREEN};font-weight:700;" if bool(data.get("DF", 0))
            else f"color:{RED};font-weight:700;"
        )

        # ---------------- history ----------------
        for key in self.hist:
            self.hist[key].append(float(data.get(key, math.nan)))
            self.hist[key] = self.hist[key][-MAX_SAMPLES:]

        x = list(range(len(self.hist["VTHEORY"])))

        def set_curve(plot_key, keys):
            curves = self.plots[plot_key][1]
            for curve, key in zip(curves, keys):
                curve.setData(x[-len(self.hist[key]):], self.hist[key])

        set_curve("speed", ["VTHEORY", "VIMU"])
        set_curve("torque", ["TAUREQ", "TAUTOTAL"])
        set_curve("tilt", ["PITCH", "ROLL", "CRIT"])
        set_curve("normal", ["NFRONT", "NREAR", "NLEFT", "NRIGHT"])
        set_curve("distance", ["DL", "DR"])
        set_curve("traction", ["FREQ"])
        set_curve("accel", ["AX", "AY", "AZ"])
        set_curve("gyro", ["GX", "GY", "GZ"])
        set_curve("env", ["TEXT", "TCAB", "HUM"])

        # ---------------- battery ----------------
        battery = max(0, min(100, int(float(data.get("BAT", 0)))))
        remain = float(data.get("REMAIN", 0.0))
        self.battery_bar.setValue(battery)
        self.battery_text.setText(f"{battery}% / {remain:.1f} min")

        if battery <= 20:
            self.battery_bar.setStyleSheet(
                f"QProgressBar {{border:1px solid #24515a;background:#081214;text-align:center;color:{TEXT};}}"
                f"QProgressBar::chunk {{background:{RED};}}"
            )
        else:
            self.battery_bar.setStyleSheet(
                f"QProgressBar {{border:1px solid #24515a;background:#081214;text-align:center;color:{TEXT};}}"
                f"QProgressBar::chunk {{background:{GREEN};}}"
            )

    # --------------------------------------------------------
    # KEYBOARD
    # --------------------------------------------------------

    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            return
        mapping = {
            Qt.Key.Key_W: "F",
            Qt.Key.Key_A: "L",
            Qt.Key.Key_D: "R",
            Qt.Key.Key_X: "B",
            Qt.Key.Key_S: "S",
        }
        cmd = mapping.get(event.key())
        if cmd:
            self.worker.send_line(f"CMD,{cmd}")

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            return
        if event.key() in {
            Qt.Key.Key_W,
            Qt.Key.Key_A,
            Qt.Key.Key_D,
            Qt.Key.Key_X,
        }:
            self.worker.send_line("CMD,S")

    # --------------------------------------------------------
    # CLOSE
    # --------------------------------------------------------

    def closeEvent(self, event):
        try:
            self.worker.send_line("CMD,S")
            self.worker.send_line("COL,O")
            self.worker.send_line("RAD,0")
        except Exception:
            pass
        self.worker.stop()
        self.worker.wait(1500)
        event.accept()


def main():
    app = QApplication(sys.argv)
    pg.setConfigOptions(antialias=True)
    window = HMI()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
