# OSMIOR - NICLA Vision / OpenMV 5.x
# ============================================================
# Arquitectura:
#   HMI <-> WiFi/TCP <-> NICLA <-> UART4 <-> UNO
#   NICLA: IMU + Google person_detection + fisica + watchdog
#
# REQUISITO:
#   OpenMV 5.0+ / MicroPython 1.28+.
#   La API actual usa csi.CSI y ml.Model.
#
# MODELO DE PERSONA:
#   Google/TFLite Micro person_detect.tflite.
#   Modelo cuantizado de ~300 KB, entrada 96x96 grayscale y 2 clases.
#   En OpenMV actual se recomienda ponerlo en ROMFS desde:
#       Tools -> Machine Vision -> Open Model Zoo
#   y cargarlo con ml.Model('/rom/...') sin load_to_fb.
# ============================================================

import gc
import math
import os
import socket
import time

import csi
import imu
import ml
import network

from machine import UART, WDT

# ------------------------------------------------------------
# HARDWARE / COMM
# ------------------------------------------------------------

AP_SSID = "Rover_OSMIOR_AP"
AP_PASS = "rover1234"
TCP_PORT = 8080
UART_ID = 4
UART_BAUD = 9600

HMI_WATCHDOG_MS = 2500

TELEMETRY_PERIOD_MS = 200      # 5 Hz
IMU_PERIOD_MS = 50             # 20 Hz
VISION_PERIOD_MS = 500         # 2 Hz
MEM_REPORT_PERIOD_MS = 5000
GC_PERIOD_MS = 1000

# ------------------------------------------------------------
# CAMERA / MODEL
# ------------------------------------------------------------

CAMERA_STREAM = False
CAMERA_FRAMESIZE = csi.QQVGA       # 160x120
PERSON_THRESHOLD = 0.60
PERSON_ON_COUNT = 2
PERSON_OFF_COUNT = 2

person_model = None
person_model_ok = False
person_model_path = "NONE"
person_score = 0.0
person_detected = False
person_on_counter = 0
person_off_counter = 0

camera = None

# ------------------------------------------------------------
# PHYSICS / ROVER CONSTANTS
# ------------------------------------------------------------

MASS_KG = 3.0
LENGTH_M = 0.35
WIDTH_M = 0.35
HALF_LENGTH_M = LENGTH_M / 2.0
HALF_WIDTH_M = WIDTH_M / 2.0
COM_HEIGHT_M = 0.25
GRAVITY = 9.81

WHEEL_DIAMETER_M = 0.070
WHEEL_RADIUS_M = WHEEL_DIAMETER_M / 2.0

MOTOR_SUPPLY_V = 5.0
PWM_COMMAND = 220
PWM_DUTY = PWM_COMMAND / 255.0
GEAR_RATIO = 90.0

# From supplied motor data:
# 3 V -> 60 rpm, 4.5 V -> 90 rpm, 6 V -> 120 rpm.
NO_LOAD_RPM_PER_V = 20.0
VOLTAGE_EQUIVALENT = MOTOR_SUPPLY_V * PWM_DUTY
THEORETICAL_RPM = NO_LOAD_RPM_PER_V * VOLTAGE_EQUIVALENT
THEORETICAL_SPEED_MPS = THEORETICAL_RPM * math.pi * WHEEL_DIAMETER_M / 60.0

# Rolling resistance is only an explicit engineering assumption for
# torque-required estimation. It is not measured.
ROLLING_RESISTANCE_COEFF = 0.03

# Battery is an autonomy estimator, not a voltage/current measurement.
FULL_RUNTIME_MIN = 90.0
LOW_BATTERY_PCT = 20.0
LOW_BATTERY_RESET_PCT = 25.0

# Critical angle from centered-COM simplified equivalent support model.
CRITICAL_ANGLE_DEG = math.degrees(math.atan2(HALF_LENGTH_M, COM_HEIGHT_M))
WARNING_ANGLE_DEG = CRITICAL_ANGLE_DEG - 10.0
RED_ANGLE_DEG = CRITICAL_ANGLE_DEG - 5.0

# IMU signs. Flip to -1.0 after bench calibration if required.
PITCH_SIGN = 1.0
ROLL_SIGN = 1.0

# ------------------------------------------------------------
# STATE
# ------------------------------------------------------------

uart = None
server = None
client = None
client_addr = None

hmi_rx_buffer = ""
uno_rx_buffer = ""

last_hmi_rx_ms = 0
last_telemetry_ms = 0
last_imu_ms = 0
last_vision_ms = 0
last_gc_ms = 0
last_mem_report_ms = 0

# Motion command state.
motion_command = "S"
uno_state = "S"
uno_df_ok = 0
uno_pwm = PWM_COMMAND
uno_dl = -1.0
uno_dr = -1.0
last_uno_ack = "NONE"

# Drive-time estimator.
drive_time_ms = 0
last_drive_update_ms = 0

# IMU state.
ax_g = 0.0
ay_g = 0.0
az_g = 1.0
gx_dps = 0.0
gy_dps = 0.0
gz_dps = 0.0
pitch_deg = 0.0
roll_deg = 0.0
v_imu_mps = 0.0
a_forward_filtered = 0.0
orientation_initialized = False

# Events / audio.
last_audio = {
    2: -100000,
    3: -100000,
    4: -100000,
    5: -100000,
}
AUDIO_COOLDOWN_MS = 5000
PERSON_AUDIO_COOLDOWN_MS = 8000

person_previous = False
critical_audio_latched = False
low_battery_latched = False

# ------------------------------------------------------------
# HELPERS
# ------------------------------------------------------------

def now_ms():
    return time.ticks_ms()


def elapsed(start, end=None):
    if end is None:
        end = now_ms()
    return time.ticks_diff(end, start)


def mem_report(tag):
    gc.collect()
    try:
        print("MEM", tag, "free=", gc.mem_free(), "alloc=", gc.mem_alloc())
    except Exception:
        pass


def safe_float(value, default=0.0):
    try:
        return float(value)
    except Exception:
        return default


def safe_int(value, default=0):
    try:
        return int(value)
    except Exception:
        return default


# ------------------------------------------------------------
# MODEL DISCOVERY / MEMORY
# ------------------------------------------------------------

def find_person_model(root="/rom", depth=2):
    """Find a Google person_detection model anywhere in ROMFS.
    The IDE/model-zoo path can change; we avoid hard-coding a subfolder.
    """
    try:
        entries = os.listdir(root)
    except Exception:
        return None

    for entry in entries:
        lower = str(entry).lower()
        path = root + "/" + str(entry)

        if lower.endswith(".tflite") and (
            "person_detection" in lower or
            "person_detect" in lower
        ):
            return path

    if depth <= 0:
        return None

    for entry in entries:
        path = root + "/" + str(entry)
        lower = str(entry).lower()

        if lower in (".", ".."):
            continue

        # Try recursive listing only for plausible model folders.
        if not lower.endswith((".tflite", ".txt", ".bin", ".py")):
            found = find_person_model(path, depth - 1)
            if found:
                return found

    return None


def init_camera_and_model():
    global camera
    global person_model
    global person_model_ok
    global person_model_path

    mem_report("before_camera")

    camera = csi.CSI(stream=CAMERA_STREAM)
    camera.reset()
    camera.pixformat(csi.GRAYSCALE)
    camera.framesize(CAMERA_FRAMESIZE)
    camera.snapshot(time=1200)

    gc.collect()
    mem_report("after_camera")

    # Current OpenMV 5.x manages ML memory with the unified allocator.
    # Do NOT use load_to_fb=True on v5.x; that keyword was removed.
    person_model_path = find_person_model("/rom", 2)

    if person_model_path is None:
        print("MODEL NOT FOUND IN /rom")
        print("Add Google person_detection from Open Model Zoo to ROMFS.")
        person_model_ok = False
        return

    try:
        print("Loading model:", person_model_path)
        gc.collect()
        person_model = ml.Model(person_model_path)
        person_model_ok = True

        print("MODEL LEN:", getattr(person_model, "len", -1))
        print("MODEL RAM:", getattr(person_model, "ram", -1))
        print("MODEL INPUT:", getattr(person_model, "input_shape", None))
        print("MODEL OUTPUT:", getattr(person_model, "output_shape", None))
        print("MODEL LABELS:", getattr(person_model, "labels", None))
        mem_report("after_model")

    except Exception as exc:
        person_model = None
        person_model_ok = False
        print("MODEL LOAD ERROR:", repr(exc))
        mem_report("model_error")


def detect_person_once():
    global person_score

    if not person_model_ok or person_model is None or camera is None:
        person_score = 0.0
        return False

    t0 = now_ms()

    try:
        img = camera.snapshot()
        outputs = person_model.predict([img])

        # Google/TFLite Micro person_detect model: [no_person, person].
        raw = outputs[0]
        flat = raw.flatten()
        n = len(flat)

        if n >= 2:
            score_no_person = safe_float(flat[0], 0.0)
            score_person = safe_float(flat[1], 0.0)

            # If labels are available and explicitly identify person,
            # use that index instead of assuming index 1.
            labels = getattr(person_model, "labels", None)
            if labels:
                for idx in range(min(len(labels), n)):
                    label = str(labels[idx]).lower()
                    if "person" in label or "dude" in label:
                        score_person = safe_float(flat[idx], score_person)
                        break

            person_score = score_person
            detected = score_person >= PERSON_THRESHOLD

        elif n == 1:
            person_score = safe_float(flat[0], 0.0)
            detected = person_score >= PERSON_THRESHOLD

        else:
            person_score = 0.0
            detected = False

        dt = max(1, elapsed(t0))
        print("VISION", "score=", round(person_score, 3), "fps=", round(1000.0 / dt, 1))
        return detected

    except Exception as exc:
        print("VISION ERROR:", repr(exc))
        person_score = 0.0
        return False


def update_person_hysteresis(raw_detected):
    global person_detected
    global person_on_counter
    global person_off_counter

    if raw_detected:
        person_on_counter += 1
        person_off_counter = 0

        if person_on_counter >= PERSON_ON_COUNT:
            person_detected = True

    else:
        person_off_counter += 1
        person_on_counter = 0

        if person_off_counter >= PERSON_OFF_COUNT:
            person_detected = False


# ------------------------------------------------------------
# IMU / PHYSICS
# ------------------------------------------------------------

def read_imu_update():
    global ax_g, ay_g, az_g
    global gx_dps, gy_dps, gz_dps
    global pitch_deg, roll_deg
    global v_imu_mps, a_forward_filtered
    global orientation_initialized, last_imu_ms

    now = now_ms()

    accel = imu.acceleration_mg()
    gyro = imu.angular_rate_mdps()

    ax_g = safe_float(accel[0], 0.0) / 1000.0
    ay_g = safe_float(accel[1], 0.0) / 1000.0
    az_g = safe_float(accel[2], 1.0) / 1000.0

    gx_dps = safe_float(gyro[0], 0.0) / 1000.0
    gy_dps = safe_float(gyro[1], 0.0) / 1000.0
    gz_dps = safe_float(gyro[2], 0.0) / 1000.0

    pitch_acc = math.degrees(math.atan2(
        ax_g,
        math.sqrt(ay_g * ay_g + az_g * az_g),
    )) * PITCH_SIGN

    roll_acc = math.degrees(math.atan2(
        ay_g,
        math.sqrt(ax_g * ax_g + az_g * az_g),
    )) * ROLL_SIGN

    if not orientation_initialized or last_imu_ms == 0:
        pitch_deg = pitch_acc
        roll_deg = roll_acc
        orientation_initialized = True
        last_imu_ms = now
        return

    dt = elapsed(last_imu_ms, now) / 1000.0
    last_imu_ms = now
    dt = min(0.2, max(0.001, dt))

    # Gyro axes are board-dependent. These mappings are the usual
    # body-frame interpretation for pitch/roll; flip signs above if needed.
    pitch_gyro = pitch_deg + (gy_dps * dt * PITCH_SIGN)
    roll_gyro = roll_deg + (gx_dps * dt * ROLL_SIGN)

    alpha = 0.98
    pitch_deg = alpha * pitch_gyro + (1.0 - alpha) * pitch_acc
    roll_deg = alpha * roll_gyro + (1.0 - alpha) * roll_acc

    # IMU-only longitudinal speed estimate.
    # Valid only as an estimate: no encoder and no external velocity reference.
    pitch_rad = math.radians(pitch_deg)
    forward_acc_mps2 = (ax_g - math.sin(pitch_rad)) * GRAVITY

    a_forward_filtered = (
        0.85 * a_forward_filtered +
        0.15 * forward_acc_mps2
    )

    if motion_command == "F":
        v_imu_mps += a_forward_filtered * dt
    elif motion_command == "B":
        v_imu_mps -= a_forward_filtered * dt
    else:
        # Turns do not define a scalar forward velocity here.
        v_imu_mps *= 0.85

    # On an explicit stop we deliberately zero the user-facing estimate.
    if motion_command == "S":
        v_imu_mps = 0.0

    v_imu_mps = max(-2.0, min(2.0, v_imu_mps))


def calculate_loads():
    pitch = math.radians(pitch_deg)
    roll = math.radians(roll_deg)

    n_front = MASS_KG * GRAVITY * (
        HALF_LENGTH_M * math.cos(pitch) - COM_HEIGHT_M * math.sin(pitch)
    ) / LENGTH_M

    n_rear = MASS_KG * GRAVITY * (
        HALF_LENGTH_M * math.cos(pitch) + COM_HEIGHT_M * math.sin(pitch)
    ) / LENGTH_M

    n_left = MASS_KG * GRAVITY * (
        HALF_WIDTH_M * math.cos(roll) + COM_HEIGHT_M * math.sin(roll)
    ) / WIDTH_M

    n_right = MASS_KG * GRAVITY * (
        HALF_WIDTH_M * math.cos(roll) - COM_HEIGHT_M * math.sin(roll)
    ) / WIDTH_M

    return (
        max(0.0, n_front),
        max(0.0, n_rear),
        max(0.0, n_left),
        max(0.0, n_right),
    )


def calculate_tilt_state():
    tilt = max(abs(pitch_deg), abs(roll_deg))

    if tilt >= RED_ANGLE_DEG:
        return "RED", tilt
    if tilt >= WARNING_ANGLE_DEG:
        return "YELLOW", tilt
    return "GREEN", tilt


def calculate_torque_required():
    # External torque demand from gravity + rolling resistance.
    # We do NOT claim this is measured motor torque.
    slope = math.radians(max(abs(pitch_deg), abs(roll_deg)))

    force_grade = MASS_KG * GRAVITY * math.sin(slope)
    force_rr = ROLLING_RESISTANCE_COEFF * MASS_KG * GRAVITY * math.cos(slope)

    force_required = max(0.0, force_grade + force_rr)

    torque_per_motor = force_required * WHEEL_RADIUS_M / 2.0
    torque_total = torque_per_motor * 2.0

    return torque_per_motor, torque_total, force_required


def update_drive_time(now):
    global drive_time_ms, last_drive_update_ms

    if last_drive_update_ms == 0:
        last_drive_update_ms = now
        return

    dt = elapsed(last_drive_update_ms, now)
    last_drive_update_ms = now

    if motion_command != "S" and dt > 0:
        drive_time_ms += dt


def battery_estimate():
    used_min = drive_time_ms / 60000.0
    remaining_min = max(0.0, FULL_RUNTIME_MIN - used_min)
    pct = 100.0 * remaining_min / FULL_RUNTIME_MIN
    return max(0.0, min(100.0, pct)), remaining_min


# ------------------------------------------------------------
# AUDIO
# ------------------------------------------------------------

def play_audio(track):
    now = now_ms()

    cooldown = PERSON_AUDIO_COOLDOWN_MS if track == 5 else AUDIO_COOLDOWN_MS
    last = last_audio.get(track, -100000)

    if elapsed(last, now) < cooldown:
        return False

    send_uno("AUD," + str(track))
    last_audio[track] = now
    return True


def update_audio_events():
    global person_previous
    global critical_audio_latched
    global low_battery_latched

    tilt_state, tilt = calculate_tilt_state()
    battery_pct, _ = battery_estimate()

    # Person: rising edge + cooldown.
    if person_detected and not person_previous:
        play_audio(5)
    person_previous = person_detected

    # Critical tilt: RED at 5 deg before theoretical critical.
    if tilt_state == "RED":
        if not critical_audio_latched:
            play_audio(4)
            critical_audio_latched = True
    elif tilt < RED_ANGLE_DEG - 2.0:
        critical_audio_latched = False

    # Low battery: one event per low-battery episode.
    if battery_pct <= LOW_BATTERY_PCT:
        if not low_battery_latched:
            play_audio(2)
            low_battery_latched = True
    elif battery_pct >= LOW_BATTERY_RESET_PCT:
        low_battery_latched = False


# ------------------------------------------------------------
# UNO UART
# ------------------------------------------------------------

def send_uno(line):
    try:
        uart.write((line + "\n").encode("ascii"))
        return True
    except Exception as exc:
        print("UART TX ERROR", repr(exc))
        return False


def apply_motion_command(cmd):
    global motion_command

    cmd = cmd.upper()

    if cmd not in ("F", "B", "L", "R", "S"):
        return False

    motion_command = cmd
    send_uno("CMD," + cmd)
    return True


def parse_uno_line(line):
    global uno_dl, uno_dr, uno_df_ok, uno_pwm
    global uno_state, last_uno_ack

    line = line.strip()

    if not line:
        return

    if line.startswith("TEL,"):
        parts = line.split(",")
        for token in parts[1:]:
            if token.startswith("DL="):
                uno_dl = safe_float(token[3:], -1.0)
            elif token.startswith("DR="):
                uno_dr = safe_float(token[3:], -1.0)
            elif token.startswith("DF="):
                uno_df_ok = safe_int(token[3:], 0)
            elif token.startswith("PWM="):
                uno_pwm = safe_int(token[4:], PWM_COMMAND)
            elif token.startswith("STATE="):
                uno_state = token[6:]
        return

    if line.startswith("ACK,"):
        last_uno_ack = line[4:]
        send_hmi("EVT;UNO_ACK=" + last_uno_ack)
        return

    if line.startswith("READY,"):
        send_hmi("EVT;UNO=" + line[6:])
        return

    if line.startswith("PONG"):
        send_hmi("EVT;UNO=PONG")
        return

    if line.startswith("ERR,"):
        send_hmi("EVT;UNO_ERR=" + line[4:])
        return


def read_uno_uart():
    global uno_rx_buffer

    try:
        if not uart.any():
            return

        data = uart.read()
        if not data:
            return

        uno_rx_buffer += data.decode("ascii", "ignore")

        if len(uno_rx_buffer) > 400:
            uno_rx_buffer = ""
            return

        while "\n" in uno_rx_buffer:
            line, uno_rx_buffer = uno_rx_buffer.split("\n", 1)
            parse_uno_line(line)

    except Exception as exc:
        print("UART RX ERROR", repr(exc))


# ------------------------------------------------------------
# HMI TCP
# ------------------------------------------------------------

def send_hmi(line):
    global client

    if client is None:
        return False

    try:
        client.sendall((line + "\n").encode("ascii"))
        return True
    except Exception:
        close_client()
        return False


def close_client():
    global client, client_addr, hmi_rx_buffer

    # Safety first: network loss -> stop rover.
    apply_motion_command("S")

    if client is not None:
        try:
            client.close()
        except Exception:
            pass

    client = None
    client_addr = None
    hmi_rx_buffer = ""


def accept_hmi():
    global client, client_addr, hmi_rx_buffer, last_hmi_rx_ms

    if client is not None:
        return

    try:
        conn, addr = server.accept()
        conn.settimeout(0.02)

        client = conn
        client_addr = addr
        hmi_rx_buffer = ""
        last_hmi_rx_ms = now_ms()

        print("HMI CONNECTED", addr)

        send_hmi("READY;OSMIOR=1;MODE=MANUAL")
        send_hmi(
            "STAT;MODEL={};MRAM={};MLEN={};FREE={}".format(
                1 if person_model_ok else 0,
                getattr(person_model, "ram", 0) if person_model_ok else 0,
                getattr(person_model, "len", 0) if person_model_ok else 0,
                gc.mem_free(),
            )
        )

    except OSError:
        pass


def handle_hmi_line(line):
    global last_hmi_rx_ms

    line = line.strip()
    if not line:
        return

    last_hmi_rx_ms = now_ms()
    upper = line.upper()

    if upper == "HELLO":
        send_hmi("READY;OSMIOR=1;MODE=MANUAL")
        return

    if upper == "HB":
        return

    if upper == "PING":
        send_hmi("PONG")
        return

    # New protocol.
    if upper.startswith("CMD,") and len(upper) >= 5:
        apply_motion_command(upper[4])
        return

    if upper.startswith("AUD,"):
        track = safe_int(upper[4:], 0)
        if 1 <= track <= 5:
            if track == 3:
                # Manual person query from HMI.
                play_audio(3)
            else:
                send_uno("AUD," + str(track))
        return

    # Compatibility with older HMI.
    if upper in ("F", "B", "L", "R", "S"):
        apply_motion_command(upper)
        return

    if upper.startswith("AUD:"):
        track = safe_int(upper[4:], 0)
        if 1 <= track <= 5:
            send_uno("AUD," + str(track))
        return


def read_hmi():
    global hmi_rx_buffer

    if client is None:
        return

    try:
        data = client.recv(512)

        if not data:
            close_client()
            return

        hmi_rx_buffer += data.decode("ascii", "ignore")

        if len(hmi_rx_buffer) > 1200:
            hmi_rx_buffer = ""
            return

        while "\n" in hmi_rx_buffer:
            line, hmi_rx_buffer = hmi_rx_buffer.split("\n", 1)
            handle_hmi_line(line)

    except OSError:
        # Timeout is normal.
        pass

    except Exception:
        close_client()


def watchdog_hmi():
    if client is None:
        return

    if elapsed(last_hmi_rx_ms) > HMI_WATCHDOG_MS:
        print("HMI WATCHDOG -> STOP")
        send_hmi("EVT;WATCHDOG=STOP")
        close_client()


# ------------------------------------------------------------
# TELEMETRY
# ------------------------------------------------------------

def build_telemetry():
    n_front, n_rear, n_left, n_right = calculate_loads()
    tilt_state, tilt = calculate_tilt_state()
    torque_motor, torque_total, force_required = calculate_torque_required()
    battery_pct, remaining_min = battery_estimate()

    return (
        "TEL;"
        "AX={:.3f};"
        "AY={:.3f};"
        "AZ={:.3f};"
        "GX={:.2f};"
        "GY={:.2f};"
        "GZ={:.2f};"
        "PITCH={:.2f};"
        "ROLL={:.2f};"
        "TILT={:.2f};"
        "CRIT={:.2f};"
        "MARGIN={:.2f};"
        "STATE={};"
        "DL={:.1f};"
        "DR={:.1f};"
        "PERSON={};"
        "PSCORE={:.3f};"
        "MODEL={};"
        "VTHEORY={:.3f};"
        "VIMU={:.3f};"
        "RPM={:.1f};"
        "VOUT={:.3f};"
        "TAUREQ={:.4f};"
        "TAUTOTAL={:.4f};"
        "FREQ={:.3f};"
        "NFRONT={:.3f};"
        "NREAR={:.3f};"
        "NLEFT={:.3f};"
        "NRIGHT={:.3f};"
        "BAT={:.1f};"
        "REMAIN={:.1f};"
        "DF={};"
        "PWM={};"
        "MODE=MANUAL"
    ).format(
        ax_g, ay_g, az_g,
        gx_dps, gy_dps, gz_dps,
        pitch_deg, roll_deg,
        tilt,
        CRITICAL_ANGLE_DEG,
        CRITICAL_ANGLE_DEG - tilt,
        tilt_state,
        uno_dl, uno_dr,
        1 if person_detected else 0,
        person_score,
        1 if person_model_ok else 0,
        THEORETICAL_SPEED_MPS if motion_command != "S" else 0.0,
        v_imu_mps,
        THEORETICAL_RPM if motion_command != "S" else 0.0,
        VOLTAGE_EQUIVALENT,
        torque_motor,
        torque_total,
        force_required,
        n_front, n_rear, n_left, n_right,
        battery_pct,
        remaining_min,
        uno_df_ok,
        uno_pwm,
    )


# ------------------------------------------------------------
# INIT
# ------------------------------------------------------------

def init_system():
    global uart, server, last_hmi_rx_ms

    # Model is intentionally initialized BEFORE WiFi sockets so we reserve
    # scarce transient memory while the system is still quiet.
    init_camera_and_model()

    mem_report("before_comm")

    uart = UART(UART_ID, baudrate=UART_BAUD, timeout=10)

    ap_id = getattr(network.WLAN, "IF_AP", getattr(network, "AP_IF", None))
    ap = network.WLAN(ap_id)
    ap.active(True)

    try:
        ap.config(ssid=AP_SSID, password=AP_PASS)
    except Exception:
        try:
            ap.config(essid=AP_SSID, key=AP_PASS)
        except Exception:
            pass

    time.sleep_ms(500)

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", TCP_PORT))
    server.listen(1)
    server.settimeout(0.02)

    last_hmi_rx_ms = now_ms()

    print("============================================")
    print("OSMIOR NICLA GATEWAY")
    print("IP:", ap.ifconfig()[0])
    print("TCP:", TCP_PORT)
    print("UART4: SDA=TX / SCL=RX")
    print("MODEL OK:", person_model_ok)
    print("MODEL PATH:", person_model_path)
    print("CRIT ANGLE:", CRITICAL_ANGLE_DEG)
    print("V THEORY:", THEORETICAL_SPEED_MPS)
    print("============================================")

    # Start UNO stopped.
    send_uno("CMD,S")


# ------------------------------------------------------------
# MAIN
# ------------------------------------------------------------

init_system()

wdt = WDT(timeout=8000)

while True:
    now = now_ms()

    accept_hmi()
    read_hmi()
    read_uno_uart()

    if elapsed(last_imu_ms, now) >= IMU_PERIOD_MS:
        read_imu_update()

    update_drive_time(now)

    if elapsed(last_vision_ms, now) >= VISION_PERIOD_MS:
        last_vision_ms = now
        raw_detected = detect_person_once()
        update_person_hysteresis(raw_detected)

    update_audio_events()

    if elapsed(last_telemetry_ms, now) >= TELEMETRY_PERIOD_MS:
        last_telemetry_ms = now
        if client is not None:
            send_hmi(build_telemetry())

    watchdog_hmi()

    if elapsed(last_gc_ms, now) >= GC_PERIOD_MS:
        last_gc_ms = now
        gc.collect()

    if elapsed(last_mem_report_ms, now) >= MEM_REPORT_PERIOD_MS:
        last_mem_report_ms = now
        if person_model_ok:
            print("RUN MEM free=", gc.mem_free(), "model_ram=", getattr(person_model, "ram", -1))

    wdt.feed()
    time.sleep_ms(5)
