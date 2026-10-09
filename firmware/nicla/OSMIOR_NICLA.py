# OSMIOR V3 - NICLA Vision / OpenMV 5.x
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

TELEMETRY_PERIOD_MS = 200      # HMI 5 Hz; UNO UART telemetry poll at 2 Hz
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

# UART half-duplex scheduler: motion/audio queued while UNO is transmitting.
UART_POLL_PERIOD_MS = 500
UART_BUSY_MOTION_MS = 12
UART_BUSY_AUDIO_MS = 18
UART_BUSY_POLL_MS = 55
UART_BUSY_GUARD_MS = 2
uart_busy_until_ms = 0
last_uart_poll_ms = 0
pending_motion = None          # (command, HMI sequence)
pending_audio = []             # [(track, HMI sequence), ...]
pending_uart_acks = []         # [(type, code, seq, sent_ticks), ...]
uart_rx_bad_bytes = 0
uno_rx_bad_bytes = 0
uno_sw_overflows = 0
uno_line_overflows = 0

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
# AUDIO / NICLA -> UNO UART scheduler
# ------------------------------------------------------------
def queue_uart_ack(kind, code, seq, sent_ms):
    # Bounded tracking list; stale ACK records must not consume RAM forever.
    if len(pending_uart_acks) >= 10:
        pending_uart_acks.pop(0)
    pending_uart_acks.append((kind, str(code), int(seq or 0), sent_ms))


def enqueue_motion(cmd, seq=0):
    global pending_motion, motion_command
    cmd = str(cmd).upper()
    if cmd not in ("F", "B", "L", "R", "S"):
        return False

    motion_command = cmd
    if pending_motion is not None:
        old_cmd, old_seq = pending_motion
        if old_seq:
            send_hmi("EVT;CMD_SUPERSEDED;SEQ={}".format(old_seq))
    pending_motion = (cmd, int(seq or 0))  # last motion wins; STOP can supersede F
    return True


def enqueue_audio(track, seq=0):
    if track < 1 or track > 5:
        return False
    if len(pending_audio) >= 5:
        pending_audio.pop(0)
    pending_audio.append((int(track), int(seq or 0)))
    return True


def send_uno(line, seq=0):
    """Compatibility wrapper; queues commands instead of writing full text frames."""
    upper = line.strip().upper()
    if upper.startswith("CMD,") and len(upper) >= 5:
        return enqueue_motion(upper[4], seq)
    if upper.startswith("AUD,"):
        track = safe_int(upper[4:].split(",", 1)[0], 0)
        return enqueue_audio(track, seq)
    if upper in ("F", "B", "L", "R", "S"):
        return enqueue_motion(upper, seq)
    return False


def play_audio(track, seq=0, force=False):
    now = now_ms()
    cooldown = PERSON_AUDIO_COOLDOWN_MS if track == 5 else AUDIO_COOLDOWN_MS
    last = last_audio.get(track, -100000)

    # Track 3 is an explicit user request, so don't silently suppress it.
    if not force and track != 3 and elapsed(last, now) < cooldown:
        return False

    queued = enqueue_audio(track, seq)
    if queued:
        last_audio[track] = now
    return queued


def update_audio_events():
    global person_previous
    global critical_audio_latched
    global low_battery_latched

    tilt_state, tilt = calculate_tilt_state()
    battery_pct, _ = battery_estimate()

    if person_detected and not person_previous:
        play_audio(5)
    person_previous = person_detected

    if tilt_state == "RED":
        if not critical_audio_latched:
            play_audio(4)
            critical_audio_latched = True
    elif tilt < RED_ANGLE_DEG - 2.0:
        critical_audio_latched = False

    if battery_pct <= LOW_BATTERY_PCT:
        if not low_battery_latched:
            play_audio(2)
            low_battery_latched = True
    elif battery_pct >= LOW_BATTERY_RESET_PCT:
        low_battery_latched = False


# ------------------------------------------------------------
# UNO UART - ASCII bytes only, no Unicode decode
# ------------------------------------------------------------
def parse_uno_line(line):
    global uno_dl, uno_dr, uno_df_ok, uno_pwm
    global uno_state, last_uno_ack, uart_busy_until_ms
    global uno_rx_bad_bytes, uno_sw_overflows, uno_line_overflows

    line = line.strip()
    if not line:
        return

    now = now_ms()

    if line.startswith("T,"):
        # Compact response: T,DL_cm,DR_cm,DF,STATE,RX_BAD,OVF,LINE_BAD
        parts = line.split(",")
        if len(parts) >= 8:
            uno_dl = safe_float(parts[1], -1.0)
            uno_dr = safe_float(parts[2], -1.0)
            uno_df_ok = safe_int(parts[3], 0)
            uno_state = parts[4]
            uno_rx_bad_bytes = safe_int(parts[5], 0)
            uno_sw_overflows = safe_int(parts[6], 0)
            uno_line_overflows = safe_int(parts[7], 0)
        uart_busy_until_ms = time.ticks_add(now, UART_BUSY_GUARD_MS)
        return

    if line.startswith("K,"):
        # Compact ACK: K,M,F or K,A,5
        parts = line.split(",")
        if len(parts) >= 3:
            kind = parts[1]
            code = parts[2]
            last_uno_ack = kind + "," + code
            matched = None
            for i, record in enumerate(pending_uart_acks):
                if record[0] == kind and record[1] == code:
                    matched = pending_uart_acks.pop(i)
                    break

            if matched is not None:
                _kind, _code, seq, sent_ms = matched
                uart_ms = elapsed(sent_ms, now)
                event = "EVT;ACK;TYPE={};CMD={};SEQ={};UART_MS={}".format(
                    kind, code, seq, uart_ms
                )
                print("UART ACK", kind, code, "seq=", seq, "latency_ms=", uart_ms)
                send_hmi(event)
            else:
                send_hmi("EVT;ACK;TYPE={};CMD={};SEQ=0".format(kind, code))
        uart_busy_until_ms = time.ticks_add(now, UART_BUSY_GUARD_MS)
        return

    if line.startswith("READY,"):
        send_hmi("EVT;UNO=" + line[6:])
        return

    if line == "PONG":
        send_hmi("EVT;UNO=PONG")
        return


def read_uno_uart():
    global uno_rx_buffer, uart_rx_bad_bytes

    try:
        count = uart.any()
        if not count:
            return
        data = uart.read(count)
        if not data:
            return

        # Parse bytes directly. Never decode random UART bytes as Unicode.
        for byte in data:
            if not isinstance(byte, int):
                byte = ord(byte)

            if byte == 13:  # CR
                continue
            if byte == 10:  # LF
                if uno_rx_buffer:
                    parse_uno_line(uno_rx_buffer)
                uno_rx_buffer = ""
                continue

            if 32 <= byte <= 126:
                if len(uno_rx_buffer) < 100:
                    uno_rx_buffer += chr(byte)
                else:
                    uno_rx_buffer = ""
                    uart_rx_bad_bytes += 1
            else:
                uart_rx_bad_bytes += 1
                # Ignore invalid byte but preserve the rest of the frame.

    except Exception as exc:
        print("UART RX ERROR", repr(exc))


def expire_uart_acks(now):
    # A lost/corrupt ACK must not block telemetry polling or future commands forever.
    for i in range(len(pending_uart_acks) - 1, -1, -1):
        record = pending_uart_acks[i]
        if elapsed(record[3], now) > 300:
            pending_uart_acks.pop(i)
            kind, code, seq, sent_ms = record
            print("UART ACK TIMEOUT", kind, code, "seq=", seq)
            send_hmi("EVT;ACK_TIMEOUT;TYPE={};CMD={};SEQ={}".format(kind, code, seq))


def flush_uno_actions(now):
    """Send one short transaction at a time to avoid UNO SoftwareSerial TX/RX overlap."""
    global pending_motion, last_uart_poll_ms, uart_busy_until_ms

    if time.ticks_diff(now, uart_busy_until_ms) < 0:
        return

    if pending_motion is not None:
        cmd, seq = pending_motion
        pending_motion = None
        sent_ms = now_ms()
        try:
            uart.write(cmd.encode("ascii"))  # one byte; UNO acts immediately
            queue_uart_ack("M", cmd, seq, sent_ms)
            uart_busy_until_ms = time.ticks_add(sent_ms, UART_BUSY_MOTION_MS)
            print("UART TX motion", cmd, "seq=", seq)
        except Exception as exc:
            print("UART TX ERROR motion", repr(exc))
        return

    if pending_audio:
        track, seq = pending_audio.pop(0)
        sent_ms = now_ms()
        try:
            uart.write(("A" + str(track)).encode("ascii"))  # two bytes
            queue_uart_ack("A", track, seq, sent_ms)
            uart_busy_until_ms = time.ticks_add(sent_ms, UART_BUSY_AUDIO_MS)
            print("UART TX audio", track, "seq=", seq)
        except Exception as exc:
            print("UART TX ERROR audio", repr(exc))
        return

    # Poll only when no command is queued and no ACK is outstanding.
    if (not pending_uart_acks and
            elapsed(last_uart_poll_ms, now) >= UART_POLL_PERIOD_MS):
        try:
            uart.write(b"?")
            last_uart_poll_ms = now
            uart_busy_until_ms = time.ticks_add(now, UART_BUSY_POLL_MS)
        except Exception as exc:
            print("UART TX ERROR poll", repr(exc))


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
    enqueue_motion("S", 0)

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

    if upper.startswith("CMD,"):
        parts = upper.split(",")
        if len(parts) >= 2 and parts[1] and parts[1][0] in ("F", "B", "L", "R", "S"):
            seq = safe_int(parts[2], 0) if len(parts) >= 3 else 0
            enqueue_motion(parts[1][0], seq)
        return

    if upper.startswith("AUD,"):
        parts = upper.split(",")
        track = safe_int(parts[1], 0) if len(parts) >= 2 else 0
        seq = safe_int(parts[2], 0) if len(parts) >= 3 else 0
        if 1 <= track <= 5:
            # Track 3 is the explicit HMI query; play immediately when requested.
            if track == 3:
                play_audio(3, seq, force=True)
            else:
                play_audio(track, seq)
        return

    if upper in ("F", "B", "L", "R", "S"):
        enqueue_motion(upper, 0)
        return

    if upper.startswith("AUD:"):
        track = safe_int(upper[4:], 0)
        if 1 <= track <= 5:
            play_audio(track)
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
        "URXBAD={};"
        "UOVF={};"
        "NURXBAD={};"
        "ULINE={};"
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
        uart_rx_bad_bytes,
        uno_sw_overflows,
        uno_rx_bad_bytes,
        uno_line_overflows,
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
    print("OSMIOR NICLA V3 GATEWAY")
    print("IP:", ap.ifconfig()[0])
    print("TCP:", TCP_PORT)
    print("UART4: SDA=TX / SCL=RX")
    print("MODEL OK:", person_model_ok)
    print("MODEL PATH:", person_model_path)
    print("CRIT ANGLE:", CRITICAL_ANGLE_DEG)
    print("V THEORY:", THEORETICAL_SPEED_MPS)
    print("============================================")

    # Start UNO stopped.
    enqueue_motion("S", 0)


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
    flush_uno_actions(now)

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
