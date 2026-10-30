import binascii
import csv
import math
import re
import struct
import threading
import time
import tkinter as tk
import tkinter.font as tkfont
from collections import deque
from decimal import Decimal, InvalidOperation, ROUND_HALF_UP
from pathlib import Path
from queue import Empty, Queue
from tkinter import filedialog, ttk

import serial
from serial.tools import list_ports
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

BAUDRATE = 128000

SYNC1 = 0x55
SYNC2 = 0xAA

SAMPLE_INTERVAL = 0.05
BUFFER_DURATION = 60
BUFFER_SIZE = int(BUFFER_DURATION / SAMPLE_INTERVAL)
TIME_BASE = [index * SAMPLE_INTERVAL for index in range(BUFFER_SIZE)]

ch1 = deque([0] * BUFFER_SIZE, maxlen=BUFFER_SIZE)
ch2 = deque([0] * BUFFER_SIZE, maxlen=BUFFER_SIZE)
ch3 = deque([0] * BUFFER_SIZE, maxlen=BUFFER_SIZE)
ise_samples = deque()
data_lock = threading.Lock()


def is_valid_group_number(value):
    return value.isdecimal() and 1 <= int(value) <= 32


def parse_setpoint(value):
    if not value.isdecimal():
        raise ValueError("Sollwert muss eine ganze Zahl sein (0 bis 300 mm)")
    setpoint = int(value)
    if not 0 <= setpoint <= 300:
        raise ValueError("Sollwert muss zwischen 0 und 300 mm liegen")
    return setpoint


def parse_factor(value):
    normalized = value.strip().replace(",", ".")
    if not re.fullmatch(r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)", normalized):
        raise ValueError("Faktor muss eine Zahl in mm sein")
    try:
        factor = Decimal(normalized)
    except InvalidOperation as exc:
        raise ValueError("Faktor muss eine gültige Zahl sein") from exc
    if not factor.is_finite() or not -300 <= factor <= 300:
        raise ValueError("Faktor muss zwischen -300 und 300 mm liegen")
    return factor


def load_modulation_csv(path):
    profile = []
    with Path(path).open("r", encoding="utf-8-sig", newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        if reader.fieldnames is None or not {"time_ms", "modulation"}.issubset(
            reader.fieldnames
        ):
            raise ValueError(
                "CSV benötigt die Spalten 'time_ms' und 'modulation'"
            )

        for line_number, row in enumerate(reader, start=2):
            if None in row:
                raise ValueError(
                    f"CSV-Zeile {line_number}: mehr Spalten als erwartet"
                )
            time_value = (row.get("time_ms") or "").strip()
            modulation_value = (row.get("modulation") or "").strip()
            if not time_value and not modulation_value:
                continue
            try:
                timestamp_ms = float(time_value)
                modulation = float(modulation_value)
            except ValueError as exc:
                raise ValueError(
                    f"Ungültiger Zahlenwert in CSV-Zeile {line_number}"
                ) from exc

            if (
                not math.isfinite(timestamp_ms)
                or not math.isfinite(modulation)
                or timestamp_ms < 0
                or not 0 <= modulation <= 1
            ):
                raise ValueError(
                    f"CSV-Zeile {line_number}: Zeit muss >= 0 ms und "
                    "Modulation zwischen 0 und 1 sein"
                )
            if profile and timestamp_ms <= profile[-1][0]:
                raise ValueError(
                    f"CSV-Zeile {line_number}: Zeitwerte müssen streng "
                    "aufsteigend sein"
                )
            profile.append((timestamp_ms, modulation))

    if not profile:
        raise ValueError("Die CSV-Datei enthält keine Profilwerte")
    return profile


def modulation_at_time(profile, elapsed_ms, repeat=False):
    duration_ms = profile[-1][0] if profile else 0
    if repeat and duration_ms > 0 and elapsed_ms >= duration_ms:
        elapsed_ms %= duration_ms

    modulation = 0.0
    for timestamp_ms, value in profile:
        if timestamp_ms > elapsed_ms:
            break
        modulation = value
    return modulation


def calculate_ise(samples, now):
    cutoff = now - BUFFER_DURATION
    integral = 0.0
    for index, (timestamp, actual, setpoint) in enumerate(samples):
        next_timestamp = (
            samples[index + 1][0]
            if index + 1 < len(samples)
            else min(now, timestamp + SAMPLE_INTERVAL)
        )
        interval_start = max(timestamp, cutoff)
        interval_end = min(next_timestamp, now)
        if actual >= 0 and interval_end > interval_start:
            error = actual - setpoint
            integral += error**2 * (interval_end - interval_start)
    return integral


def format_ise(value):
    formatted = f"{value:.2e}" if value > 10000 else f"{value:.1f}"
    return f"ISE (60 s): {formatted} mm²·s"


def extract_telemetry_frames(pending):
    values = []
    sync_header = bytes((SYNC1, SYNC2))
    while True:
        frame_start = pending.find(sync_header)
        if frame_start < 0:
            pending[:] = b"\x55" if pending.endswith(b"\x55") else b""
            break
        if frame_start:
            del pending[:frame_start]
        if len(pending) < 8:
            break

        values.append(struct.unpack("<hhh", pending[2:8]))
        del pending[:8]
    return values


def calculate_setpoint(base_height, factor, modulation):
    setpoint = int(
        (Decimal(base_height) + factor * Decimal(str(modulation))).quantize(
            Decimal("1"), rounding=ROUND_HALF_UP
        )
    )
    if not 0 <= setpoint <= 300:
        raise ValueError(
            "Berechnete Sollhöhe liegt außerhalb des Bereichs 0 bis 300 mm"
        )
    return setpoint


def parse_gain(value, name):
    normalized = value.strip().replace(",", ".")
    if not re.fullmatch(r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)", normalized):
        raise ValueError(f"{name}: Zahl mit maximal zwei Nachkommastellen eingeben")

    try:
        raw_value = Decimal(normalized) * 100
    except InvalidOperation as exc:
        raise ValueError(f"{name}: ungültiger Zahlenwert") from exc

    if (
        not raw_value.is_finite()
        or raw_value != raw_value.to_integral_value()
        or not -32768 <= raw_value <= 32767
    ):
        raise ValueError(
            f"{name} muss zwischen -327,68 und 327,67 liegen "
            "und darf höchstens zwei Nachkommastellen haben"
        )
    return int(raw_value)


def build_command(group, setpoint, kp, ki, kd):
    if not 1 <= group <= 32:
        raise ValueError("Gruppennummer muss zwischen 1 und 32 liegen")
    if not 0 <= setpoint <= 300:
        raise ValueError("Sollwert muss zwischen 0 und 300 mm liegen")
    if any(not -32768 <= gain <= 32767 for gain in (kp, ki, kd)):
        raise ValueError("KP, KI und KD müssen in den int16-Bereich passen")

    payload = b"\xA5\x5A" + struct.pack("<Bhhhh", group, setpoint, kp, ki, kd)
    return payload + struct.pack("<H", binascii.crc_hqx(payload, 0xFFFF))


def main():
    root = tk.Tk()
    root.title("PID-Scope | V 1.0 (c) DHBW Stuttgart - Mechatronik - Prof. Tobias Flämig")
    root.geometry("1000x700")

    port_var = tk.StringVar()
    group_var = tk.StringVar(value="1")
    setpoint_var = tk.StringVar(value="150")
    kp_var = tk.StringVar(value="0.33")
    ki_var = tk.StringVar(value="0.10")
    kd_var = tk.StringVar(value="0.02")
    factor_var = tk.StringVar(value="100")
    repeat_var = tk.BooleanVar(value=False)
    status_var = tk.StringVar(value="Bereit")
    ise_var = tk.StringVar(value="ISE (60 s): 0 mm²·s")
    profile_var = tk.StringVar(value="Kein CSV-Profil geladen")
    stop_event = None
    reader_thread = None
    command_queue = None
    parameter_update_after = None
    profile_update_after = None
    last_queued_parameters = None
    last_parameter_error = None
    modulation_profile = []
    profile_start_time = None
    is_running = False
    closing = False

    controls = ttk.Frame(root, padding=8)
    controls.pack(side=tk.TOP, fill=tk.X)

    connection_row = ttk.Frame(controls)
    connection_row.pack(side=tk.TOP, fill=tk.X, pady=(0, 6))
    ttk.Label(connection_row, text="COM-Schnittstelle:").pack(
        side=tk.LEFT, padx=(0, 4)
    )
    port_menu = ttk.Combobox(
        connection_row,
        textvariable=port_var,
        state="readonly",
        width=12,
    )
    port_menu.pack(side=tk.LEFT, padx=(0, 8))

    ttk.Label(connection_row, text="Gruppe (1–32):").pack(
        side=tk.LEFT, padx=(0, 4)
    )
    group_entry = ttk.Entry(connection_row, textvariable=group_var, width=5)
    group_entry.pack(side=tk.LEFT, padx=(0, 8))

    parameter_row = ttk.Frame(controls)
    parameter_row.pack(side=tk.TOP, fill=tk.X, pady=(0, 6))
    parameter_fields = (
        ("Nullhöhe (0–300 mm)", setpoint_var, 7),
        ("Faktor (mm)", factor_var, 7),
        ("KP (-327,68–327,67)", kp_var, 9),
        ("KI (-327,68–327,67)", ki_var, 9),
        ("KD (-327,68–327,67)", kd_var, 9),
    )
    parameter_entries = []
    for label, variable, width in parameter_fields:
        field = ttk.Frame(parameter_row)
        field.pack(side=tk.LEFT, padx=(0, 10))
        ttk.Label(field, text=label).pack(side=tk.TOP, anchor=tk.W)
        entry = ttk.Entry(field, textvariable=variable, width=width)
        entry.pack(side=tk.TOP)
        parameter_entries.append(entry)

    profile_row = ttk.Frame(controls)
    profile_row.pack(side=tk.TOP, fill=tk.X, pady=(0, 6))
    ttk.Label(
        profile_row,
        text=(
            "Spalten: time_ms, modulation (0–1) | "
            "Sollhöhe = Nullhöhe + Faktor × Modulation"
        ),
    ).pack(side=tk.LEFT, padx=(0, 8))
    ttk.Label(profile_row, textvariable=profile_var).pack(side=tk.LEFT)
    ttk.Checkbutton(
        profile_row,
        text="Repeat",
        variable=repeat_var,
    ).pack(side=tk.LEFT, padx=(12, 0))

    action_row = ttk.Frame(controls)
    action_row.pack(side=tk.TOP, fill=tk.X)

    fig = Figure(figsize=(12, 6))
    ax = fig.add_subplot(111)
    line1, = ax.plot([], [], label="Istwert (mm)")
    line2, = ax.plot([], [], label="Sollwert (mm)")
    line3, = ax.plot([], [], label="Stellwert (%)")

    ax.set_title("PID-Scope")
    ax.set_xlabel("Zeit (s)")
    ax.grid(True)
    ax.legend()
    ax.set_xlim(0, BUFFER_DURATION)

    canvas = FigureCanvasTkAgg(fig, master=root)
    canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

    def autorange(samples):
        minimum = min(min(channel) for channel in samples)
        maximum = max(max(channel) for channel in samples)
        span = maximum - minimum
        padding = max(span * 0.05, 1)
        ax.set_ylim(minimum - padding, maximum + padding)

    def update(_frame):
        with data_lock:
            samples = (tuple(ch1), tuple(ch2), tuple(ch3))
            error_samples = tuple(ise_samples)

        line1.set_data(TIME_BASE, samples[0])
        line2.set_data(TIME_BASE, samples[1])
        line3.set_data(TIME_BASE, samples[2])
        autorange(samples)
        ise_var.set(format_ise(calculate_ise(error_samples, time.monotonic())))

        return line1, line2, line3

    def update_start_button(*_args):
        can_start = not is_running and bool(port_var.get())
        start_button.configure(state=tk.NORMAL if can_start else tk.DISABLED)

    def collect_parameters(elapsed_ms=None):
        if not is_valid_group_number(group_var.get()):
            raise ValueError("Gruppennummer muss zwischen 1 und 32 liegen")
        base_height = parse_setpoint(setpoint_var.get())
        factor = parse_factor(factor_var.get())
        if elapsed_ms is None:
            elapsed_ms = (
                max(0, (time.monotonic() - profile_start_time) * 1000)
                if profile_start_time is not None
                else 0
            )
        modulation = modulation_at_time(
            modulation_profile, elapsed_ms, repeat_var.get()
        )
        effective_setpoint = calculate_setpoint(base_height, factor, modulation)
        return (
            int(group_var.get()),
            effective_setpoint,
            parse_gain(kp_var.get(), "KP"),
            parse_gain(ki_var.get(), "KI"),
            parse_gain(kd_var.get(), "KD"),
        )

    def queue_current_parameters():
        nonlocal last_queued_parameters, last_parameter_error
        if not is_running or command_queue is None:
            return

        try:
            parameters = collect_parameters()
            request = build_command(*parameters)
        except ValueError as exc:
            if str(exc) != last_parameter_error:
                status_var.set(str(exc))
                last_parameter_error = str(exc)
            return

        last_parameter_error = None
        if parameters == last_queued_parameters:
            return

        try:
            command_queue.get_nowait()
        except Empty:
            pass
        command_queue.put_nowait(request)
        last_queued_parameters = parameters
        status_var.set(f"Sende Sollhöhe {parameters[1]} mm und PID-Parameter ...")

    def schedule_parameter_update(*_args):
        nonlocal parameter_update_after
        if not is_running:
            return
        if parameter_update_after is not None:
            root.after_cancel(parameter_update_after)
        parameter_update_after = root.after(300, send_parameter_update)

    def send_parameter_update():
        nonlocal parameter_update_after
        parameter_update_after = None
        queue_current_parameters()

    def update_profile():
        nonlocal profile_update_after
        profile_update_after = None
        if not is_running or profile_start_time is None:
            return
        queue_current_parameters()
        profile_update_after = root.after(50, update_profile)

    def load_profile():
        nonlocal modulation_profile
        selected_file = filedialog.askopenfilename(
            parent=root,
            title="Sollhöhenprofil auswählen",
            filetypes=(("CSV-Dateien", "*.csv"), ("Alle Dateien", "*.*")),
        )
        if not selected_file:
            return
        try:
            loaded_profile = load_modulation_csv(selected_file)
        except (OSError, UnicodeError, csv.Error, ValueError) as exc:
            status_var.set(f"CSV konnte nicht geladen werden: {exc}")
            return

        modulation_profile = loaded_profile
        profile_var.set(f"{Path(selected_file).name} ({len(loaded_profile)} Werte)")
        status_var.set("CSV-Profil geladen; Start beginnt bei Zeit 0")

    def cancel_scheduled_profile_update():
        nonlocal profile_update_after
        if profile_update_after is not None:
            root.after_cancel(profile_update_after)
            profile_update_after = None

    def cancel_scheduled_parameter_update():
        nonlocal parameter_update_after
        if parameter_update_after is not None:
            root.after_cancel(parameter_update_after)
            parameter_update_after = None

    def refresh_ports():
        available_ports = sorted(port.device for port in list_ports.comports())
        port_menu.configure(values=available_ports)

        if port_var.get() not in available_ports:
            preferred_port = "COM8" if "COM8" in available_ports else ""
            port_var.set(preferred_port or (available_ports[0] if available_ports else ""))

        update_start_button()
        if available_ports:
            status_var.set("Bereit")
        else:
            status_var.set("Keine COM-Schnittstelle gefunden")

    def reader_finished(error_message):
        nonlocal is_running, reader_thread, stop_event
        nonlocal command_queue, last_queued_parameters
        nonlocal profile_start_time
        if closing:
            return

        cancel_scheduled_parameter_update()
        cancel_scheduled_profile_update()
        is_running = False
        reader_thread = None
        stop_event = None
        command_queue = None
        last_queued_parameters = None
        profile_start_time = None
        port_menu.configure(state="readonly")
        group_entry.configure(state="normal")
        load_profile_button.configure(state=tk.NORMAL)
        for entry in parameter_entries:
            entry.configure(state="normal")
        refresh_button.configure(state=tk.NORMAL)
        stop_button.configure(state=tk.DISABLED)
        update_start_button()
        status_var.set(error_message or "Gestoppt")

    def serial_reader(
        port_name, request, group_number, outgoing_commands, reader_stop_event
    ):
        error_message = None
        try:
            with serial.Serial(
                port_name,
                BAUDRATE,
                timeout=0.05,
                write_timeout=1,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
            ) as ser:
                ser.write(request)
                ser.flush()
                root.after(
                    0,
                    lambda: status_var.set(
                        f"Verbunden: {port_name} | Parameter für Gruppe "
                        f"{group_number} gesendet"
                    ),
                )

                pending = bytearray()
                while not reader_stop_event.is_set():
                    try:
                        update_request = outgoing_commands.get_nowait()
                    except Empty:
                        pass
                    else:
                        ser.write(update_request)
                        ser.flush()
                        root.after(
                            0,
                            lambda: status_var.set(
                                f"Parametersatz für Gruppe {group_number} aktualisiert"
                            ),
                        )

                    pending.extend(ser.read(max(ser.in_waiting, 1)))
                    frames = extract_telemetry_frames(pending)
                    if frames:
                        received_at = time.monotonic()
                        with data_lock:
                            for v1, v2, v3 in frames:
                                ch1.append(v1)
                                ch2.append(v2)
                                ch3.append(v3)
                                ise_samples.append((received_at, v1, v2))
                            cutoff = received_at - BUFFER_DURATION
                            while (
                                len(ise_samples) > 1
                                and ise_samples[1][0] <= cutoff
                            ):
                                ise_samples.popleft()
        except serial.SerialException as exc:
            error_message = f"Fehler an {port_name}: {exc}"
        finally:
            if not closing:
                root.after(
                    0,
                    lambda message=error_message: reader_finished(message),
                )

    def start_reader():
        nonlocal is_running, reader_thread, stop_event
        nonlocal command_queue, last_queued_parameters
        nonlocal profile_start_time, profile_update_after

        port_name = port_var.get()
        if not port_name:
            status_var.set("Bitte eine COM-Schnittstelle auswählen")
            return
        if is_running:
            return

        try:
            parameters = collect_parameters(elapsed_ms=0)
            request = build_command(*parameters)
        except ValueError as exc:
            status_var.set(str(exc))
            return

        group_number = parameters[0]
        profile_start_time = time.monotonic()
        command_queue = Queue(maxsize=1)
        last_queued_parameters = parameters
        stop_event = threading.Event()
        with data_lock:
            ise_samples.clear()
        is_running = True
        port_menu.configure(state=tk.DISABLED)
        group_entry.configure(state=tk.DISABLED)
        load_profile_button.configure(state=tk.DISABLED)
        refresh_button.configure(state=tk.DISABLED)
        start_button.configure(state=tk.DISABLED)
        stop_button.configure(state=tk.NORMAL)
        status_var.set(f"Verbinde mit {port_name} und sende Parametersatz ...")

        reader_thread = threading.Thread(
            target=serial_reader,
            args=(port_name, request, group_number, command_queue, stop_event),
            daemon=True,
        )
        reader_thread.start()
        if modulation_profile:
            profile_update_after = root.after(50, update_profile)

    def stop_reader():
        nonlocal profile_update_after
        if stop_event is not None:
            stop_event.set()
            cancel_scheduled_parameter_update()
            cancel_scheduled_profile_update()
            stop_button.configure(state=tk.DISABLED)
            status_var.set("Stoppe Datenaufnahme ...")

    def close_window():
        nonlocal closing, profile_update_after
        closing = True
        if stop_event is not None:
            stop_event.set()
        cancel_scheduled_parameter_update()
        cancel_scheduled_profile_update()
        if reader_thread is not None:
            reader_thread.join(timeout=1)
        root.destroy()

    refresh_button = ttk.Button(
        connection_row,
        text="COM aktualisieren",
        command=refresh_ports,
    )
    refresh_button.pack(side=tk.LEFT, padx=(0, 8))

    load_profile_button = ttk.Button(
        profile_row,
        text="CSV-Profil laden",
        command=load_profile,
    )
    load_profile_button.pack(side=tk.LEFT, padx=(0, 8))

    start_button = ttk.Button(
        action_row,
        text="Start",
        command=start_reader,
        state=tk.DISABLED,
    )
    start_button.pack(side=tk.LEFT, padx=(0, 4))

    stop_button = ttk.Button(
        action_row,
        text="Stop",
        command=stop_reader,
        state=tk.DISABLED,
    )
    stop_button.pack(side=tk.LEFT)

    status_label = ttk.Label(action_row, textvariable=status_var)
    status_label.pack(side=tk.LEFT, padx=(12, 0))
    ise_font = tkfont.nametofont("TkDefaultFont").copy()
    ise_font.configure(size=abs(ise_font.cget("size")) * 2)
    ttk.Label(
        action_row,
        textvariable=ise_var,
        font=ise_font,
    ).pack(side=tk.RIGHT, padx=(8, 0))

    port_var.trace_add("write", update_start_button)
    for variable in (
        setpoint_var,
        factor_var,
        kp_var,
        ki_var,
        kd_var,
        repeat_var,
    ):
        variable.trace_add("write", schedule_parameter_update)
    refresh_ports()

    animation = FuncAnimation(
        fig,
        update,
        interval=20,
        blit=False,
        cache_frame_data=False,
    )
    fig.tight_layout()
    root.protocol("WM_DELETE_WINDOW", close_window)
    root.mainloop()


if __name__ == "__main__":
    main()
