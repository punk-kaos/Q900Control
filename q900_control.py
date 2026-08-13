#!/usr/bin/env python3
"""Standalone PyQt6 control console for the Q900 radio.

The CAT and spectrum protocol definitions in this file follow
qpmrpancatweb_1.15.html, the USB CAT reference application.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import IntEnum
import socket
import sys
import threading
import time
from typing import Callable

import numpy as np
import sounddevice as sd
import serial
from serial.tools import list_ports

from PyQt6.QtCore import QObject, QPoint, QRectF, Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QColor, QFont, QImage, QKeySequence, QPainter, QPen, QShortcut
from PyQt6.QtWidgets import (
    QApplication,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QInputDialog,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPushButton,
    QComboBox,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)


SYNC = b"\xa5\xa5\xa5\xa5"
SPECTRUM_PAYLOAD_LENGTH = 516
SPECTRUM_BINS = 512
SPAN_HZ = (48_000, 24_000, 12_000, 6_000, 3_000, 1_500)
# The Q900 FFT view is offset: the CAT-tuned carrier appears 13 kHz to the
# right of the spectrum midpoint, independent of the selected span.
FFT_TUNED_OFFSET_HZ = 13_000


class Command(IntEnum):
    PTT = 0x07
    SET_FREQUENCIES = 0x09
    SET_MODES = 0x0A
    STATUS = 0x0B
    SPEAKER_VOLUME = 0x0D
    RF_GAIN = 0x13
    IF_GAIN = 0x14
    SQUELCH = 0x15
    AGC = 0x16
    PREAMP = 0x17
    NOISE_REDUCTION = 0x19
    NOISE_BLANKER = 0x1A
    ACTIVE_VFO = 0x1B
    SPLIT = 0x1C
    ATU = 0x21
    SPAN = 0x22
    TX_POWER = 0x2C
    CW_SIDETONE = 0x31
    CW_SPEED = 0x35
    SPECTRUM = 0x39


class Mode(IntEnum):
    USB = 0
    LSB = 1
    CWR = 2
    CWL = 3
    AM = 4
    WFM = 5
    NFM = 6
    DIGI = 7
    PKT = 8


def crc16_ccitt(data: bytes) -> int:
    """Return CRC-16/CCITT-FALSE used by CAT and spectrum frames."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_frame(command: int, payload: bytes = b"") -> bytes:
    body = bytes((len(payload) + 3, command)) + payload
    return SYNC + body + crc16_ccitt(body).to_bytes(2, "big")


@dataclass(frozen=True, slots=True)
class CatFrame:
    command: int
    payload: bytes


@dataclass(frozen=True, slots=True)
class SpectrumFrame:
    metadata: bytes
    bins: bytes


class StreamParser:
    """Decode interleaved HTML-reference CAT and fixed-width spectrum frames."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self._spectrum_crc_mode = "auto"
        self._spectrum_crc_seen = 0
        self._spectrum_crc_good = 0

    def feed(self, data: bytes) -> list[CatFrame | SpectrumFrame]:
        self._buffer.extend(data)
        frames: list[CatFrame | SpectrumFrame] = []
        while True:
            start = self._buffer.find(SYNC)
            if start < 0:
                del self._buffer[:-3]
                break
            if start:
                del self._buffer[:start]
            if len(self._buffer) < 5:
                break

            length = self._buffer[4]
            cat_size = 5 + length
            if length >= 3 and len(self._buffer) >= cat_size:
                body = bytes(self._buffer[4 : cat_size - 2])
                received_crc = int.from_bytes(self._buffer[cat_size - 2 : cat_size], "big")
                if crc16_ccitt(body) == received_crc:
                    frames.append(CatFrame(body[1], body[2:]))
                    del self._buffer[:cat_size]
                    continue

            spectrum_size = 4 + SPECTRUM_PAYLOAD_LENGTH
            if len(self._buffer) < spectrum_size:
                break
            raw = bytes(self._buffer[4:spectrum_size])
            expected_crc = int.from_bytes(raw[-2:], "big")
            bins = raw[2:-2]
            plausible = len(bins) == SPECTRUM_BINS and max(bins) - min(bins) > 1 and min(bins) != 255 and max(bins) != 0
            crc_ok = crc16_ccitt(raw[:-2]) == expected_crc
            if self._spectrum_crc_mode == "auto":
                self._spectrum_crc_seen += 1
                if crc_ok:
                    self._spectrum_crc_good += 1
                    self._spectrum_crc_mode = "enforce"
                elif self._spectrum_crc_seen >= 6:
                    # The reference HTML supports Q900 firmware that streams
                    # valid 512-bin frames without a matching CRC tail.
                    self._spectrum_crc_mode = "off"
            if plausible and (self._spectrum_crc_mode != "enforce" or crc_ok):
                frames.append(SpectrumFrame(raw[:2], bins))
                del self._buffer[:spectrum_size]
                continue
            del self._buffer[0]
        return frames


@dataclass(slots=True)
class RadioState:
    listening: bool = False
    connected: bool = False
    transport: str = "TCP"
    vfo_a_hz: int = 440_400_000
    vfo_b_hz: int = 440_500_000
    vfo_a_mode: Mode = Mode.NFM
    vfo_b_mode: Mode = Mode.NFM
    active_vfo_b: bool = False
    ptt: bool = False
    s_meter: int = 0
    swr: int = 0
    span_index: int = 2
    utc: tuple[int, int, int] = (0, 0, 0)
    status_flags: int = 0
    rf_gain: int = 48
    if_gain: int = 50
    squelch: int = 0
    agc: int = 3
    preamp: int = 0
    speaker_volume: int = 0
    noise_reduction: int = 1
    noise_blanker: int = 0
    split: bool = False
    atu: int = 0
    tx_power_high: bool = False
    cw_sidetone_hz: int = 600
    cw_speed: int = 26


class RadioSignals(QObject):
    state_changed = pyqtSignal(object)
    spectrum_received = pyqtSignal(bytes)
    connection_error = pyqtSignal(str)
    audio_state_changed = pyqtSignal(str)


class UsbAudioMonitor:
    """Route Q900 USB receive audio to a local speaker device only.

    The Q900 exposes separate Core Audio input and output devices. This class
    intentionally never opens the Q900 output device, which is the radio's
    transmit-audio path.
    """

    def __init__(self, signals: RadioSignals) -> None:
        self.signals = signals
        self._input_stream: sd.InputStream | None = None
        self._output_stream: sd.OutputStream | None = None
        self._audio_queue: deque[np.ndarray] = deque()
        self._queue_lock = threading.Lock()
        self._queued_frames = 0

    @staticmethod
    def input_devices() -> list[tuple[int, str]]:
        return [
            (index, device["name"])
            for index, device in enumerate(sd.query_devices())
            if device["max_input_channels"] > 0 and "q900" in device["name"].lower()
        ]

    @staticmethod
    def output_devices() -> list[tuple[int, str]]:
        return [
            (index, device["name"])
            for index, device in enumerate(sd.query_devices())
            if device["max_output_channels"] > 0 and "q900" not in device["name"].lower()
        ]

    def start(self, input_device: int, output_device: int) -> None:
        self.stop()
        input_info = sd.query_devices(input_device, "input")
        output_info = sd.query_devices(output_device, "output")
        sample_rate = int(min(input_info["default_samplerate"], output_info["default_samplerate"]))
        input_channels = min(2, input_info["max_input_channels"])
        output_channels = min(2, output_info["max_output_channels"])
        blocksize = 960
        max_queued_frames = sample_rate * 2

        def input_callback(indata, frames, timing, status):  # type: ignore[no-untyped-def]
            # Only channel 1 is receive audio. Channel 2 may carry auxiliary data.
            mono = indata[:, 0].copy()
            with self._queue_lock:
                while self._audio_queue and self._queued_frames + frames > max_queued_frames:
                    self._queued_frames -= len(self._audio_queue.popleft())
                self._audio_queue.append(mono)
                self._queued_frames += frames

        def output_callback(outdata, frames, timing, status):  # type: ignore[no-untyped-def]
            outdata.fill(0)
            offset = 0
            with self._queue_lock:
                while offset < frames and self._audio_queue:
                    block = self._audio_queue[0]
                    count = min(frames - offset, len(block))
                    outdata[offset : offset + count, :] = block[:count, np.newaxis]
                    offset += count
                    if count == len(block):
                        self._audio_queue.popleft()
                    else:
                        self._audio_queue[0] = block[count:]
                    self._queued_frames -= count

        self._input_stream = sd.InputStream(
            device=input_device,
            samplerate=sample_rate,
            blocksize=blocksize,
            channels=input_channels,
            dtype="float32",
            latency="high",
            callback=input_callback,
        )
        self._output_stream = sd.OutputStream(
            device=output_device,
            samplerate=sample_rate,
            blocksize=blocksize,
            channels=output_channels,
            dtype="float32",
            latency="high",
            callback=output_callback,
        )
        self._output_stream.start()
        self._input_stream.start()
        self.signals.audio_state_changed.emit(
            f"USB RX audio: {input_info['name']} channel 1 -> {output_info['name']} ({sample_rate} Hz)"
        )

    def stop(self) -> None:
        for stream in (self._input_stream, self._output_stream):
            if stream:
                stream.stop()
                stream.close()
        self._input_stream = None
        self._output_stream = None
        with self._queue_lock:
            self._audio_queue.clear()
            self._queued_frames = 0

    @property
    def running(self) -> bool:
        return self._input_stream is not None and self._output_stream is not None


class RadioClient:
    """TCP/8081 control listener using source-backed CAT commands only."""

    def __init__(self, signals: RadioSignals) -> None:
        self.state = RadioState()
        self.signals = signals
        self._listener: socket.socket | None = None
        self._socket: socket.socket | serial.Serial | None = None
        self._stop = threading.Event()
        self._listen_thread: threading.Thread | None = None
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()

    def start_listener(self, host: str = "0.0.0.0", port: int = 8081) -> None:
        self.disconnect()
        try:
            listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.bind((host, port))
            listener.listen(1)
            listener.settimeout(0.5)
            self._listener = listener
            self._stop.clear()
            self.state.listening = True
            self._emit_state()
            self._listen_thread = threading.Thread(target=self._accept_loop, name="q900-listener", daemon=True)
            self._listen_thread.start()
        except OSError as error:
            if self._listener:
                self._listener.close()
            self._listener = None
            self.state.listening = False
            self.signals.connection_error.emit(str(error))

    def disconnect(self) -> None:
        self._stop.set()
        listener, self._listener = self._listener, None
        if listener:
            listener.close()
        sock, self._socket = self._socket, None
        if sock:
            try:
                if isinstance(sock, socket.socket):
                    self._write(sock, set_ptt(False))
                else:
                    # Keep hardware PTT lines inactive before closing USB.
                    sock.dtr = False
                    sock.rts = False
            except (OSError, serial.SerialException):
                pass
            sock.close()
        if self.state.connected or self.state.listening:
            self.state.listening = False
            self.state.connected = False
            self.state.ptt = False
            self._emit_state()

    def send(self, data: bytes) -> None:
        with self._lock:
            if not self._socket:
                raise ConnectionError("Radio is not connected")
            self._write(self._socket, data)

    @staticmethod
    def _write(transport: socket.socket | serial.Serial, data: bytes) -> None:
        if isinstance(transport, socket.socket):
            transport.sendall(data)
        else:
            transport.write(data)

    def connect_usb(self, port: str, baudrate: int = 115200) -> None:
        self.disconnect()
        try:
            # Do not open pyserial with its default modem-control state: some
            # Q900 USB adapters wire DTR/RTS to PTT and can key on open.
            device = serial.Serial(port=None, baudrate=baudrate, timeout=0.1, write_timeout=1)
            device.rtscts = False
            device.dsrdtr = False
            device.dtr = False
            device.rts = False
            device.port = port
            device.open()
            device.dtr = False
            device.rts = False
            self._socket = device
            self._stop.clear()
            self.state.transport = "USB"
            # Do not send a CAT PTT frame over USB until its asserted value is
            # verified on hardware. Low DTR/RTS is the only connection action.
            self.state.connected = True
            self._emit_state()
            self._thread = threading.Thread(target=self._receive_loop, name="q900-usb", daemon=True)
            self._thread.start()
        except (OSError, serial.SerialException) as error:
            self._socket = None
            self.signals.connection_error.emit(str(error))

    def _accept_loop(self) -> None:
        while not self._stop.is_set() and self._listener:
            try:
                sock, _address = self._listener.accept()
            except TimeoutError:
                continue
            except OSError:
                break
            with self._lock:
                previous, self._socket = self._socket, sock
            if previous:
                previous.close()
            sock.settimeout(0.1)
            try:
                # The reference application releases PTT repeatedly on every connection.
                for _ in range(5):
                    self.send(set_ptt(False))
                    time.sleep(0.06)
            except OSError:
                sock.close()
                continue
            self.state.connected = True
            self.state.transport = "TCP"
            self._emit_state()
            self._thread = threading.Thread(target=self._receive_loop, name="q900-tcp", daemon=True)
            self._thread.start()

    def set_value(self, field: str, command: Command, value: int) -> None:
        setattr(self.state, field, value)
        self.send(encode_frame(command, bytes((value & 0xFF,))))
        self._emit_state()

    def tune(self, frequency_hz: int) -> None:
        frequency_hz = max(100_000, min(2_000_000_000, frequency_hz))
        if self.state.active_vfo_b:
            self.state.vfo_b_hz = frequency_hz
        else:
            self.state.vfo_a_hz = frequency_hz
        payload = self.state.vfo_a_hz.to_bytes(4, "big") + self.state.vfo_b_hz.to_bytes(4, "big")
        self.send(encode_frame(Command.SET_FREQUENCIES, payload))
        self._emit_state()

    def set_mode(self, mode: Mode) -> None:
        if self.state.active_vfo_b:
            self.state.vfo_b_mode = mode
        else:
            self.state.vfo_a_mode = mode
        self.send(encode_frame(Command.SET_MODES, bytes((self.state.vfo_a_mode, self.state.vfo_b_mode))))
        self._emit_state()

    def select_vfo(self, vfo_b: bool) -> None:
        self.state.active_vfo_b = vfo_b
        self.send(encode_frame(Command.ACTIVE_VFO, bytes((int(vfo_b),))))
        self._emit_state()

    def set_span(self, index: int) -> None:
        self.state.span_index = max(0, min(5, index))
        self.send(encode_frame(Command.SPAN, bytes((5 - self.state.span_index,))))
        self._emit_state()

    def set_split(self, enabled: bool) -> None:
        self.state.split = enabled
        self.send(encode_frame(Command.SPLIT, bytes((int(enabled),))))
        self._emit_state()

    def set_tx_power(self, high: bool) -> None:
        self.state.tx_power_high = high
        self.send(encode_frame(Command.TX_POWER, bytes((int(high),))))
        self._emit_state()

    def set_atu(self, value: int) -> None:
        self.state.atu = value
        self.send(encode_frame(Command.ATU, bytes((value,))))
        self._emit_state()

    def _receive_loop(self) -> None:
        parser = StreamParser()
        last_status = 0.0
        last_spectrum = 0.0
        spectrum_pending_at = 0.0
        try:
            while not self._stop.is_set() and self._socket:
                now = time.monotonic()
                try:
                    if now - last_status >= 0.49:
                        self.send(encode_frame(Command.STATUS))
                        last_status = now
                    if not spectrum_pending_at or now - spectrum_pending_at >= 0.15:
                        if now - last_spectrum >= 0.12:
                            self.send(encode_frame(Command.SPECTRUM))
                            last_spectrum = now
                            spectrum_pending_at = now
                    transport = self._socket
                    if isinstance(transport, socket.socket):
                        data = transport.recv(65536)
                    else:
                        data = transport.read(65536)
                except TimeoutError:
                    continue
                if not data and isinstance(transport, serial.Serial):
                    continue
                if not data:
                    break
                for frame in parser.feed(data):
                    if isinstance(frame, SpectrumFrame):
                        spectrum_pending_at = 0.0
                        self.signals.spectrum_received.emit(frame.bins)
                    elif frame.command == Command.STATUS:
                        self._handle_status(frame.payload)
        except (OSError, serial.SerialException) as error:
            if not self._stop.is_set():
                self.signals.connection_error.emit(str(error))
        finally:
            self.state.connected = False
            self.state.ptt = False
            self._emit_state()

    def _handle_status(self, data: bytes) -> None:
        if len(data) < 24:
            return
        self.state.ptt = data[0] == 1
        if data[1] in Mode._value2member_map_:
            self.state.vfo_a_mode = Mode(data[1])
        if data[2] in Mode._value2member_map_:
            self.state.vfo_b_mode = Mode(data[2])
        self.state.vfo_a_hz = int.from_bytes(data[3:7], "big")
        self.state.vfo_b_hz = int.from_bytes(data[7:11], "big")
        self.state.active_vfo_b = data[11] == 1
        self.state.span_index = data[16] if data[16] < len(SPAN_HZ) else 2
        self.state.utc = tuple(data[18:21])
        self.state.status_flags = data[21]
        self.state.s_meter = data[22]
        self.state.swr = data[23] & 0x3F
        self._emit_state()

    def _emit_state(self) -> None:
        self.signals.state_changed.emit(self.state)


STYLESHEET = """
QMainWindow, QWidget { background: #030712; color: #dbe5ed; }
QFrame#panel { background: #07111f; border: 1px solid #16445a; border-radius: 12px; }
QFrame#meter { background: #24262b; border-radius: 7px; }
QPushButton#tile { background: #0d1724; border: 1px solid #1f6b82; border-radius: 7px; min-width: 105px; min-height: 58px; }
QPushButton#tile:hover { background: #122334; border-color: #42c7d7; }
QPushButton#tile:disabled { border-color: #183142; color: #687780; }
QLabel#tileTitle { color: #dce4ed; font: 700 16px "Menlo"; }
QLabel#tileValue { color: #50d9e8; font: 14px "Menlo"; }
QLineEdit#frequency { background: #061322; border: none; color: #65eaf2; font: 700 52px "Menlo"; letter-spacing: 7px; padding: 8px; }
QLabel#meterLabel { color: #aab0bc; font-size: 13px; }
QFrame#bar { background: #111317; border-radius: 4px; min-height: 10px; }
QFrame#fill { background: #71db8d; border-radius: 4px; }
"""


class Meter(QWidget):
    def __init__(self, title: str, color: str = "#83e99a") -> None:
        super().__init__()
        self._color = color
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 8, 10, 8)
        row = QHBoxLayout()
        label = QLabel(title)
        label.setObjectName("meterLabel")
        self.value = QLabel("0")
        self.value.setStyleSheet(f"color: {color}; font: 700 16px Menlo")
        row.addWidget(label)
        row.addStretch()
        row.addWidget(self.value)
        layout.addLayout(row)
        bar = QFrame()
        bar.setObjectName("bar")
        inner = QHBoxLayout(bar)
        inner.setContentsMargins(0, 0, 0, 0)
        self.fill = QFrame()
        self.fill.setObjectName("fill")
        self.fill.setStyleSheet(f"background: {color}; border-radius: 4px;")
        self.fill.setFixedWidth(1)
        inner.addWidget(self.fill)
        inner.addStretch()
        layout.addWidget(bar)

    def set_value(self, value: int) -> None:
        value = max(0, min(34, value))
        self.value.setText(str(value))
        # QFrame has no useful horizontal size hint; use a fixed fill width so
        # the adjacent stretch cannot collapse the meter to zero pixels.
        self.fill.setFixedWidth(max(1, value * 9))


class ControlTile(QPushButton):
    def __init__(self, title: str, value: str) -> None:
        super().__init__()
        self.setObjectName("tile")
        self.title = title
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 6, 8, 6)
        title_label = QLabel(title)
        title_label.setObjectName("tileTitle")
        title_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.value = QLabel(value)
        self.value.setObjectName("tileValue")
        self.value.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(title_label)
        layout.addWidget(self.value)

    def set_value(self, value: str) -> None:
        self.value.setText(value)


class SpectrumWaterfall(QWidget):
    """Canvas-like spectrum and waterfall based on the HTML reference behavior."""

    tune_requested = pyqtSignal(int)

    def __init__(self) -> None:
        super().__init__()
        self.setMinimumHeight(360)
        self.setMouseTracking(True)
        self._bins = bytes(SPECTRUM_BINS)
        self._rows: list[bytes] = []
        self._display_center_hz = 440_400_000
        self._tuned_hz = 440_400_000
        self._mode = Mode.NFM
        self._span_hz = SPAN_HZ[2]
        self._drag_start: QPoint | None = None
        self._drag_center = 0
        self._last_drag_send = 0.0
        self._dragged = False

    def set_state(self, state: RadioState) -> None:
        tuned_hz = state.vfo_b_hz if state.active_vfo_b else state.vfo_a_hz
        self._tuned_hz = tuned_hz
        self._display_center_hz = self._tuned_hz
        self._mode = state.vfo_b_mode if state.active_vfo_b else state.vfo_a_mode
        self._span_hz = SPAN_HZ[state.span_index]
        self.update()

    def add_bins(self, bins: bytes) -> None:
        self._bins = bins
        self._rows.insert(0, bins)
        self._rows = self._rows[:140]
        self.update()

    def paintEvent(self, event) -> None:  # type: ignore[no-untyped-def]
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor("#080b10"))
        width, height = self.width(), self.height()
        spectrum_height = int(height * 0.43)
        self._draw_spectrum(painter, width, spectrum_height)
        self._draw_waterfall(painter, width, spectrum_height, height - spectrum_height)
        self._draw_tuned_cursor(painter, width, height)

    def _draw_tuned_cursor(self, painter: QPainter, width: int, height: int) -> None:
        """Render the active VFO and its mode-specific receive passband."""
        x = round(self._frequency_to_x(self._tuned_hz + FFT_TUNED_OFFSET_HZ, width))
        bands = self._passband_ranges()
        painter.setPen(Qt.PenStyle.NoPen)
        for low_hz, high_hz in bands:
            left = self._frequency_to_x(self._tuned_hz + FFT_TUNED_OFFSET_HZ + low_hz, width)
            right = self._frequency_to_x(self._tuned_hz + FFT_TUNED_OFFSET_HZ + high_hz, width)
            painter.setBrush(QColor(54, 203, 221, 62))
            painter.drawRect(QRectF(min(left, right), 0, max(2, abs(right - left)), height))
            painter.setBrush(QColor(88, 230, 241, 130))
            painter.drawRect(QRectF(min(left, right), 0, max(1, abs(right - left)), 3))
        painter.setPen(QPen(QColor("#45e4ef"), 2))
        painter.drawLine(x, 0, x, height)
        painter.setBrush(QColor("#45e4ef"))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawPolygon(QPoint(x - 7, 0), QPoint(x + 7, 0), QPoint(x, 10))
        painter.setPen(QColor("#06111b"))
        label_x = min(max(8, int(x + 10)), max(8, width - 210))
        painter.drawText(label_x, 18, f"{self._mode.name}  {self._tuned_hz / 1_000_000:.3f} MHz")

    def _frequency_to_x(self, frequency_hz: int, width: int) -> float:
        return width / 2 + (frequency_hz - self._display_center_hz) * width / self._span_hz

    def _passband_ranges(self) -> tuple[tuple[int, int], ...]:
        """Return receive bandwidth offsets relative to the dial frequency."""
        if self._mode == Mode.USB:
            return ((300, 2_800),)
        if self._mode == Mode.LSB:
            return ((-2_800, -300),)
        if self._mode in (Mode.CWL, Mode.CWR):
            return ((-250, 250),)
        if self._mode == Mode.AM:
            return ((-4_000, -150), (150, 4_000))
        if self._mode == Mode.NFM:
            # Typical narrow FM receive bandwidth is about 5 kHz total.
            # Leave a small center gap so the tuned carrier remains visible.
            return ((-2_500, -150), (150, 2_500))
        if self._mode == Mode.WFM:
            return ((-50_000, 50_000),)
        return ((-1_500, 1_500),)

    def _draw_spectrum(self, painter: QPainter, width: int, height: int) -> None:
        painter.fillRect(0, 0, width, height, QColor("#131617"))
        grid = QPen(QColor("#293238"), 1)
        painter.setPen(grid)
        for y in range(0, height, max(1, height // 5)):
            painter.drawLine(0, y, width, y)
        for x in range(0, width, max(1, width // 8)):
            painter.drawLine(x, 0, x, height)
        if len(self._bins) < 2:
            return
        points = []
        for x in range(width):
            index = int(x * (len(self._bins) - 1) / max(1, width - 1))
            value = max(0, min(255, self._bins[index]))
            y = height - int(value / 255 * (height - 12))
            points.append(QPoint(x, y))
        painter.setPen(QPen(QColor("#76e0ee"), 1.3))
        for first, second in zip(points, points[1:]):
            painter.drawLine(first, second)
        painter.setPen(QColor("#9aaab5"))
        painter.drawText(8, 18, f"{self._display_center_hz - self._span_hz // 2:,} Hz")
        painter.drawText(max(8, width - 180), 18, f"{self._display_center_hz + self._span_hz // 2:,} Hz")

    def _draw_waterfall(self, painter: QPainter, width: int, top: int, height: int) -> None:
        painter.fillRect(0, top, width, height, QColor("#02050c"))
        if not self._rows:
            painter.setPen(QColor("#63727d"))
            painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, "Waiting for spectrum frames")
            return
        row_height = max(1, height // min(len(self._rows), 100))
        image = QImage(width, row_height, QImage.Format.Format_RGB32)
        for row_number, bins in enumerate(self._rows[:height // row_height]):
            minimum, maximum = min(bins), max(bins)
            spread = max(1, maximum - minimum)
            for x in range(width):
                index = int(x * (len(bins) - 1) / max(1, width - 1))
                intensity = (bins[index] - minimum) * 255 // spread
                color = QColor(intensity, 80 + intensity * 175 // 255, 40 + (255 - intensity) * 150 // 255)
                for y in range(row_height):
                    image.setPixelColor(x, y, color)
            painter.drawImage(0, top + row_number * row_height, image)

    def mousePressEvent(self, event) -> None:  # type: ignore[no-untyped-def]
        if event.button() == Qt.MouseButton.LeftButton:
            self._drag_start = event.position().toPoint()
            self._drag_center = self._display_center_hz
            self._dragged = False

    def mouseMoveEvent(self, event) -> None:  # type: ignore[no-untyped-def]
        if not self._drag_start:
            return
        dx = event.position().x() - self._drag_start.x()
        self._dragged = self._dragged or abs(dx) >= 4
        frequency = self._drag_center - int(dx * self._span_hz / max(1, self.width()))
        self._display_center_hz = frequency
        self._tuned_hz = frequency
        self.update()
        if time.monotonic() - self._last_drag_send >= 0.12:
            self.tune_requested.emit(frequency)
            self._last_drag_send = time.monotonic()

    def mouseReleaseEvent(self, event) -> None:  # type: ignore[no-untyped-def]
        if self._drag_start and event.button() == Qt.MouseButton.LeftButton:
            if self._dragged:
                self.tune_requested.emit(self._tuned_hz)
            else:
                # The spectrum and waterfall share the same frequency axis.
                # Map a click directly to its RF position in the current span.
                clicked_hz = self._drag_center + int(
                    (event.position().x() - self.width() / 2) * self._span_hz / max(1, self.width())
                )
                self._display_center_hz = clicked_hz
                self._tuned_hz = clicked_hz
                self.tune_requested.emit(clicked_hz)
                self.update()
            self._drag_start = None


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Q900 Control")
        self.resize(1600, 920)
        self.signals = RadioSignals()
        self.client = RadioClient(self.signals)
        self.audio = UsbAudioMonitor(self.signals)
        self.tiles: dict[str, ControlTile] = {}
        self.signals.state_changed.connect(self.update_state)
        self.signals.connection_error.connect(self.show_error)
        self.signals.audio_state_changed.connect(self.show_audio_state)
        left_tune = QShortcut(QKeySequence(Qt.Key.Key_Left), self)
        left_tune.setContext(Qt.ShortcutContext.ApplicationShortcut)
        left_tune.activated.connect(lambda: self.keyboard_tune(-1))
        right_tune = QShortcut(QKeySequence(Qt.Key.Key_Right), self)
        right_tune.setContext(Qt.ShortcutContext.ApplicationShortcut)
        right_tune.activated.connect(lambda: self.keyboard_tune(1))
        self._tune_shortcuts = (left_tune, right_tune)

        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)
        layout.addLayout(self._top_panel())
        layout.addWidget(self._control_bank())
        self.spectrum = SpectrumWaterfall()
        self.signals.spectrum_received.connect(self.spectrum.add_bins)
        self.spectrum.tune_requested.connect(self.tune)
        layout.addWidget(self.spectrum, 1)
        layout.addLayout(self._audio_panel())
        self.status = QLabel("Listener stopped. Start TCP listening or connect over USB.")
        self.status.setStyleSheet("color: #9aaab5; padding: 4px 10px")
        layout.addWidget(self.status)

    def _top_panel(self) -> QHBoxLayout:
        top = QHBoxLayout()
        panel = QFrame()
        panel.setObjectName("panel")
        frequency_layout = QVBoxLayout(panel)
        header = QHBoxLayout()
        self.transport = QComboBox()
        self.transport.addItems(("TCP Listener", "USB Serial"))
        self.transport.currentIndexChanged.connect(self.update_transport_ui)
        self.host = QLineEdit("0.0.0.0")
        self.host.setPlaceholderText("Listen address")
        self.host.setMaximumWidth(220)
        self.serial_port = QComboBox()
        self.serial_port.setMinimumWidth(220)
        self.serial_port.setVisible(False)
        self.connect_button = QPushButton("Start Listener")
        self.connect_button.clicked.connect(self.toggle_connection)
        self.refresh_ports = QPushButton("Refresh")
        self.refresh_ports.clicked.connect(self.populate_serial_ports)
        self.refresh_ports.setVisible(False)
        header.addWidget(self.host)
        header.addWidget(self.serial_port)
        header.addWidget(self.transport)
        header.addWidget(self.refresh_ports)
        header.addWidget(self.connect_button)
        header.addStretch()
        self.mode_selector = QComboBox()
        for mode in Mode:
            self.mode_selector.addItem(mode.name, mode)
        self.mode_selector.currentIndexChanged.connect(self.select_mode)
        self.mode_selector.setToolTip("Operating mode for the active VFO")
        header.addWidget(self.mode_selector)
        self.vfo_badge = QLabel("A")
        self.vfo_badge.setStyleSheet("background: #477fd5; border-radius: 15px; padding: 8px; font-weight: bold")
        header.addWidget(self.vfo_badge)
        frequency_layout.addLayout(header)
        self.frequency = QLineEdit("440.400")
        self.frequency.setObjectName("frequency")
        self.frequency.setPlaceholderText("Frequency in MHz")
        self.frequency.setToolTip("Enter a frequency in MHz, for example 440.400 or 14.074")
        self.frequency.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.frequency.returnPressed.connect(self.submit_frequency)
        self.frequency.installEventFilter(self)
        frequency_layout.addWidget(self.frequency)
        top.addWidget(panel, 3)

        meters = QFrame()
        meters.setObjectName("meter")
        meter_layout = QVBoxLayout(meters)
        self.s_meter = Meter("S Meter")
        self.swr_meter = Meter("SWR / TX Meter", "#eeae63")
        meter_layout.addWidget(self.s_meter)
        meter_layout.addWidget(self.swr_meter)
        top.addWidget(meters, 2)
        return top

    def _control_bank(self) -> QScrollArea:
        controls = [
            ("POWER", "Power", None), ("RFG", "48", ("rf_gain", Command.RF_GAIN, 0, 100)),
            ("IFG", "50", ("if_gain", Command.IF_GAIN, 0, 80)), ("SQL", "0", ("squelch", Command.SQUELCH, 0, 20)),
            ("AGC", "Slow", "agc"), ("AMP", "Off", "preamp"), ("SVOL", "0", ("speaker_volume", Command.SPEAKER_VOLUME, 0, 30)),
            ("HVOL", "0", None), ("MIC", "6", None), ("CMP", "9", None), ("BAS", "20", None),
            ("TRB", "20", None), ("SPLIT", "Off", "split"), ("A/B", "Frequency A", "vfo"),
            ("NB", "Off", ("noise_blanker", Command.NOISE_BLANKER, 0, 5)), ("NR", "On", ("noise_reduction", Command.NOISE_REDUCTION, 0, 5)),
            ("NBL", "7", None), ("PEAK", "15", None), ("ATU", "Off", "atu"), ("SPAN", "12 kHz", "span"),
            ("REF", "17", None), ("PWR", "Low", "tx_power"), ("TONE", "600 Hz", "tone"),
            ("SPEED", "26", ("cw_speed", Command.CW_SPEED, 5, 48)), ("DISP", "Display", None),
            ("RIT", "0", None), ("XIT", "0", None), ("LTIME", "100", None),
        ]
        content = QWidget()
        grid = QGridLayout(content)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(8)
        for index, (title, value, action) in enumerate(controls):
            tile = ControlTile(title, value)
            self.tiles[title] = tile
            if action is not None:
                tile.clicked.connect(lambda checked=False, action=action: self.activate_control(action))
            grid.addWidget(tile, index // 14, index % 14)
        area = QScrollArea()
        area.setWidgetResizable(True)
        area.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        area.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        area.setWidget(content)
        area.setFixedHeight(205)
        return area

    def _audio_panel(self) -> QHBoxLayout:
        audio = QHBoxLayout()
        audio.addWidget(QLabel("USB RX Audio"))
        self.audio_input = QComboBox()
        self.audio_output = QComboBox()
        self.refresh_audio_devices()
        refresh = QPushButton("Refresh Audio")
        refresh.clicked.connect(self.refresh_audio_devices)
        self.audio_button = QPushButton("Start Audio")
        self.audio_button.clicked.connect(self.toggle_audio)
        audio.addWidget(self.audio_input)
        audio.addWidget(QLabel("to"))
        audio.addWidget(self.audio_output)
        audio.addWidget(refresh)
        audio.addWidget(self.audio_button)
        audio.addStretch()
        return audio

    def refresh_audio_devices(self) -> None:
        current_input = self.audio_input.currentData() if hasattr(self, "audio_input") else None
        current_output = self.audio_output.currentData() if hasattr(self, "audio_output") else None
        if not hasattr(self, "audio_input"):
            return
        self.audio_input.clear()
        self.audio_output.clear()
        for index, name in UsbAudioMonitor.input_devices():
            self.audio_input.addItem(name, index)
        for index, name in UsbAudioMonitor.output_devices():
            self.audio_output.addItem(name, index)
        if self.audio_input.count() == 0:
            self.audio_input.addItem("No Q900 USB input found", None)
        if self.audio_output.count() == 0:
            self.audio_output.addItem("No local speaker output found", None)
        if current_input is not None:
            self.audio_input.setCurrentIndex(max(0, self.audio_input.findData(current_input)))
        if current_output is not None:
            self.audio_output.setCurrentIndex(max(0, self.audio_output.findData(current_output)))

    def toggle_audio(self) -> None:
        if self.audio.running:
            self.audio.stop()
            self.audio_button.setText("Start Audio")
            self.status.setText("USB audio monitor stopped.")
            return
        input_device = self.audio_input.currentData()
        output_device = self.audio_output.currentData()
        if input_device is None or output_device is None:
            self.status.setText("Select a Q900 USB input and a local speaker output.")
            return
        try:
            self.audio.start(input_device, output_device)
            self.audio_button.setText("Stop Audio")
        except sd.PortAudioError as error:
            self.show_error(f"Audio: {error}")

    def start_audio_default(self) -> None:
        """Start receive monitoring when the Q900 and a local speaker exist."""
        if self.audio_input.currentData() is None or self.audio_output.currentData() is None:
            return
        try:
            self.audio.start(self.audio_input.currentData(), self.audio_output.currentData())
            self.audio_button.setText("Stop Audio")
        except sd.PortAudioError as error:
            self.status.setText(f"USB audio not started: {error}")

    def show_audio_state(self, message: str) -> None:
        self.status.setText(message)

    def toggle_connection(self) -> None:
        if self.client.state.connected or self.client.state.listening:
            self.client.disconnect()
        elif self.transport.currentIndex() == 1:
            port = self.serial_port.currentData()
            if not port:
                self.status.setText("No USB serial device is available. Connect the radio and refresh the list.")
                return
            self.status.setText(f"Opening USB serial port {port} at 115200 baud...")
            threading.Thread(target=self.client.connect_usb, args=(port,), daemon=True).start()
        else:
            self.status.setText("Starting TCP/8081 listener. Waiting for radio connection...")
            self.client.start_listener(self.host.text().strip() or "0.0.0.0")

    def update_transport_ui(self) -> None:
        usb = self.transport.currentIndex() == 1
        self.refresh_ports.setVisible(usb)
        self.host.setVisible(not usb)
        self.serial_port.setVisible(usb)
        if usb:
            self.populate_serial_ports()
        self.connect_button.setText("Connect USB" if usb else "Start Listener")

    def populate_serial_ports(self) -> None:
        selected = self.serial_port.currentData()
        ports = list(list_ports.comports())
        self.serial_port.clear()
        for port in ports:
            description = port.description if port.description and port.description != "n/a" else "Serial device"
            self.serial_port.addItem(f"{port.device} - {description}", port.device)
        if not ports:
            self.serial_port.addItem("No USB serial devices found", None)
        elif selected:
            index = self.serial_port.findData(selected)
            if index >= 0:
                self.serial_port.setCurrentIndex(index)
        self.status.setText("USB device list refreshed." if ports else "No USB serial devices found.")

    def submit_frequency(self) -> None:
        try:
            frequency_hz = round(float(self.frequency.text().replace(",", "").strip()) * 1_000_000)
            self.frequency.clearFocus()
            self.tune(frequency_hz)
        except ValueError:
            self.status.setText("Frequency must be a number in MHz, for example 440.400 or 14.074.")

    def select_mode(self) -> None:
        if not self.client.state.connected:
            return
        mode = self.mode_selector.currentData()
        if not isinstance(mode, Mode):
            return
        try:
            self.client.set_mode(mode)
        except (ConnectionError, OSError) as error:
            self.show_error(str(error))

    def tune(self, frequency: int) -> None:
        try:
            self.client.tune(frequency)
        except ConnectionError:
            self.status.setText("Wait for the radio to connect before tuning.")

    def keyboard_tune(self, direction: int) -> None:
        """Step the active VFO by 0.01 kHz (10 Hz) from a rounded boundary."""
        current_hz = self.client.state.vfo_b_hz if self.client.state.active_vfo_b else self.client.state.vfo_a_hz
        step_hz = 10
        rounded_hz = round(current_hz / step_hz) * step_hz
        self.tune(rounded_hz + direction * step_hz)

    def eventFilter(self, watched: QObject, event) -> bool:  # type: ignore[no-untyped-def]
        if watched is self.frequency and event.type() == event.Type.KeyPress:
            if event.key() == Qt.Key.Key_Left:
                self.keyboard_tune(-1)
                return True
            if event.key() == Qt.Key.Key_Right:
                self.keyboard_tune(1)
                return True
        return super().eventFilter(watched, event)

    def keyPressEvent(self, event) -> None:  # type: ignore[no-untyped-def]
        if event.key() == Qt.Key.Key_Left:
            self.keyboard_tune(-1)
            return
        if event.key() == Qt.Key.Key_Right:
            self.keyboard_tune(1)
            return
        super().keyPressEvent(event)

    def activate_control(self, action: object) -> None:
        if not self.client.state.connected:
            self.status.setText("Wait for the radio to connect before changing controls.")
            return
        try:
            if isinstance(action, tuple):
                field, command, minimum, maximum = action
                current = getattr(self.client.state, field)
                value, accepted = QInputDialog.getInt(self, field.replace("_", " ").title(), "Value", current, minimum, maximum)
                if accepted:
                    self.client.set_value(field, command, value)
            elif action == "agc":
                labels = ("Off", "Fast", "Mid", "Slow", "SSlow", "Auto")
                self.client.set_value("agc", Command.AGC, (self.client.state.agc + 1) % len(labels))
            elif action == "preamp":
                self.client.set_value("preamp", Command.PREAMP, 1 - self.client.state.preamp)
            elif action == "split":
                self.client.set_split(not self.client.state.split)
            elif action == "vfo":
                self.client.select_vfo(not self.client.state.active_vfo_b)
            elif action == "span":
                self.client.set_span((self.client.state.span_index + 1) % len(SPAN_HZ))
            elif action == "atu":
                self.client.set_atu((self.client.state.atu + 1) % 3)
            elif action == "tx_power":
                self.client.set_tx_power(not self.client.state.tx_power_high)
            elif action == "tone":
                value, accepted = QInputDialog.getInt(self, "CW Sidetone", "Frequency (Hz)", self.client.state.cw_sidetone_hz, 400, 800, 10)
                if accepted:
                    self.client.state.cw_sidetone_hz = value
                    self.client.send(encode_frame(Command.CW_SIDETONE, bytes((round(value / 10),))))
                    self.client._emit_state()
        except (ConnectionError, OSError) as error:
            self.show_error(str(error))

    def update_state(self, state: RadioState) -> None:
        frequency = state.vfo_b_hz if state.active_vfo_b else state.vfo_a_hz
        # Status arrives every 490 ms. Never replace an operator's live edit.
        if not self.frequency.hasFocus():
            self.frequency.setText(f"{frequency / 1_000_000:.3f}")
        self.vfo_badge.setText("B" if state.active_vfo_b else "A")
        active_mode = state.vfo_b_mode if state.active_vfo_b else state.vfo_a_mode
        mode_index = self.mode_selector.findData(active_mode)
        if mode_index >= 0 and mode_index != self.mode_selector.currentIndex():
            self.mode_selector.blockSignals(True)
            self.mode_selector.setCurrentIndex(mode_index)
            self.mode_selector.blockSignals(False)
        self.s_meter.set_value(state.s_meter)
        # Firmware only supplies a meaningful second meter while transmitting.
        self.swr_meter.set_value(state.swr if state.ptt else 0)
        if state.connected:
            self.connect_button.setText("Disconnect")
        elif state.listening:
            self.connect_button.setText("Stop Listener")
        else:
            self.connect_button.setText("Connect USB" if self.transport.currentIndex() == 1 else "Start Listener")
        self.status.setText(
            "On air" if state.ptt else
            (f"Radio connected via {state.transport}" if state.connected else
             ("Listening on TCP/8081. Waiting for radio..." if state.listening else "Listener stopped"))
        )
        self.tiles["RFG"].set_value(str(state.rf_gain))
        self.tiles["IFG"].set_value(str(state.if_gain))
        self.tiles["SQL"].set_value(str(state.squelch))
        self.tiles["AGC"].set_value(("Off", "Fast", "Mid", "Slow", "SSlow", "Auto")[state.agc])
        self.tiles["AMP"].set_value("On" if state.preamp else "Off")
        self.tiles["SVOL"].set_value(str(state.speaker_volume))
        self.tiles["NB"].set_value("Off" if state.noise_blanker == 0 else str(state.noise_blanker))
        self.tiles["NR"].set_value("Off" if state.noise_reduction == 0 else str(state.noise_reduction))
        self.tiles["SPLIT"].set_value("On" if state.split else "Off")
        self.tiles["A/B"].set_value("Frequency B" if state.active_vfo_b else "Frequency A")
        self.tiles["ATU"].set_value(("Off", "On", "Scan")[state.atu])
        self.tiles["SPAN"].set_value(f"{SPAN_HZ[state.span_index] / 1000:g} kHz")
        self.tiles["PWR"].set_value("High" if state.tx_power_high else "Low")
        self.tiles["TONE"].set_value(f"{state.cw_sidetone_hz} Hz")
        self.tiles["SPEED"].set_value(str(state.cw_speed))
        self.spectrum.set_state(state)
        if state.connected and state.transport == "USB" and not self.audio.running:
            QTimer.singleShot(0, self.start_audio_default)

    def show_error(self, message: str) -> None:
        self.status.setText(f"Connection error: {message}")

    def closeEvent(self, event) -> None:  # type: ignore[no-untyped-def]
        self.audio.stop()
        self.client.disconnect()
        event.accept()


def self_test() -> None:
    assert crc16_ccitt(bytes.fromhex("0339")) == 0xEF26
    assert encode_frame(Command.STATUS).hex() == "a5a5a5a5030bf937"
    cat = encode_frame(Command.SET_FREQUENCIES, b"\x01\x02")
    spectrum_raw = bytes((0, 0)) + bytes(range(256)) * 2
    spectrum = SYNC + spectrum_raw + crc16_ccitt(spectrum_raw).to_bytes(2, "big")
    parser = StreamParser()
    assert parser.feed(cat[:6]) == []
    frames = parser.feed(cat[6:] + spectrum)
    assert isinstance(frames[0], CatFrame) and frames[0].command == Command.SET_FREQUENCIES
    assert isinstance(frames[1], SpectrumFrame) and len(frames[1].bins) == 512
    # Some firmware revisions have a spectrum CRC tail that does not validate.
    # Match the reference app: accept plausible frames during CRC auto-detection.
    invalid_crc_spectrum = SYNC + spectrum_raw + b"\x00\x00"
    fallback = StreamParser()
    assert isinstance(fallback.feed(invalid_crc_spectrum)[0], SpectrumFrame)
    print("Q900 protocol self-test passed")


def main() -> None:
    if "--self-test" in sys.argv:
        self_test()
        return
    app = QApplication(sys.argv)
    app.setStyleSheet(STYLESHEET)
    app.setFont(QFont("Arial", 10))
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
