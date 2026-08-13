#!/usr/bin/env python3
"""Standalone PyQt6 control console for the Q900 radio.

The CAT and spectrum protocol definitions in this file follow
qpmrpancatweb_1.15.html, the USB CAT reference application.
"""

from __future__ import annotations

from collections import deque
import ctypes
from dataclasses import dataclass
from enum import IntEnum
import multiprocessing as mp
import queue
import signal
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
    QDialog,
    QDialogButtonBox,
    QScrollArea,
    QSlider,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)


SYNC = b"\xa5\xa5\xa5\xa5"
SPECTRUM_PAYLOAD_LENGTH = 516
SPECTRUM_BINS = 512
SPAN_HZ = (48_000, 24_000, 12_000, 6_000, 3_000, 1_500)
# The Q900 default IQ translation places the CAT-tuned carrier 12 kHz above
# the stream reference. Use the same reference for the CAT spectrum cursor.
FFT_TUNED_OFFSET_HZ = 12_000


def set_interactive_qos() -> None:
    """Keep latency-sensitive media workers out of macOS background QoS."""
    if sys.platform != "darwin":
        return
    try:
        system = ctypes.CDLL("/usr/lib/libSystem.B.dylib")
        system.pthread_set_qos_class_self_np(0x21, 0)
    except (AttributeError, OSError):
        pass


class Command(IntEnum):
    PTT = 0x07
    SET_FREQUENCIES = 0x09
    SET_MODES = 0x0A
    STATUS = 0x0B
    POWER = 0x0C
    SPEAKER_VOLUME = 0x0D
    HEADPHONE_VOLUME = 0x0E
    MIC_GAIN = 0x0F
    COMPRESSOR = 0x10
    TX_BASS = 0x11
    TX_TREBLE = 0x12
    RF_GAIN = 0x13
    IF_GAIN = 0x14
    SQUELCH = 0x15
    AGC = 0x16
    PREAMP = 0x17
    NOISE_REDUCTION = 0x19
    NOISE_BLANKER = 0x1A
    ACTIVE_VFO = 0x1B
    SPLIT = 0x1C
    NOISE_BLANKER_THRESHOLD = 0x1F
    PEAK_THRESHOLD = 0x20
    ATU = 0x21
    SPAN = 0x22
    TX_POWER = 0x2C
    CW_SIDETONE = 0x31
    CW_TXRX_DELAY = 0x32
    CW_SPEED = 0x35
    USB_FORMAT = 0x33
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


# Q900 CAT mode values. The Q900 user manual includes FT8/data/custom-digital
# operation; DIGI and PKT are valid CAT selections (7 and 8), not merely
# status values.
SELECTABLE_MODES = tuple(Mode)


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
    headphone_volume: int = 0
    mic_gain: int = 6
    compressor: int = 9
    tx_bass: int = 20
    tx_treble: int = 20
    noise_blanker_threshold: int = 7
    peak_threshold: int = 15
    cw_txrx_delay: int = 100


class RadioSignals(QObject):
    state_changed = pyqtSignal(object)
    spectrum_received = pyqtSignal(bytes)
    connection_error = pyqtSignal(str)
    audio_state_changed = pyqtSignal(str)
    rigctl_clients_changed = pyqtSignal(int)
    rigctl_ptt_requested = pyqtSignal(bool)
    sdr_stream_changed = pyqtSignal(bool)


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

    @staticmethod
    def microphone_devices() -> list[tuple[int, str]]:
        return [
            (index, device["name"])
            for index, device in enumerate(sd.query_devices())
            if device["max_input_channels"] > 0 and "q900" not in device["name"].lower()
        ]

    @staticmethod
    def q900_output_devices() -> list[tuple[int, str]]:
        return [
            (index, device["name"])
            for index, device in enumerate(sd.query_devices())
            if device["max_output_channels"] > 0 and "q900" in device["name"].lower()
        ]

    @staticmethod
    def named_device(name: str, direction: str) -> int | None:
        key = name.casefold()
        for index, device in enumerate(sd.query_devices()):
            channels = device[f"max_{direction}_channels"]
            if channels > 0 and device["name"].casefold() == key:
                return index
        return None

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


class SDRReceiver:
    """Worker-based 48 kHz complex I/Q USB/LSB receive demodulator."""

    SAMPLE_RATE = 48_000
    BLOCK_FRAMES = 960
    OUTPUT_PREROLL_BLOCKS = 13
    SSB_OUTPUT_GAIN = 40.0
    NFM_OUTPUT_GAIN = 3.0
    AM_OUTPUT_GAIN = 24.0

    def __init__(self, output: Callable[[np.ndarray], None]) -> None:
        self._output = output
        self._queue: queue.Queue[np.ndarray | None] = queue.Queue(maxsize=32)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._input_blocks: deque[np.ndarray] = deque()
        self._input_words = 0
        self._input_lock = threading.Lock()
        self.mode = "USB"
        # Q900 network IQ places the CAT-tuned carrier near +12 kHz.
        self.offset_hz = 12_000
        self.swap_iq = False
        self.invert_q = False

    def start(self) -> None:
        if self._thread:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="q900-sdr-rx", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        try:
            self._queue.put_nowait(None)
        except queue.Full:
            pass
        if self._thread:
            self._thread.join(timeout=0.5)
        self._thread = None
        with self._input_lock:
            self._input_blocks.clear()
            self._input_words = 0
        while not self._queue.empty():
            try:
                self._queue.get_nowait()
            except queue.Empty:
                break

    def feed(self, words: np.ndarray) -> None:
        if len(words) < 2:
            return
        with self._input_lock:
            self._input_blocks.append(words.copy())
            self._input_words += len(words)
            if self._input_words < self.BLOCK_FRAMES * 2:
                return
            blocks: list[np.ndarray] = []
            remaining = self.BLOCK_FRAMES * 2
            while remaining:
                block = self._input_blocks.popleft()
                if len(block) <= remaining:
                    blocks.append(block)
                    remaining -= len(block)
                else:
                    blocks.append(block[:remaining])
                    self._input_blocks.appendleft(block[remaining:])
                    remaining = 0
            self._input_words -= self.BLOCK_FRAMES * 2
            block = np.concatenate(blocks)
        try:
            self._queue.put_nowait(block)
        except queue.Full:
            pass

    def _run(self) -> None:
        set_interactive_qos()
        phase = 0
        dc = 0j
        previous = 1 + 0j
        fm_dc = 0.0
        fm_deemphasis = 0.0
        am_history = np.zeros(64, dtype=np.complex64)
        am_taps = self._lowpass_taps(4_500, 65)
        ssb_previous_input = 0.0
        ssb_previous_output = 0.0
        output_pending: deque[np.ndarray] = deque()
        while not self._stop.is_set():
            try:
                words = self._queue.get(timeout=0.1)
            except queue.Empty:
                continue
            if words is None:
                continue
            iq = words.astype(np.float32).reshape(-1, 2) / 32768.0
            if self.swap_iq:
                iq = iq[:, ::-1]
            signal = iq[:, 0] + 1j * iq[:, 1] * (-1 if self.invert_q else 1)
            # Do not subtract each packet's mean: at a 1 ms packet size that
            # removes/modulates wanted low-frequency SSB audio. Track only the
            # slowly varying I/Q DC component across full audio blocks.
            dc = 0.995 * dc + 0.005 * np.mean(signal)
            signal -= dc
            count = len(signal)
            index = np.arange(count) + phase
            if self.mode == "NFM":
                shift = np.exp(-1j * 2 * np.pi * self.offset_hz * index / self.SAMPLE_RATE)
                baseband = signal * shift
                discriminator = np.angle(baseband * np.conj(np.concatenate(([previous], baseband[:-1]))))
                previous = baseband[-1]
                # Remove residual carrier offset, then apply a 300 us
                # de-emphasis filter for intelligible narrow-FM audio.
                fm_dc = 0.995 * fm_dc + 0.005 * float(np.mean(discriminator))
                discriminator -= fm_dc
                alpha = 1 - np.exp(-1 / (self.SAMPLE_RATE * 300e-6))
                audio = np.empty_like(discriminator)
                for sample_index, sample in enumerate(discriminator):
                    fm_deemphasis += alpha * (sample - fm_deemphasis)
                    audio[sample_index] = fm_deemphasis
                audio = np.convolve(audio, np.ones(7, dtype=np.float32) / 7, mode="same")
                gain = self.NFM_OUTPUT_GAIN
            elif self.mode == "AM":
                shift = np.exp(-1j * 2 * np.pi * self.offset_hz * index / self.SAMPLE_RATE)
                baseband = signal * shift
                # Isolate the selected AM channel before envelope detection.
                # Taking |I+jQ| across the whole 48 kHz stream demodulates
                # every nearby carrier/noise source into an audible buzz.
                combined = np.concatenate((am_history, baseband))
                baseband = np.convolve(combined, am_taps, mode="valid")
                am_history = combined[-64:]
                envelope = np.abs(baseband)
                # The carrier is the envelope DC term. Removing the block
                # mean makes AM audio available immediately on entry rather
                # than waiting seconds for a slow DC follower to settle.
                audio = envelope - np.mean(envelope)
                gain = self.AM_OUTPUT_GAIN
            else:
                # USB and LSB share the same suppressed-carrier frequency.
                # Sideband content is carried in the complex samples around
                # it, so both must translate the selected carrier to zero.
                shift = np.exp(-1j * 2 * np.pi * self.offset_hz * index / self.SAMPLE_RATE)
                baseband = signal * shift
                audio = np.convolve(baseband.real, np.ones(9, dtype=np.float32) / 9, mode="same")
                # Remove residual carrier/DC without suppressing voice tones.
                highpassed = np.empty_like(audio)
                for sample_index, sample in enumerate(audio):
                    filtered = sample - ssb_previous_input + 0.995 * ssb_previous_output
                    ssb_previous_input = sample
                    ssb_previous_output = filtered
                    highpassed[sample_index] = filtered
                audio = highpassed
                gain = self.SSB_OUTPUT_GAIN
            phase += count
            # Keep SDR audio gain fixed. The previous block AGC could clamp
            # weak USB/LSB speech after a stronger packet and sound as if the
            # decoder was repeatedly muted.
            output_pending.append(np.clip(audio * gain, -1.0, 1.0).astype(np.float32))
            # macOS can defer a backgrounded GUI process for substantially
            # longer than a normal UDP gap. Keep 260 ms ahead of playback.
            if len(output_pending) >= self.OUTPUT_PREROLL_BLOCKS:
                self._output(output_pending.popleft())

    @staticmethod
    def _lowpass_taps(cutoff_hz: float, count: int) -> np.ndarray:
        index = np.arange(count, dtype=np.float32) - (count - 1) / 2
        taps = 2 * cutoff_hz / SDRReceiver.SAMPLE_RATE * np.sinc(2 * cutoff_hz * index / SDRReceiver.SAMPLE_RATE)
        taps *= np.hamming(count)
        return (taps / np.sum(taps)).astype(np.float32)


class NetworkAudioMonitor:
    """Receive Q900 UDP/8000 signed-16 PCM and play it locally."""

    SAMPLE_RATE = 48_000
    BLOCK_SIZE = 960

    def __init__(self, signals: RadioSignals) -> None:
        self.signals = signals
        self._socket: socket.socket | None = None
        self._stream: sd.OutputStream | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._queue: deque[np.ndarray] = deque()
        self._queue_lock = threading.Lock()
        self._queued_frames = 0
        self._packet_count = 0
        self._last_packet_size = 0
        self._format = "waiting"
        self._stats_lock = threading.Lock()
        self._iq_handler: Callable[[np.ndarray], None] | None = None
        self._stream_type = 0

    def start(self, output_device: int, port: int = 8000) -> None:
        self.stop()
        output_info = sd.query_devices(output_device, "output")
        output_channels = min(2, output_info["max_output_channels"])
        max_queued_frames = self.SAMPLE_RATE * 2

        def output_callback(outdata, frames, timing, status):  # type: ignore[no-untyped-def]
            outdata.fill(0)
            offset = 0
            with self._queue_lock:
                while offset < frames and self._queue:
                    block = self._queue[0]
                    count = min(frames - offset, len(block))
                    outdata[offset : offset + count, :] = block[:count, np.newaxis]
                    offset += count
                    if count == len(block):
                        self._queue.popleft()
                    else:
                        self._queue[0] = block[count:]
                    self._queued_frames -= count

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Do not share UDP/8000. On macOS SO_REUSEADDR allows another local
        # process to bind the same port and consume the radio's datagrams.
        # A bind conflict must fail visibly rather than leave us "waiting".
        try:
            sock.bind(("0.0.0.0", port))
        except OSError:
            sock.close()
            raise
        sock.settimeout(0.25)
        self._socket = sock
        self._stop.clear()
        self._stream = sd.OutputStream(
            device=output_device,
            samplerate=self.SAMPLE_RATE,
            blocksize=self.BLOCK_SIZE,
            channels=output_channels,
            dtype="float32",
            latency="high",
            callback=output_callback,
        )
        self._stream.start()

        def receive_loop() -> None:
            set_interactive_qos()
            while not self._stop.is_set() and self._socket:
                try:
                    packet, peer = self._socket.recvfrom(65_535)
                except TimeoutError:
                    continue
                except OSError:
                    break
                packet_type = packet[4] if packet.startswith(SYNC) and len(packet) >= 9 else 0
                if packet_type == 0x68:
                    payload = packet[9:]
                    if len(payload) % 2:
                        continue
                    handler = self._iq_handler
                    if handler:
                        handler(np.frombuffer(payload, dtype="<i2"))
                    with self._stats_lock:
                        self._packet_count += 1
                        self._last_packet_size = len(payload)
                        self._format = "Q900 IQ S16LE"
                        self._stream_type = packet_type
                    continue
                # Normal radio audio is duplicated stereo S16LE.
                if packet_type == 0x67:
                    packet = packet[9:]
                    audio_format = "Q900 framed S16LE stereo"
                else:
                    audio_format = "S16LE"
                # Also accept unframed PCM from firmware variants.
                if len(packet) < 2 or len(packet) % 2:
                    continue
                samples = np.frombuffer(packet, dtype="<i2").astype(np.float32) / 32768.0
                if audio_format.startswith("Q900 framed") or (len(packet) >= 3_840 and len(packet) % 4 == 0):
                    mono = samples.reshape(-1, 2).mean(axis=1)
                    if not audio_format.startswith("Q900 framed"):
                        audio_format += " stereo"
                else:
                    mono = samples
                    audio_format += " mono"
                with self._queue_lock:
                    while self._queue and self._queued_frames + len(mono) > max_queued_frames:
                        self._queued_frames -= len(self._queue.popleft())
                    self._queue.append(mono)
                    self._queued_frames += len(mono)
                with self._stats_lock:
                    first_packet = self._packet_count == 0
                    self._packet_count += 1
                    self._last_packet_size = len(packet)
                    self._format = audio_format
                    self._stream_type = packet_type
                if first_packet:
                    print(
                        f"Q900 UDP audio from {peer[0]}:{peer[1]}: {len(packet)} bytes, "
                        f"{audio_format}, first 16 bytes={packet[:16].hex()}"
                    )

        self._thread = threading.Thread(target=receive_loop, name="q900-udp-audio", daemon=True)
        self._thread.start()
        self.signals.audio_state_changed.emit(f"Network RX audio: listening on UDP/{port} -> {output_info['name']}")

    def set_iq_handler(self, handler: Callable[[np.ndarray], None] | None) -> None:
        self._iq_handler = handler

    def enqueue_audio(self, samples: np.ndarray) -> None:
        with self._queue_lock:
            while self._queue and self._queued_frames + len(samples) > self.SAMPLE_RATE * 2:
                self._queued_frames -= len(self._queue.popleft())
            self._queue.append(samples)
            self._queued_frames += len(samples)

    @property
    def stream_type(self) -> int:
        with self._stats_lock:
            return self._stream_type

    def stop(self) -> None:
        self._stop.set()
        sock, self._socket = self._socket, None
        if sock:
            sock.close()
        if self._stream:
            self._stream.stop()
            self._stream.close()
        self._stream = None
        with self._queue_lock:
            self._queue.clear()
            self._queued_frames = 0
        with self._stats_lock:
            self._packet_count = 0
            self._last_packet_size = 0
            self._format = "waiting"
            self._stream_type = 0

    def sendto(self, payload: bytes, target: tuple[str, int]) -> None:
        """Send from the same UDP/8000 socket used by the Q900 media session."""
        if not self._socket:
            raise ConnectionError("Network audio receiver is not running")
        self._socket.sendto(payload, target)

    @property
    def socket(self) -> socket.socket:
        if not self._socket:
            raise ConnectionError("Network audio receiver is not running")
        return self._socket

    @property
    def running(self) -> bool:
        return self._stream is not None

    @property
    def status(self) -> str:
        with self._stats_lock:
            if not self._packet_count:
                return "UDP waiting"
            return f"UDP {self._packet_count} pkts  {self._last_packet_size} B  {self._format}"


def udp_audio_sender(
    audio_queue: mp.Queue,
    udp_socket: socket.socket,
    target: tuple[str, int],
    stop: mp.Event,
    keyed: mp.Event,
    packets: mp.Value,
    underruns: mp.Value,
    late_ms: mp.Value,
) -> None:
    """Pace TX audio outside the GUI process and its contended Python GIL."""
    packet_bytes = 96 * 2 * 2
    preroll_bytes = 9_600 * 2 * 2
    pending = bytearray()
    while len(pending) < preroll_bytes and not stop.is_set():
        try:
            pending.extend(audio_queue.get(timeout=0.05))
        except queue.Empty:
            continue
    while not keyed.wait(0.05) and not stop.is_set():
        pass

    period = 0.002

    def next_payload() -> bytes:
        while len(pending) < packet_bytes:
            try:
                # Capture arrives in 20 ms blocks through a feeder pipe. At
                # block boundaries get_nowait() can race that feeder and turn
                # available audio into a false underrun. Wait no longer than
                # one native packet before substituting silence.
                pending.extend(audio_queue.get(timeout=period))
            except queue.Empty:
                break
        if len(pending) < packet_bytes:
            underruns.value += 1
            return bytes(packet_bytes)
        payload = bytes(pending[:packet_bytes])
        del pending[:packet_bytes]
        return payload

    # Put 20 ms into the radio's TX ring before paced delivery. The host wake
    # delay is below one packet now, but the firmware/network path still has
    # several milliseconds of variance that a two-packet cushion cannot hide.
    for _ in range(10):
        try:
            udp_socket.sendto(next_payload(), target)
        except OSError:
            pass
        packets.value += 1

    mach_time = mach_wait = None
    ticks_per_second = 0.0
    if sys.platform == "darwin":
        class TimebaseInfo(ctypes.Structure):
            _fields_ = [("numer", ctypes.c_uint32), ("denom", ctypes.c_uint32)]

        try:
            system = ctypes.CDLL("/usr/lib/libSystem.B.dylib")
            info = TimebaseInfo()
            system.mach_timebase_info(ctypes.byref(info))
            system.mach_absolute_time.restype = ctypes.c_uint64
            system.mach_wait_until.argtypes = (ctypes.c_uint64,)
            system.pthread_set_qos_class_self_np(0x21, 0)
            mach_time = system.mach_absolute_time
            mach_wait = system.mach_wait_until
            ticks_per_second = 1_000_000_000 * info.denom / info.numer
        except (AttributeError, OSError):
            mach_time = mach_wait = None
    deadline = mach_time() if mach_time else time.monotonic()
    while not stop.is_set():
        try:
            udp_socket.sendto(next_payload(), target)
        except OSError:
            pass
        packets.value += 1
        if mach_time and mach_wait:
            deadline += int(period * ticks_per_second)
            mach_wait(deadline)
            lateness = (mach_time() - deadline) / ticks_per_second
            late_ms.value = max(late_ms.value, lateness * 1000)
            if lateness > period:
                # Do not catch up by sending several packets back-to-back.
                # A stale Mach deadline drains the capture buffer and makes
                # one delayed wake appear as a run of audio underruns.
                deadline = mach_time()
        else:
            deadline += period
            remaining = deadline - time.monotonic()
            if remaining < 0:
                late_ms.value = max(late_ms.value, -remaining * 1000)
                deadline = time.monotonic()
            else:
                time.sleep(remaining)


class TransmitAudioRouter:
    """Route computer microphone audio to USB playback or the network audio port."""

    SAMPLE_RATE = 48_000
    BLOCK_SIZE = 960
    NETWORK_SAMPLE_RATE = 48_000
    NETWORK_PACKET_SAMPLES = 96
    NETWORK_PREROLL_SAMPLES = 9_600

    def __init__(self, signals: RadioSignals) -> None:
        self.signals = signals
        self._input_stream: sd.InputStream | None = None
        self._output_stream: sd.OutputStream | None = None
        self._udp_socket: socket.socket | None = None
        self._udp_target: tuple[str, int] | None = None
        self._level = 0.0
        self._level_lock = threading.Lock()
        self._output_level = 0.0
        self._usb_queue: deque[np.ndarray] = deque()
        self._usb_queue_lock = threading.Lock()
        self._usb_queued_frames = 0
        self._mp = mp.get_context("spawn")
        self._udp_queue: mp.Queue | None = None
        self._udp_sender: mp.Process | None = None
        self._udp_stop: mp.Event | None = None
        self._udp_keyed: mp.Event | None = None
        self._udp_packets: mp.Value | None = None
        self._udp_underruns: mp.Value | None = None
        self._udp_late_ms: mp.Value | None = None
        self._udp_clipped: mp.Value | None = None

    def start_usb(self, microphone: int, q900_output: int) -> None:
        self.stop()
        output_info = sd.query_devices(q900_output, "output")
        output_channels = min(2, output_info["max_output_channels"])
        max_queued_frames = self.SAMPLE_RATE * 2

        def input_callback(indata, frames, timing, status):  # type: ignore[no-untyped-def]
            with self._level_lock:
                self._level = float(np.max(np.abs(indata[:, 0])))
            mono = indata[:, 0].copy()
            with self._usb_queue_lock:
                while self._usb_queue and self._usb_queued_frames + frames > max_queued_frames:
                    self._usb_queued_frames -= len(self._usb_queue.popleft())
                self._usb_queue.append(mono)
                self._usb_queued_frames += frames

        def output_callback(outdata, frames, timing, status):  # type: ignore[no-untyped-def]
            outdata.fill(0)
            offset = 0
            with self._usb_queue_lock:
                while offset < frames and self._usb_queue:
                    block = self._usb_queue[0]
                    count = min(frames - offset, len(block))
                    # q900_output is explicitly the Q900 speaker/output
                    # interface (device 0 here), not its microphone input.
                    outdata[offset : offset + count, :] = block[:count, np.newaxis]
                    offset += count
                    if count == len(block):
                        self._usb_queue.popleft()
                    else:
                        self._usb_queue[0] = block[count:]
                    self._usb_queued_frames -= count
            with self._level_lock:
                self._output_level = float(np.max(np.abs(outdata[:, 0])))

        self._input_stream = sd.InputStream(
            device=microphone,
            samplerate=self.SAMPLE_RATE,
            blocksize=self.BLOCK_SIZE,
            channels=1,
            dtype="float32",
            latency="high",
            callback=input_callback,
        )
        self._output_stream = sd.OutputStream(
            device=q900_output,
            samplerate=self.SAMPLE_RATE,
            blocksize=self.BLOCK_SIZE,
            channels=output_channels,
            dtype="float32",
            latency="high",
            callback=output_callback,
        )
        self._output_stream.start()
        self._input_stream.start()
        self.signals.audio_state_changed.emit("PTT audio: microphone -> Q900 USB speaker/output")

    def start_udp(self, microphone: int, target: tuple[str, int], network_audio: NetworkAudioMonitor) -> None:
        self.stop()
        self._udp_target = target
        self._udp_queue = self._mp.Queue(maxsize=50)
        self._udp_stop = self._mp.Event()
        self._udp_keyed = self._mp.Event()
        self._udp_packets = self._mp.Value("L", 0, lock=False)
        self._udp_underruns = self._mp.Value("L", 0, lock=False)
        self._udp_late_ms = self._mp.Value("d", 0.0, lock=False)
        self._udp_clipped = self._mp.Value("L", 0, lock=False)

        def callback(indata, frames, timing, status):  # type: ignore[no-untyped-def]
            pcm = np.clip(indata[:, 0], -1, 1)
            with self._level_lock:
                self._level = float(np.max(np.abs(pcm)))
                self._output_level = self._level
            if self._udp_clipped and np.any(np.abs(pcm) >= 0.98):
                self._udp_clipped.value += 1
            samples = (pcm * 32767).astype("<i2")
            # Firmware consumes 48 kHz stereo frames from the network ring.
            # Duplicate mono microphone samples as interleaved L/R words.
            payload = np.repeat(samples, 2).tobytes()
            if self._udp_queue:
                try:
                    self._udp_queue.put_nowait(payload)
                except queue.Full:
                    pass

        self._input_stream = sd.InputStream(
            device=microphone,
            samplerate=self.NETWORK_SAMPLE_RATE,
            blocksize=self.BLOCK_SIZE,
            channels=1,
            dtype="float32",
            latency="high",
            callback=callback,
        )
        self._udp_sender = self._mp.Process(
            target=udp_audio_sender,
            args=(
                self._udp_queue,
                network_audio.socket,
                target,
                self._udp_stop,
                self._udp_keyed,
                self._udp_packets,
                self._udp_underruns,
                self._udp_late_ms,
            ),
            name="q900-udp-tx",
            daemon=True,
        )
        self._udp_sender.start()
        self._input_stream.start()
        time.sleep(self.NETWORK_PREROLL_SAMPLES / self.NETWORK_SAMPLE_RATE)
        self.signals.audio_state_changed.emit(
            f"PTT audio: microphone -> Q900 UDP {target[0]}:{target[1]} (48 kHz stereo S16LE)"
        )

    def network_ptt_started(self) -> None:
        """Start UDP delivery only after CAT PTT has enabled the radio's TX ring."""
        if self._udp_keyed:
            self._udp_keyed.set()

    def stop(self) -> None:
        if self._udp_stop:
            self._udp_stop.set()
        if self._udp_keyed:
            self._udp_keyed.set()
        if self._udp_sender:
            self._udp_sender.join(timeout=0.5)
            if self._udp_sender.is_alive():
                self._udp_sender.terminate()
                self._udp_sender.join(timeout=0.5)
        self._udp_sender = None
        self._udp_queue = None
        self._udp_stop = None
        self._udp_keyed = None
        for stream in (self._input_stream, self._output_stream):
            if stream:
                stream.stop()
                stream.close()
        self._input_stream = None
        self._output_stream = None
        if self._udp_socket:
            self._udp_socket.close()
        self._udp_socket = None
        self._udp_target = None
        with self._usb_queue_lock:
            self._usb_queue.clear()
            self._usb_queued_frames = 0
        with self._level_lock:
            self._level = 0.0
            self._output_level = 0.0

    @property
    def running(self) -> bool:
        return self._input_stream is not None or self._output_stream is not None

    @property
    def level(self) -> float:
        with self._level_lock:
            return self._level

    @property
    def output_level(self) -> float:
        with self._level_lock:
            return self._output_level

    @property
    def network_status(self) -> str:
        packets = self._udp_packets.value if self._udp_packets else 0
        underruns = self._udp_underruns.value if self._udp_underruns else 0
        late_ms = self._udp_late_ms.value if self._udp_late_ms else 0.0
        clipped = self._udp_clipped.value if self._udp_clipped else 0
        return f"UDP {packets} pkts  gaps {underruns}  late {late_ms:.1f} ms  clip {clipped}"


class RigctlServer:
    """Local Hamlib rigctl subset backed by the application's radio state."""

    LEVELS = {
        "AF": ("speaker_volume", Command.SPEAKER_VOLUME, 0, 30),
        "RF": ("rf_gain", Command.RF_GAIN, 0, 100),
        "SQL": ("squelch", Command.SQUELCH, 0, 20),
        "MICGAIN": ("mic_gain", Command.MIC_GAIN, 0, 100),
    }

    def __init__(self, client: RadioClient, signals: RadioSignals) -> None:
        self.client = client
        self.signals = signals
        self._listener: socket.socket | None = None
        self._stop = threading.Event()
        self._clients: set[socket.socket] = set()
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None

    @property
    def client_count(self) -> int:
        with self._lock:
            return len(self._clients)

    def start(self, port: int = 4532) -> None:
        if self._listener:
            return
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            listener.bind(("127.0.0.1", port))
        except OSError:
            listener.close()
            raise
        listener.listen()
        listener.settimeout(0.5)
        self._listener = listener
        self._stop.clear()
        self._thread = threading.Thread(target=self._accept_loop, name="rigctl", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        listener, self._listener = self._listener, None
        if listener:
            listener.close()
        with self._lock:
            clients, self._clients = tuple(self._clients), set()
        for client in clients:
            client.close()
        self.signals.rigctl_ptt_requested.emit(False)
        self.signals.rigctl_clients_changed.emit(0)

    @staticmethod
    def _reply(sock: socket.socket, *lines: str) -> None:
        sock.sendall(("\n".join(lines) + "\n").encode())

    @staticmethod
    def _ok(sock: socket.socket) -> None:
        sock.sendall(b"RPRT 0\n")

    @staticmethod
    def _error(sock: socket.socket, code: int = -11) -> None:
        sock.sendall(f"RPRT {code}\n".encode())

    def _accept_loop(self) -> None:
        while not self._stop.is_set() and self._listener:
            try:
                sock, _address = self._listener.accept()
            except TimeoutError:
                continue
            except OSError:
                break
            sock.settimeout(0.5)
            with self._lock:
                self._clients.add(sock)
                count = len(self._clients)
            self.signals.rigctl_clients_changed.emit(count)
            threading.Thread(target=self._client_loop, args=(sock,), name="rigctl-client", daemon=True).start()

    def _client_loop(self, sock: socket.socket) -> None:
        buffer = bytearray()
        try:
            while not self._stop.is_set():
                try:
                    data = sock.recv(1024)
                except TimeoutError:
                    continue
                if not data:
                    break
                buffer.extend(data)
                while b"\n" in buffer:
                    raw, _, buffer = buffer.partition(b"\n")
                    self._handle(sock, raw.decode(errors="ignore").strip())
        except OSError:
            pass
        finally:
            with self._lock:
                self._clients.discard(sock)
                count = len(self._clients)
            sock.close()
            if count == 0:
                self.signals.rigctl_ptt_requested.emit(False)
            self.signals.rigctl_clients_changed.emit(count)

    def _handle(self, sock: socket.socket, line: str) -> None:
        if not line:
            return
        if line.startswith("\\"):
            command, *arguments = line.split(None, 1)
            args = arguments[0] if arguments else ""
        else:
            command, args = line[0], line[1:].strip()
        state = self.client.state
        try:
            if command == "q":
                self._ok(sock)
                sock.close()
            elif command in ("f", "\\get_freq"):
                self._reply(sock, str(state.vfo_b_hz if state.active_vfo_b else state.vfo_a_hz))
            elif command in ("F", "\\set_freq"):
                self.client.tune(int(float(args)))
                self._ok(sock)
            elif command in ("m", "\\get_mode"):
                mode = state.vfo_b_mode if state.active_vfo_b else state.vfo_a_mode
                rigctl_mode = "FM" if mode == Mode.NFM else mode.name
                self._reply(sock, rigctl_mode, "2400")
            elif command in ("M", "\\set_mode"):
                mode_name = args.split()[0].upper()
                self.client.set_mode(Mode.NFM if mode_name == "FM" else Mode[mode_name])
                self._ok(sock)
            elif command in ("t", "\\get_ptt"):
                self._reply(sock, str(int(state.ptt)))
            elif command in ("T", "\\set_ptt"):
                active = bool(int(args))
                self.signals.rigctl_ptt_requested.emit(active)
                self._ok(sock)
            elif command in ("v", "\\get_vfo"):
                self._reply(sock, "VFOB" if state.active_vfo_b else "VFOA")
            elif command in ("V", "\\set_vfo"):
                self.client.select_vfo(args.upper() == "VFOB")
                self._ok(sock)
            elif command in ("s", "\\get_split_vfo"):
                self._reply(sock, str(int(state.split)), "VFOB" if state.active_vfo_b else "VFOA")
            elif command in ("S", "\\set_split_vfo"):
                self.client.set_split(bool(int(args.split()[0])))
                self._ok(sock)
            elif command in ("l", "\\get_level") and args.upper() in self.LEVELS:
                field, _cat, _minimum, _maximum = self.LEVELS[args.upper()]
                self._reply(sock, str(getattr(state, field)))
            elif command in ("L", "\\set_level") and len(args.split()) == 2:
                level, value = args.split()
                field, cat, minimum, maximum = self.LEVELS[level.upper()]
                self.client.set_value(field, cat, max(minimum, min(maximum, int(float(value)))))
                self._ok(sock)
            elif command == "\\chk_vfo":
                self._reply(sock, "0")
            elif command == "\\get_info":
                self._reply(sock, "Q900 Control rigctl relay")
            elif command == "\\dump_state":
                modes = "0x1ff"
                self._reply(
                    sock,
                    "0", "2", "0",
                    f"100000.000000 2000000000.000000 {modes} -1 -1 0x1 0x1",
                    "0 0 0 0 0 0 0",
                    f"100000.000000 2000000000.000000 {modes} 1 100 0x1 0x1",
                    "0 0 0 0 0 0 0",
                    *[f"{modes} {step}" for step in (1, 10, 100, 1000, 5000, 10000)],
                    "0 0",
                    "0xc 2400", "0x2 500", "0x1 6000", "0x20 12000", "0 0",
                    "0", "0", "0", "0", "", "", "0x0", "0x0", "0x1", "0x0", "0x0", "0x0",
                    "vfo_ops=0x0", "ptt_type=0x1", "done",
                )
            else:
                self._ok(sock)
        except (ConnectionError, KeyError, OSError, ValueError):
            self._error(sock)


class RadioClient:
    """TCP/8081 control listener using source-backed CAT commands only."""

    def __init__(self, signals: RadioSignals) -> None:
        self.state = RadioState()
        self.signals = signals
        self._listener: socket.socket | None = None
        self._socket: socket.socket | serial.Serial | None = None
        self._tcp_peer: tuple[str, int] | None = None
        self._digital_mode_locks: dict[bool, Mode] = {}
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
        self._digital_mode_locks.clear()
        listener, self._listener = self._listener, None
        if listener:
            listener.close()
        sock, self._socket = self._socket, None
        self._tcp_peer = None
        if sock:
            try:
                if isinstance(sock, socket.socket):
                    self._write(sock, encode_frame(Command.PTT, b"\x01"))
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

    def set_ptt(self, active: bool) -> None:
        self.send(encode_frame(Command.PTT, bytes((0 if active else 1,))))
        self.state.ptt = active
        self._emit_state()

    def set_stream_format(self, value: int) -> None:
        self.send(encode_frame(Command.USB_FORMAT, bytes((value,))))

    @property
    def udp_target(self) -> tuple[str, int] | None:
        return (self._tcp_peer[0], 8000) if self._tcp_peer else None

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
            device = serial.Serial(
                port=None,
                baudrate=baudrate,
                timeout=0.1,
                write_timeout=1,
                rtscts=False,
                dsrdtr=False,
            )
            # These states are applied by pyserial as part of open(), before
            # the receive loop emits a single CAT request.
            device.dtr = False
            device.rts = False
            device.port = port
            device.open()
            device.dtr = False
            device.rts = False
            # Let the USB CDC line state settle before CAT polling begins.
            # This avoids a status/spectrum request racing the radio's serial
            # control-line transition immediately after enumeration.
            time.sleep(0.25)
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
                sock, address = self._listener.accept()
            except TimeoutError:
                continue
            except OSError:
                break
            with self._lock:
                previous, self._socket = self._socket, sock
            if previous:
                previous.close()
            sock.settimeout(0.1)
            self._tcp_peer = (address[0], 8000)
            try:
                # The reference application releases PTT repeatedly on every connection.
                for _ in range(5):
                    self.send(encode_frame(Command.PTT, b"\x01"))
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
        active_vfo_b = self.state.active_vfo_b
        if self.state.active_vfo_b:
            self.state.vfo_b_mode = mode
        else:
            self.state.vfo_a_mode = mode
        # Firmware status reporting is unstable in DIGI/PKT, despite the
        # radio retaining the selected mode. Keep the operator's selection
        # from being overwritten by those transient status bytes.
        if mode in (Mode.DIGI, Mode.PKT):
            self._digital_mode_locks[active_vfo_b] = mode
        else:
            self._digital_mode_locks.pop(active_vfo_b, None)
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
        for vfo_b, raw_mode in ((False, data[1]), (True, data[2])):
            if raw_mode not in Mode._value2member_map_:
                continue
            reported_mode = Mode(raw_mode)
            # A digital mode selected before the app connects must receive the
            # same protection as one selected in the UI. Once it appears in a
            # status frame, ignore the firmware's subsequent unstable bytes.
            if reported_mode in (Mode.DIGI, Mode.PKT):
                self._digital_mode_locks.setdefault(vfo_b, reported_mode)
            mode = self._digital_mode_locks.get(vfo_b, reported_mode)
            if vfo_b:
                self.state.vfo_b_mode = mode
            else:
                self.state.vfo_a_mode = mode
        self.state.vfo_a_hz = int.from_bytes(data[3:7], "big")
        self.state.vfo_b_hz = int.from_bytes(data[7:11], "big")
        # The reference client does not apply status[11] as an A/B update.
        # Firmware reports this byte inconsistently during digital modes; the
        # active VFO is changed only by the operator's CAT command.
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
QSlider::groove:horizontal { background: #07111f; height: 5px; border-radius: 2px; }
QSlider::sub-page:horizontal { background: #34b8ca; border-radius: 2px; }
QSlider::handle:horizontal { background: #dce5ed; width: 12px; margin: -5px 0; border-radius: 6px; }
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
        self._sdr_active = False
        self._sdr_offset_hz = 0
        self._sdr_mode = "USB"

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

    def set_sdr(self, active: bool, offset_hz: int, mode: str) -> None:
        self._sdr_active = active
        self._sdr_offset_hz = offset_hz
        self._sdr_mode = mode
        self.update()

    def paintEvent(self, event) -> None:  # type: ignore[no-untyped-def]
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor("#080b10"))
        width, height = self.width(), self.height()
        spectrum_height = int(height * 0.43)
        self._draw_spectrum(painter, width, spectrum_height)
        self._draw_waterfall(painter, width, spectrum_height, height - spectrum_height)
        self._draw_tuned_cursor(painter, width, height)
        if self._sdr_active:
            self._draw_sdr_cursor(painter, width, height)

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

    def _draw_sdr_cursor(self, painter: QPainter, width: int, height: int) -> None:
        """Show the host-selected I/Q signal relative to the CAT frequency."""
        frequency = self._tuned_hz + self._sdr_offset_hz
        x = self._frequency_to_x(frequency, width)
        if self._sdr_mode == "NFM":
            low_hz, high_hz = -2_500, 2_500
        elif self._sdr_mode == "AM":
            low_hz, high_hz = -4_000, 4_000
        elif self._sdr_mode == "LSB":
            low_hz, high_hz = -2_800, -300
        else:
            low_hz, high_hz = 300, 2_800
        left = self._frequency_to_x(frequency + low_hz, width)
        right = self._frequency_to_x(frequency + high_hz, width)
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QColor(238, 174, 99, 62))
        painter.drawRect(QRectF(min(left, right), 0, max(2, abs(right - left)), height))
        painter.setPen(QPen(QColor("#eeae63"), 2))
        painter.drawLine(round(x), 0, round(x), height)
        painter.setPen(QColor("#eeae63"))
        label_x = min(max(8, int(x + 10)), max(8, width - 230))
        painter.drawText(label_x, 38, f"SDR {self._sdr_mode}  {self._sdr_offset_hz:+d} Hz")

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
        # Build a one-pixel-high scanline and let Qt scale it vertically. The
        # former nested Python loop wrote every pixel in each row height and
        # can consume a full core while SDR media is active.
        image = QImage(width, 1, QImage.Format.Format_RGB32)
        for row_number, bins in enumerate(self._rows[:height // row_height]):
            minimum, maximum = min(bins), max(bins)
            spread = max(1, maximum - minimum)
            for x in range(width):
                index = int(x * (len(bins) - 1) / max(1, width - 1))
                intensity = (bins[index] - minimum) * 255 // spread
                image.setPixel(
                    x,
                    0,
                    0xFF000000 | (intensity << 16) | ((80 + intensity * 175 // 255) << 8) | (40 + (255 - intensity) * 150 // 255),
                )
            painter.drawImage(QRectF(0, top + row_number * row_height, width, row_height), image)

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
        self.network_audio = NetworkAudioMonitor(self.signals)
        self.sdr_receiver = SDRReceiver(self.network_audio.enqueue_audio)
        self.tx_audio = TransmitAudioRouter(self.signals)
        self.rigctl = RigctlServer(self.client, self.signals)
        self._ptt_source: str | None = None
        self._virtual_receive_active = False
        self._last_ptt_network_status = ""
        self._sdr_active = False
        self._sdr_switch_pending = False
        self._sdr_restore_pending = False
        self._sdr_restore_attempts = 0
        self._ptt_meter_timer = QTimer(self)
        self._ptt_meter_timer.setInterval(100)
        self._ptt_meter_timer.timeout.connect(self.update_ptt_meter)
        self._network_audio_timer = QTimer(self)
        self._network_audio_timer.setInterval(500)
        self._network_audio_timer.timeout.connect(self.update_network_audio_status)
        self._sdr_switch_timer = QTimer(self)
        self._sdr_switch_timer.setSingleShot(True)
        self._sdr_switch_timer.timeout.connect(self.sdr_switch_timeout)
        self._sdr_restore_timer = QTimer(self)
        self._sdr_restore_timer.setSingleShot(True)
        self._sdr_restore_timer.timeout.connect(self.retry_normal_audio)
        self.tiles: dict[str, ControlTile] = {}
        self.signals.state_changed.connect(self.update_state)
        self.signals.connection_error.connect(self.show_error)
        self.signals.audio_state_changed.connect(self.show_audio_state)
        self.signals.rigctl_clients_changed.connect(self.update_rigctl_status)
        self.signals.rigctl_ptt_requested.connect(self.handle_rigctl_ptt)
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
        layout.addLayout(self._ptt_panel())
        self.status = QLabel("Listener stopped. Start TCP listening or connect over USB.")
        self.status.setStyleSheet("color: #9aaab5; padding: 4px 10px")
        layout.addWidget(self.status)
        try:
            self.rigctl.start()
        except OSError as error:
            self.rigctl_status.setText(f"rigctl: unavailable ({error})")
            self.rigctl_status.setStyleSheet("color: #eeae63; font: 13px Menlo")

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
        for mode in SELECTABLE_MODES:
            self.mode_selector.addItem(mode.name, mode)
        self.mode_selector.currentIndexChanged.connect(self.select_mode)
        self.mode_selector.setToolTip("Operating mode for the active VFO")
        header.addWidget(self.mode_selector)
        self.sdr_button = QPushButton("SDR Off")
        self.sdr_button.clicked.connect(self.toggle_sdr)
        self.sdr_mode_selector = QComboBox()
        self.sdr_mode_selector.addItems(("USB", "LSB", "NFM", "AM"))
        self.sdr_mode_selector.setVisible(False)
        self.sdr_mode_selector.currentTextChanged.connect(self.set_sdr_mode)
        self.sdr_offset = QSpinBox()
        self.sdr_offset.setRange(-24_000, 24_000)
        self.sdr_offset.setSingleStep(100)
        self.sdr_offset.setSuffix(" Hz")
        self.sdr_offset.setValue(self.sdr_receiver.offset_hz)
        self.sdr_offset.valueChanged.connect(self.set_sdr_offset)
        self.sdr_offset.setVisible(False)
        header.addWidget(self.sdr_button)
        header.addWidget(self.sdr_mode_selector)
        header.addWidget(self.sdr_offset)
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
            ("POWER", "Wake", "power"), ("RFG", "48", ("rf_gain", Command.RF_GAIN, 0, 100)),
            ("IFG", "50", ("if_gain", Command.IF_GAIN, 0, 80)), ("SQL", "0", ("squelch", Command.SQUELCH, 0, 20)),
            ("AGC", "Slow", "agc"), ("AMP", "Off", "preamp"), ("SVOL", "0", ("speaker_volume", Command.SPEAKER_VOLUME, 0, 30)),
            ("HVOL", "0", ("headphone_volume", Command.HEADPHONE_VOLUME, 5, 80)), ("MIC", "6", ("mic_gain", Command.MIC_GAIN, 0, 100)),
            ("CMP", "9", ("compressor", Command.COMPRESSOR, 0, 14)), ("BAS", "20", ("tx_bass", Command.TX_BASS, 0, 40)),
            ("TRB", "20", ("tx_treble", Command.TX_TREBLE, 0, 40)), ("SPLIT", "Off", "split"), ("A/B", "Frequency A", "vfo"),
            ("NB", "Off", ("noise_blanker", Command.NOISE_BLANKER, 0, 5)), ("NR", "On", ("noise_reduction", Command.NOISE_REDUCTION, 0, 5)),
            ("NBL", "7", ("noise_blanker_threshold", Command.NOISE_BLANKER_THRESHOLD, 0, 255)),
            ("PEAK", "15", ("peak_threshold", Command.PEAK_THRESHOLD, 0, 255)), ("ATU", "Off", "atu"), ("SPAN", "12 kHz", "span"),
            ("REF", "17", None), ("PWR", "Low", "tx_power"), ("TONE", "600 Hz", "tone"),
            ("SPEED", "26", ("cw_speed", Command.CW_SPEED, 5, 48)), ("DISP", "Display", None),
            ("RIT", "0", None), ("XIT", "0", None), ("LTIME", "100", ("cw_txrx_delay", Command.CW_TXRX_DELAY, 0, 255)),
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
        self.network_audio_status = QLabel("")
        self.network_audio_status.setStyleSheet("color: #8ba0ae; font: 13px Menlo")
        audio.addWidget(self.network_audio_status)
        self.rigctl_status = QLabel("rigctl: listening on 127.0.0.1:4532")
        self.rigctl_status.setStyleSheet("color: #8ba0ae; font: 13px Menlo")
        audio.addWidget(self.rigctl_status)
        audio.addStretch()
        return audio

    def _ptt_panel(self) -> QHBoxLayout:
        panel = QHBoxLayout()
        panel.addWidget(QLabel("PTT Microphone"))
        self.microphone = QComboBox()
        self.tx_output = QComboBox()
        self.refresh_transmit_devices()
        refresh = QPushButton("Refresh PTT Devices")
        refresh.clicked.connect(self.refresh_transmit_devices)
        self.ptt_button = QPushButton("Hold To Talk")
        self.ptt_button.setStyleSheet(
            "background: #132535; border: 1px solid #3689a3; border-radius: 10px; "
            "color: #dce5ed; font: 700 16px Menlo; padding: 10px 24px;"
        )
        self.ptt_button.pressed.connect(self.start_ptt)
        self.ptt_button.released.connect(self.stop_ptt)
        self.ptt_level = QLabel("MIC 0%  TX 0%")
        self.ptt_level.setStyleSheet("color: #8ba0ae; font: 13px Menlo")
        panel.addWidget(self.microphone)
        panel.addWidget(QLabel("USB TX device"))
        panel.addWidget(self.tx_output)
        panel.addWidget(refresh)
        panel.addWidget(self.ptt_button)
        panel.addWidget(self.ptt_level)
        panel.addStretch()
        return panel

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

    def refresh_transmit_devices(self) -> None:
        current_microphone = self.microphone.currentData() if hasattr(self, "microphone") else None
        current_output = self.tx_output.currentData() if hasattr(self, "tx_output") else None
        if not hasattr(self, "microphone"):
            return
        self.microphone.clear()
        self.tx_output.clear()
        for index, name in UsbAudioMonitor.microphone_devices():
            self.microphone.addItem(name, index)
        for index, name in UsbAudioMonitor.q900_output_devices():
            self.tx_output.addItem(name, index)
        if self.microphone.count() == 0:
            self.microphone.addItem("No computer microphone found", None)
        if self.tx_output.count() == 0:
            self.tx_output.addItem("No Q900 USB TX device found", None)
        if current_microphone is not None:
            self.microphone.setCurrentIndex(max(0, self.microphone.findData(current_microphone)))
        if current_output is not None:
            self.tx_output.setCurrentIndex(max(0, self.tx_output.findData(current_output)))

    def start_ptt(self) -> None:
        if self._sdr_active or self._sdr_switch_pending or self._sdr_restore_pending:
            self.status.setText("SDR transmit is unavailable until the network IQ TX format is validated.")
            return
        if self._ptt_source == "rigctl":
            self.status.setText("Rigctl virtual audio is transmitting.")
            return
        if not self.client.state.connected:
            self.status.setText("Connect the radio before using PTT.")
            return
        microphone = self.microphone.currentData()
        if microphone is None:
            self.status.setText("Select a computer microphone before using PTT.")
            return
        try:
            if self.client.state.transport == "USB":
                output = self.tx_output.currentData()
                if output is None:
                    self.status.setText("Select the Q900 USB transmit-audio device before using PTT.")
                    return
                self.tx_audio.start_usb(microphone, output)
            else:
                target = self.client.udp_target
                if target is None:
                    self.status.setText("No inbound radio address is available for network PTT audio.")
                    return
                if not self.network_audio.running:
                    self.status.setText("Start network receive audio before using network PTT.")
                    return
                self.tx_audio.start_udp(microphone, target, self.network_audio)
            # Audio must be established before the transmitter is keyed.
            self.client.set_ptt(True)
            self._ptt_source = "gui"
            if self.client.state.transport == "TCP":
                self.tx_audio.network_ptt_started()
            self._ptt_meter_timer.start()
            self.ptt_button.setText("TRANSMITTING")
            self.ptt_button.setStyleSheet(
                "background: #6b1e2b; border: 1px solid #ff667a; border-radius: 10px; "
                "color: white; font: 700 16px Menlo; padding: 10px 24px;"
            )
        except (ConnectionError, OSError, serial.SerialException, sd.PortAudioError) as error:
            self.tx_audio.stop()
            self.show_error(f"PTT: {error}")

    def stop_ptt(self) -> None:
        if self._ptt_source != "gui":
            return
        try:
            # Release first so no microphone data continues after unkeying.
            self.client.set_ptt(False)
        except (ConnectionError, OSError, serial.SerialException):
            pass
        if self.client.state.transport == "TCP":
            self._last_ptt_network_status = self.tx_audio.network_status
        self.tx_audio.stop()
        self._ptt_source = None
        self._ptt_meter_timer.stop()
        self.ptt_level.setText(f"MIC 0%  TX 0%  {self._last_ptt_network_status}")
        self.ptt_button.setText("Hold To Talk")
        self.ptt_button.setStyleSheet(
            "background: #132535; border: 1px solid #3689a3; border-radius: 10px; "
            "color: #dce5ed; font: 700 16px Menlo; padding: 10px 24px;"
        )

    def update_rigctl_status(self, count: int) -> None:
        if count:
            label = f"rigctl: {count} client{'s' if count != 1 else ''}, virtual audio ready"
            color = "#71db8d"
            self.start_virtual_receive_audio()
        else:
            label = "rigctl: listening on 127.0.0.1:4532"
            color = "#8ba0ae"
            if self._ptt_source == "rigctl":
                self.handle_rigctl_ptt(False)
            self.stop_virtual_receive_audio()
        self.rigctl_status.setText(label)
        self.rigctl_status.setStyleSheet(f"color: {color}; font: 13px Menlo")

    def start_virtual_receive_audio(self) -> None:
        if not self.client.state.connected or self._virtual_receive_active:
            return
        output = UsbAudioMonitor.named_device("Virtual Desktop Mic", "output")
        if output is None:
            self.rigctl_status.setText("rigctl: client connected, Virtual Desktop Mic unavailable")
            return
        try:
            # The standard receive monitor auto-starts on the selected local
            # speaker. A rigctl client explicitly takes ownership of RX and
            # redirects that stream into its virtual microphone device.
            self.audio.stop()
            self.network_audio.stop()
            self._network_audio_timer.stop()
            if self.client.state.transport == "TCP":
                self.network_audio.start(output)
                self._network_audio_timer.start()
            else:
                input_device = self.audio_input.currentData()
                if input_device is None:
                    return
                self.audio.start(input_device, output)
            self._virtual_receive_active = True
        except (OSError, sd.PortAudioError) as error:
            self.rigctl_status.setText(f"rigctl virtual RX: {error}")

    def stop_virtual_receive_audio(self) -> None:
        if not self._virtual_receive_active:
            return
        self.audio.stop()
        self.network_audio.stop()
        self._network_audio_timer.stop()
        self._virtual_receive_active = False
        if self.client.state.connected:
            QTimer.singleShot(0, self.start_audio_default)

    def handle_rigctl_ptt(self, active: bool) -> None:
        if active and (self._sdr_active or self._sdr_switch_pending or self._sdr_restore_pending):
            self.rigctl_status.setText("rigctl: SDR IQ transmit is not implemented")
            return
        if not active:
            if self._ptt_source != "rigctl":
                return
            try:
                self.client.set_ptt(False)
            except (ConnectionError, OSError, serial.SerialException):
                pass
            self.tx_audio.stop()
            self._ptt_source = None
            return
        if self._ptt_source:
            return
        if not self.client.state.connected:
            return
        microphone = UsbAudioMonitor.named_device("Virtual Desktop Speakers", "input")
        if microphone is None:
            self.rigctl_status.setText("rigctl: Virtual Desktop Speakers unavailable")
            return
        try:
            if self.client.state.transport == "TCP":
                target = self.client.udp_target
                if target is None:
                    return
                if not self.network_audio.running:
                    self.start_virtual_receive_audio()
                if not self.network_audio.running:
                    return
                self.tx_audio.start_udp(microphone, target, self.network_audio)
            else:
                output = self.tx_output.currentData()
                if output is None:
                    return
                self.tx_audio.start_usb(microphone, output)
            self.client.set_ptt(True)
            self._ptt_source = "rigctl"
            if self.client.state.transport == "TCP":
                self.tx_audio.network_ptt_started()
        except (ConnectionError, OSError, serial.SerialException, sd.PortAudioError) as error:
            self.tx_audio.stop()
            self.rigctl_status.setText(f"rigctl PTT: {error}")

    def update_ptt_meter(self) -> None:
        level = min(1.0, getattr(self.tx_audio, "level", 0.0))
        output_level = min(1.0, getattr(self.tx_audio, "output_level", 0.0))
        status = self.tx_audio.network_status if self.client.state.transport == "TCP" else ""
        self.ptt_level.setText(
            f"MIC {round(level * 100):d}%  TX {round(output_level * 100):d}%  {status}"
        )

    def toggle_audio(self) -> None:
        if self.audio.running or self.network_audio.running:
            self.audio.stop()
            self.network_audio.stop()
            self.audio_button.setText("Start Audio")
            self.status.setText("Receive audio monitor stopped.")
            return
        if self.client.state.transport == "TCP":
            output_device = self.audio_output.currentData()
            if output_device is None:
                self.status.setText("Select a local speaker output.")
                return
            try:
                self.network_audio.start(output_device)
                self.audio_button.setText("Stop Audio")
                self._network_audio_timer.start()
            except (OSError, sd.PortAudioError) as error:
                self.show_error(f"Network audio: {error}")
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
        """Start USB or network receive monitoring after a radio connects."""
        if self.audio_output.currentData() is None:
            return
        try:
            if self.client.state.transport == "TCP":
                self.network_audio.start(self.audio_output.currentData())
                self._network_audio_timer.start()
            elif self.audio_input.currentData() is not None:
                self.audio.start(self.audio_input.currentData(), self.audio_output.currentData())
            else:
                return
            self.audio_button.setText("Stop Audio")
        except (OSError, sd.PortAudioError) as error:
            self.status.setText(f"Receive audio not started: {error}")

    def show_audio_state(self, message: str) -> None:
        self.status.setText(message)

    def update_network_audio_status(self) -> None:
        if self.network_audio.running:
            self.network_audio_status.setText(self.network_audio.status)
        else:
            self._network_audio_timer.stop()
            self.network_audio_status.setText("")

    def toggle_sdr(self) -> None:
        if self._sdr_active:
            self.exit_sdr()
            return
        if self._sdr_switch_pending or self._sdr_restore_pending or not self.client.state.connected:
            return
        if self.client.state.transport != "TCP":
            self.status.setText("SDR network IQ requires the TCP network transport.")
            return
        if self._ptt_source:
            self.status.setText("Release PTT before entering SDR mode.")
            return
        if not self.network_audio.running:
            self.start_audio_default()
        if not self.network_audio.running:
            self.status.setText("Start network receive audio before entering SDR mode.")
            return
        try:
            # 0 selects the known normal stream; 1 requests the alternate IQ
            # stream. We only enable SDR after observing a 0x68 packet.
            self.client.set_stream_format(0)
            self.client.set_stream_format(1)
            self._sdr_switch_pending = True
            self.sdr_button.setText("SDR Starting")
            self._sdr_switch_timer.start(2000)
            self.poll_sdr_stream()
        except (ConnectionError, OSError) as error:
            self.show_error(f"SDR: {error}")

    def set_sdr_mode(self, mode: str) -> None:
        self.sdr_receiver.mode = mode
        self.spectrum.set_sdr(self._sdr_active, self.sdr_receiver.offset_hz, mode)

    def set_sdr_offset(self, offset_hz: int) -> None:
        self.sdr_receiver.offset_hz = offset_hz
        self.spectrum.set_sdr(self._sdr_active, offset_hz, self.sdr_receiver.mode)

    def poll_sdr_stream(self) -> None:
        if not self._sdr_switch_pending:
            return
        if self.network_audio.stream_type != 0x68:
            QTimer.singleShot(50, self.poll_sdr_stream)
            return
        self._sdr_switch_timer.stop()
        self._sdr_switch_pending = False
        self._sdr_active = True
        self.network_audio.set_iq_handler(self.sdr_receiver.feed)
        self.sdr_receiver.start()
        self.sdr_button.setText("SDR On")
        self.sdr_mode_selector.setVisible(True)
        self.sdr_offset.setVisible(True)
        self.spectrum.set_sdr(True, self.sdr_receiver.offset_hz, self.sdr_receiver.mode)
        self.status.setText("SDR RX active: 48 kHz network IQ at +12 kHz. TX unavailable.")

    def sdr_switch_timeout(self) -> None:
        if not self._sdr_switch_pending:
            return
        self._sdr_switch_pending = False
        self.sdr_button.setText("SDR Off")
        try:
            self.client.set_stream_format(0)
        except (ConnectionError, OSError):
            pass
        self.status.setText("SDR IQ stream was not detected; restored normal audio.")

    def exit_sdr(self) -> None:
        self._sdr_switch_timer.stop()
        self._sdr_switch_pending = False
        self._sdr_active = False
        self.network_audio.set_iq_handler(None)
        self.sdr_receiver.stop()
        self._sdr_restore_pending = True
        self._sdr_restore_attempts = 0
        self.sdr_button.setText("SDR Restoring")
        self.sdr_mode_selector.setVisible(False)
        self.sdr_offset.setVisible(False)
        self.spectrum.set_sdr(False, 0, self.sdr_receiver.mode)
        self.status.setText("SDR mode stopped; restoring normal network audio.")
        self.retry_normal_audio()

    def retry_normal_audio(self) -> None:
        if not self._sdr_restore_pending:
            return
        if self.network_audio.stream_type == 0x67:
            self._sdr_restore_pending = False
            self.sdr_button.setText("SDR Off")
            self.status.setText("Normal network audio restored.")
            return
        self._sdr_restore_attempts += 1
        try:
            self.client.set_stream_format(0)
        except (ConnectionError, OSError):
            self._sdr_restore_pending = False
            self.sdr_button.setText("SDR Off")
            return
        if self._sdr_restore_attempts >= 4:
            self._sdr_restore_pending = False
            self.sdr_button.setText("SDR Off")
            self.status.setText("Normal audio was requested but no 0x67 stream was observed.")
            return
        self._sdr_restore_timer.start(400)

    def toggle_connection(self) -> None:
        if self.client.state.connected or self.client.state.listening:
            if self._sdr_active or self._sdr_switch_pending:
                self.exit_sdr()
            self._sdr_restore_timer.stop()
            self._sdr_restore_pending = False
            self.client.disconnect()
            self.network_audio.stop()
            self._network_audio_timer.stop()
            self.network_audio_status.setText("")
            self.audio_button.setText("Start Audio")
        elif self.transport.currentIndex() == 1:
            port = self.serial_port.currentData()
            if not port:
                self.status.setText("No USB serial device is available. Connect the radio and refresh the list.")
                return
            self.status.setText(f"Opening USB serial port {port} at 115200 baud...")
            threading.Thread(target=self.client.connect_usb, args=(port,), daemon=True).start()
        else:
            output_device = self.audio_output.currentData()
            if output_device is None:
                self.status.setText("Select a local speaker output before starting the TCP listener.")
                return
            try:
                # The radio may begin sending UDP as soon as its TCP session
                # completes. Bind the media port before accepting that session.
                self.network_audio.start(output_device)
                self._network_audio_timer.start()
                self.audio_button.setText("Stop Audio")
            except (OSError, sd.PortAudioError) as error:
                self.show_error(f"Network audio: {error}")
                return
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
                dialog = QDialog(self)
                dialog.setWindowTitle(field.replace("_", " ").title())
                layout = QVBoxLayout(dialog)
                value_label = QLabel(str(current))
                value_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
                value_label.setStyleSheet("color: #50d9e8; font: 700 24px Menlo")
                slider = QSlider(Qt.Orientation.Horizontal)
                slider.setRange(minimum, maximum)
                slider.setValue(current)
                slider.valueChanged.connect(lambda value: value_label.setText(str(value)))
                buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Cancel | QDialogButtonBox.StandardButton.Ok)
                buttons.accepted.connect(dialog.accept)
                buttons.rejected.connect(dialog.reject)
                layout.addWidget(value_label)
                layout.addWidget(slider)
                layout.addWidget(buttons)
                if dialog.exec() == QDialog.DialogCode.Accepted:
                    self.client.set_value(field, command, slider.value())
            elif action == "power":
                message = "Put the radio into standby? CAT control will be unavailable until it is woken locally or reconnected."
                choice, accepted = QInputDialog.getItem(self, "Radio Power", message, ("Wake", "Standby"), 0, False)
                if not accepted:
                    return
                if choice == "Standby":
                    self.client.send(encode_frame(Command.POWER, b"\x00"))
                    self.tiles["POWER"].set_value("Standby")
                else:
                    self.client.send(encode_frame(Command.POWER, b"\x01"))
                    self.tiles["POWER"].set_value("Wake")
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
        self.tiles["NB"].set_value(str(state.noise_blanker))
        self.tiles["NR"].set_value(str(state.noise_reduction))
        self.tiles["SPLIT"].set_value("On" if state.split else "Off")
        self.tiles["A/B"].set_value("Frequency B" if state.active_vfo_b else "Frequency A")
        self.tiles["ATU"].set_value(("Off", "On", "Scan")[state.atu])
        self.tiles["SPAN"].set_value(f"{SPAN_HZ[state.span_index] / 1000:g} kHz")
        self.tiles["PWR"].set_value("High" if state.tx_power_high else "Low")
        self.tiles["TONE"].set_value(f"{state.cw_sidetone_hz} Hz")
        self.tiles["SPEED"].set_value(str(state.cw_speed))
        self.tiles["HVOL"].set_value(str(state.headphone_volume))
        self.tiles["MIC"].set_value(str(state.mic_gain))
        self.tiles["CMP"].set_value(str(state.compressor))
        self.tiles["BAS"].set_value(str(state.tx_bass))
        self.tiles["TRB"].set_value(str(state.tx_treble))
        self.tiles["NBL"].set_value(str(state.noise_blanker_threshold))
        self.tiles["PEAK"].set_value(str(state.peak_threshold))
        self.tiles["LTIME"].set_value(str(state.cw_txrx_delay))
        self.spectrum.set_state(state)
        if state.connected and state.transport == "USB" and not self.audio.running:
            QTimer.singleShot(0, self.start_audio_default)
        if state.connected and state.transport == "TCP" and not self.network_audio.running:
            QTimer.singleShot(0, self.start_audio_default)
        # Keep UDP/8000 bound while the TCP listener waits for a radio. Some
        # firmware starts media before the first status frame reaches the UI.
        if not state.connected and not state.listening and self.network_audio.running:
            self.network_audio.stop()
            self._network_audio_timer.stop()
            self.network_audio_status.setText("")
            self.audio_button.setText("Start Audio")

    def show_error(self, message: str) -> None:
        self.status.setText(f"Connection error: {message}")

    def closeEvent(self, event) -> None:  # type: ignore[no-untyped-def]
        self.rigctl.stop()
        if self._sdr_active or self._sdr_switch_pending:
            self.exit_sdr()
        self._sdr_restore_timer.stop()
        self._sdr_restore_pending = False
        self.stop_ptt()
        self.audio.stop()
        self.network_audio.stop()
        self.client.disconnect()
        event.accept()


def self_test() -> None:
    assert crc16_ccitt(bytes.fromhex("0339")) == 0xEF26
    assert encode_frame(Command.STATUS).hex() == "a5a5a5a5030bf937"
    assert encode_frame(Command.PTT, b"\x00").hex() == "a5a5a5a504070089cb"
    assert encode_frame(Command.PTT, b"\x01").hex() == "a5a5a5a504070199ea"
    captured_audio = bytes.fromhex("a5a5a5a56721002c00") + bytes(192)
    assert captured_audio.startswith(b"\xa5\xa5\xa5\xa5\x67\x21\x00")
    assert len(captured_audio[9:]) == 192
    raw_tx = np.zeros(96, dtype="<i2").tobytes()
    assert len(raw_tx) == 192
    mono_tx = np.arange(96, dtype="<i2")
    stereo_tx = np.repeat(mono_tx, 2)
    assert len(stereo_tx.tobytes()) == 384
    assert np.array_equal(stereo_tx[0::2], stereo_tx[1::2])
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
    # Let Ctrl+C exit through Qt so it cannot interrupt a paint callback and
    # leave the audio/network worker cleanup unfinished.
    signal.signal(signal.SIGINT, lambda _signal, _frame: app.quit())
    signal_timer = QTimer()
    signal_timer.start(100)
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
