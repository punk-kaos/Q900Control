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
import os
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
    QCheckBox,
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
# Spectrum frames arrive at roughly 8 Hz and every arrival used to force a
# repaint. Coalesce them: the GUI process shares its GIL with the microphone
# callback, so paint cost is transmit audio quality.
SPECTRUM_MAX_REPAINT_HZ = 15
SPAN_HZ = (48_000, 24_000, 12_000, 6_000, 3_000, 1_500)
# Measuring the radio's media clock needs an uninterrupted run of packets.
# Within a run the rate is packets divided by the elapsed time between its first
# and last arrival, so arrival jitter only enters through the two endpoints and
# contributes jitter/window: a few ppm over tens of seconds. Anything that would
# corrupt that -- a scheduling stall, a pause while transmitting, or a reordered
# datagram -- ends the run instead of being averaged into it, because a single
# 30 ms stall inside a 20 s window is a 1500 ppm error, the same order as the
# crystal offset being measured.
# A read later than this is reported as a stall. It does not end the run: a late
# read moves an endpoint without changing the packet count, so count-over-span
# remains unbiased, whereas restarting on every late read never accumulates a
# usable window at all.
CLOCK_STALL_NS = 4_000_000
# Longer than this is a real pause in the stream rather than a late read, and the
# radio genuinely stops producing audio during one.
CLOCK_RUN_GAP_NS = 50_000_000
CLOCK_MIN_RUN_PACKETS = 5_000
# Set Q900_RX_RECORD to a path prefix to log the arrival pattern of the radio's
# media stream: one 12-byte record per packet holding an 8-byte little-endian
# monotonic nanosecond stamp, a 2-byte payload length and a 2-byte stream type.
# Analyse with `--analyze-rx <prefix>`. This distinguishes a radio that sends in
# bursts from a receive thread that is being starved, which need opposite fixes.
RX_RECORD_PREFIX = os.environ.get("Q900_RX_RECORD") or None
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
        # These mirror the whole 48 kHz stream about its own DC, not about the
        # tuned carrier, so they detune rather than swap sidebands. Use the
        # offset control to retune and the mode selector to pick a sideband.
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
        ssb_history = np.zeros(_HILBERT_LEN - 1, dtype=np.complex128)
        ssb_taps = np.ones(9, dtype=np.float32) / 9
        ssb_smooth_history = np.zeros(len(ssb_taps) - 1, dtype=np.float64)
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
                # USB and LSB share the same suppressed-carrier frequency, so
                # both translate the selected carrier to zero. They differ only
                # in which side of zero carries the wanted audio. Taking
                # baseband.real is a product detector: it folds both sides
                # together, so it has no opposite-sideband rejection and the
                # mode selector has no audible effect.
                #
                # Use a phasing detector instead. With H the Hilbert transform
                # (H(w) = -j*sgn(w), realised by _HILBERT_TAPS), I - H{Q} keeps
                # only positive baseband frequencies and I + H{Q} keeps only
                # negative ones.
                shift = np.exp(-1j * 2 * np.pi * self.offset_hz * index / self.SAMPLE_RATE)
                baseband = signal * shift
                combined = np.concatenate((ssb_history, baseband))
                ssb_history = combined[-(_HILBERT_LEN - 1):]
                quadrature = np.convolve(combined.imag, _HILBERT_TAPS, mode="valid")
                in_phase = combined.real[_HILBERT_DELAY : _HILBERT_DELAY + count]
                if self.mode == "LSB":
                    audio = in_phase + quadrature
                else:
                    audio = in_phase - quadrature
                # Smooth with carried state. A mode="same" convolution per block
                # zero-pads both edges, which puts a discontinuity at every
                # block boundary and buzzes at the block rate.
                smoothing_input = np.concatenate((ssb_smooth_history, audio))
                ssb_smooth_history = smoothing_input[-(len(ssb_taps) - 1):]
                audio = np.convolve(smoothing_input, ssb_taps, mode="valid")
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
        self._underflows = 0
        # The radio's media stream is clocked by its own crystal, and neither this
        # application nor the radio's UHSDR firmware rate-matches the two ends.
        # The arrival rate of its packets therefore measures that clock, which is
        # the rate transmit audio has to be delivered at.
        #
        # Measure it over the current uninterrupted run, not over the whole
        # session. The stream stops while transmitting and does not begin the
        # instant the socket opens, and averaging across a dead interval yields a
        # figure that climbs towards the true rate forever without reaching it.
        self._clock_run_start_ns = 0
        self._clock_run_last_ns = 0
        self._clock_run_packets = 0
        self._clock_outliers = 0
        self._clock_gaps = 0
        self._clock_best_rate = 0.0
        self._clock_best_seconds = 0.0
        self._clock_align_first_ns = 0
        self._clock_align_first_index = 0
        self._clock_align_last_ns = 0
        self._clock_align_last_index = 0

    def start(self, output_device: int, port: int = 8000) -> None:
        self.stop()
        output_info = sd.query_devices(output_device, "output")
        output_channels = min(2, output_info["max_output_channels"])
        max_queued_frames = self.SAMPLE_RATE * 2

        def output_callback(outdata, frames, timing, status):  # type: ignore[no-untyped-def]
            if status.output_underflow:
                # Playback ran dry: audible as a click, and a symptom of this
                # process being too busy to service the audio device in time.
                self._underflows += 1
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
            arrival_log = None
            if RX_RECORD_PREFIX:
                try:
                    arrival_log = open(f"{RX_RECORD_PREFIX}.rx.time", "wb")
                except OSError:
                    arrival_log = None
            try:
                receive_packets(arrival_log)
            finally:
                if arrival_log is not None:
                    arrival_log.close()

        def receive_packets(arrival_log) -> None:  # type: ignore[no-untyped-def]
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
                    arrived_ns = time.monotonic_ns()
                    self._note_arrival(arrived_ns)
                    if arrival_log is not None:
                        arrival_log.write(
                            arrived_ns.to_bytes(8, "little")
                            + min(len(packet), 0xFFFF).to_bytes(2, "little")
                            + packet_type.to_bytes(2, "little")
                        )
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
            self._clock_run_start_ns = 0
            self._clock_run_last_ns = 0
            self._clock_run_packets = 0
            self._clock_outliers = 0
            self._clock_gaps = 0
            self._clock_best_rate = 0.0
            self._clock_best_seconds = 0.0
            self._clock_align_first_ns = 0
            self._clock_align_first_index = 0
            self._clock_align_last_ns = 0
            self._clock_align_last_index = 0
        self._stream_type = 0
        self._underflows = 0

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

    def _note_arrival(self, now_ns: int) -> None:
        """Track the radio's media clock. Caller must hold _stats_lock.

        The rate is packets over the span between the first and last arrival of
        the current run. A delayed read shifts an endpoint but does not change
        the packet total, so that ratio stays unbiased as the window grows even
        when reads are frequently late; breaking the run on every late read
        instead throws away the measurement and never converges. Only a real
        pause in the stream starts a new run, because during a pause the radio
        genuinely stops producing audio.
        """
        if not self._clock_run_last_ns:
            self._clock_run_start_ns = now_ns
            self._clock_run_last_ns = now_ns
            return
        delta = now_ns - self._clock_run_last_ns
        if delta > CLOCK_RUN_GAP_NS:
            self._clock_gaps += 1
            rate, seconds, packets = self._clock_run()
            if packets >= CLOCK_MIN_RUN_PACKETS and seconds > self._clock_best_seconds:
                self._clock_best_rate = rate
                self._clock_best_seconds = seconds
            self._clock_run_start_ns = now_ns
            self._clock_run_last_ns = now_ns
            self._clock_run_packets = 0
            self._clock_align_first_ns = 0
            self._clock_align_first_index = 0
            self._clock_align_last_ns = 0
            self._clock_align_last_index = 0
            return
        self._clock_run_packets += 1
        if delta > CLOCK_STALL_NS:
            # A late read, or the boundary between groups if the radio sends its
            # media in bursts. Record it as an alignment point: measuring between
            # two boundaries makes the span cover a whole number of groups, where
            # endpoints falling mid-group would understate it by up to one group
            # and bias the rate by group duration over window.
            self._clock_outliers += 1
            if not self._clock_align_first_ns:
                self._clock_align_first_ns = now_ns
                self._clock_align_first_index = self._clock_run_packets
            self._clock_align_last_ns = now_ns
            self._clock_align_last_index = self._clock_run_packets
        if now_ns > self._clock_run_last_ns:
            self._clock_run_last_ns = now_ns

    def _clock_run(self) -> tuple[float, float, int]:
        """Return (packets_per_second, run_seconds, packets) for the current run.

        Caller must hold _stats_lock. Returns zeros until the run is long enough
        for the figure to mean anything.
        """
        packets = self._clock_align_last_index - self._clock_align_first_index
        span_ns = self._clock_align_last_ns - self._clock_align_first_ns
        if packets < CLOCK_MIN_RUN_PACKETS or span_ns <= 0:
            # No usable pair of boundaries, so the stream is smoothly paced and
            # the first and last arrival are themselves aligned.
            packets = self._clock_run_packets
            span_ns = self._clock_run_last_ns - self._clock_run_start_ns
        seconds = span_ns / 1e9
        if packets >= CLOCK_MIN_RUN_PACKETS and seconds > 0:
            if seconds >= self._clock_best_seconds:
                return packets / seconds, seconds, self._clock_run_packets
        if self._clock_best_seconds > 0:
            return self._clock_best_rate, self._clock_best_seconds, self._clock_run_packets
        return 0.0, max(0.0, seconds), self._clock_run_packets

    @property
    def measured_packet_rate(self) -> float:
        """Packets per second observed from the radio, or 0.0 if not yet known.

        This is the radio's own media clock. Transmit audio has to be delivered
        at this rate, not at the host's nominal rate: neither this application
        nor the radio's UHSDR firmware rate-matches the two ends, so any
        difference accumulates in the radio's ring until it slips.
        """
        with self._stats_lock:
            return self._clock_run()[0]

    @property
    def status(self) -> str:
        with self._stats_lock:
            if not self._packet_count:
                return "UDP waiting"
            underflows = f"  drops {self._underflows}" if self._underflows else ""
            gaps = f"  breaks {self._clock_gaps}" if self._clock_gaps else ""
            if self._clock_outliers:
                gaps += f"  stalls {self._clock_outliers}"
            rate, seconds, run_packets = self._clock_run()
            if rate:
                # Report the implied sample rate as well. It validates the
                # assumption that a 192-byte payload is one millisecond of
                # 48 kHz stereo: a wildly different figure means the assumed
                # cadence, not the crystal, is wrong.
                frames = NETWORK_TX_PACKET_BYTES // 4
                nominal = 1.0 / NETWORK_TX_PERIOD
                clock = (
                    f"  radio {rate:.2f} pkt/s = {rate * frames:.0f} Hz "
                    f"({(rate / nominal - 1.0) * 1e6:+.0f} ppm) over {seconds:.0f}s"
                )
            else:
                clock = f"  radio clock: {run_packets}/{CLOCK_MIN_RUN_PACKETS} pkts"
            return (
                f"UDP {self._packet_count} pkts  {self._last_packet_size} B  "
                f"{self._format}{clock}{gaps}{underflows}"
            )


# One network TX datagram is the radio's native media quantum: 48 interleaved
# stereo frames, 96 signed 16-bit words, 192 bytes, 1 ms at 48 kHz. That is the
# payload size the radio itself sends on RX and the size the working I/Q sender
# uses. Every other quantity on this path is expressed in whole packets so the
# parent and the sender process cannot disagree about the unit again.
#
# The firmware's UDP receive callback (0x0806C8A8) pushes every word of a
# datagram into the ring, so a larger datagram is not truncated as previously
# recorded here. It is still the wrong choice: the ring's rate corrector runs
# once per datagram (0x0806C80C), so a longer datagram buys proportionally less
# correction authority and interacts more coarsely with the radio's 64-word
# consumer block. The only hard limits are a 2560-byte cap on what the callback
# will stage, and a payload that must be a whole number of stereo frames -- the
# word count is derived as `bytes >> 1`, so any length that is not a multiple of
# 4 permanently shifts the ring's L/R parity.
NETWORK_TX_PERIOD = 0.001
NETWORK_TX_PACKET_BYTES = 48 * 2 * 2
NETWORK_TX_MAX_DATAGRAM_BYTES = 2560
# The radio's transmit ring, transcribed from the firmware so the host's choices
# below can be derived rather than guessed. Depth is measured in int16 words and
# one stereo frame is two words.
#   0x0806C7DC  depth = (write - read) mod 6144
#   0x0806C83A  depth < 1536 -> duplicate the datagram's last frame
#   0x0806C846  depth > 4608 -> drop it
#   0x0806C95A  consumer takes 64 words per DSP block, 1500 blocks/s
# A correction is applied once per datagram, so at 1 ms pacing leaving the
# 1536..4608 window means a duplicated or dropped frame a thousand times a
# second. That is what roughness on this path sounds like.
RADIO_RING_WORDS = 6144
RADIO_RING_SHALLOW_WORDS = 1536
RADIO_RING_DEEP_WORDS = 4608
RADIO_CONSUME_WORDS_PER_S = 96_000
RADIO_RING_TARGET_WORDS = (RADIO_RING_SHALLOW_WORDS + RADIO_RING_DEEP_WORDS) // 2

NETWORK_TX_PREROLL_PACKETS = 80      # buffered before the transmitter is keyed
# Cushion held inside the sender process, and the depth the startup trim leaves
# after priming. It has to cover more than one 20 ms microphone callback or the
# buffer bottoms out every mic period and any late block becomes an audible gap.
NETWORK_TX_LOW_WATER_PACKETS = 60
NETWORK_TX_HIGH_WATER_PACKETS = 200  # hard cap on buffered capture
NETWORK_TX_MAX_CATCHUP_PACKETS = 8
# Ceiling on schedule debt carried forward after a resync. Debt has to be repaid
# or the long-run send rate falls below the radio's consume rate and its ring
# walks down into the duplication region. The useful bound is how far the ring
# can drain before that happens.
NETWORK_TX_MAX_DEBT_PACKETS = 16
# The radio consumes its TX ring only while PTT is asserted -- 0x0803432C tests
# state[0xAF] and runs either the receive path or the transmit path, never both
# -- and nothing in the firmware ever resets the ring indices. So whatever depth
# was left at the previous unkey is still sitting there at the next key-up, and
# priming on top of it accumulates: within a few transmissions the depth passes
# 4608 words and the firmware drops a frame from every datagram, then passes
# 6143 and overflows, which advances the read index by a single word and
# permanently breaks its 64-word alignment. Once misaligned, peek() straddles the
# end of the ring -- it has no wrap handling -- and reads out of bounds.
#
# There is no way to flush it from the host: datagrams sent while unkeyed are
# discarded by the PTT gate, and no CAT command reports or clears the ring. The
# only mechanism is the consumer itself, so key the transmitter and send nothing
# until it has drained. Allow a margin over a completely full ring so the
# starting depth is deterministic regardless of how the last transmission ended.
NETWORK_TX_RING_DRAIN = 1.1 * RADIO_RING_WORDS / RADIO_CONSUME_WORDS_PER_S
# Minimum spacing inside a burst. Bursting at line rate asks the radio's Ethernet
# and lwIP receive path to absorb a thousand times its steady-state packet rate,
# and a datagram lost there is a millisecond of audio missing from the ring with
# nothing to resend it. Four times real time empties a backlog quickly while
# staying two orders of magnitude below line rate.
NETWORK_TX_BURST_GAP = NETWORK_TX_PERIOD / 4
# Priming aims for the centre of the corrector's window so drift in either
# direction has the most room. The consumer keeps running while the burst is
# being paced out, so a primed packet does not net its whole 96 words: it nets
# 96 less whatever is consumed during its own slot. Solving for the target depth
# is why this is derived rather than written down -- at 20 packets the ring
# settled just above the duplication threshold, which is where any downward
# drift immediately became audible.
NETWORK_TX_PRIME_PACKETS = round(
    RADIO_RING_TARGET_WORDS
    / (NETWORK_TX_PACKET_BYTES // 2 - NETWORK_TX_BURST_GAP * RADIO_CONSUME_WORDS_PER_S)
)
# Rate-conversion servo. The ratio starts from the radio's measured clock, so the
# servo only has to absorb the host audio clock's own error, and both gains are
# deliberately gentle: the buffer holds tens of milliseconds, so there is no need
# to correct quickly and every reason not to modulate the audio while doing it.
RESAMPLE_KP = 0.002
RESAMPLE_KI = 1.0e-7
RESAMPLE_TRIM_LIMIT = 0.005
# Two-pole smoothing of the depth error, in packets, so roughly two seconds.
#
# Without it the proportional path put 229 ppm rms of ratio wobble into the 2 to
# 200 Hz band, which frequency-modulates the audio: measured as a sideband family
# a few Hz either side of a transmitted tone at -28 dB, and heard as roughness.
# The cause is that capture arrives in 20 ms blocks, so the buffer depth is a
# sawtooth of one whole block, and KP converted that granularity directly into
# rate. The loop itself has a natural frequency near 0.006 Hz, so a two second
# filter is two orders of magnitude faster than anything the servo needs to do
# and costs it nothing. Slow drift below 2 Hz is left alone: it is the servo
# working, and at these amplitudes it is inaudible pitch wander rather than
# roughness.
RESAMPLE_SMOOTH_PACKETS = 2000
# Rate conversion filter. Linear interpolation was measured putting a spurious
# sideband comb on transmitted audio at -43 dB, spaced at the rate its fractional
# phase wraps -- 23.66 Hz for the 493 ppm offset seen on this radio. That is
# plainly audible as roughness, it is present on a steady tone, and it is the one
# thing the USB transmit path does not do, which is why USB sounded clean while
# the network path did not.
#
# The cause is that a two-tap interpolator's response depends on the fractional
# phase: at phase 0 it is a passthrough, at phase 0.5 it is a mild lowpass. With
# the phase walking continuously that difference becomes amplitude and spectral
# modulation at the wrap rate. A windowed-sinc bank has essentially the same
# response at every phase, so there is nothing left to modulate.
RESAMPLE_TAPS = 24
RESAMPLE_PHASES = 512
# Cutoff at Nyquist rather than below it. That makes the zero-phase row an exact
# delta, because sinc() lands on a zero at every other integer tap, so a ratio of
# exactly one stays bit-identical and enabling conversion cannot touch a correctly
# clocked link. Backing the cutoff off to 0.88 measured no better on spurs and
# cost that property, since the row became a mild lowpass instead.
RESAMPLE_CUTOFF = 1.0
RESAMPLE_KAISER_BETA = 9.0


def _resample_bank(
    taps: int = RESAMPLE_TAPS,
    phases: int = RESAMPLE_PHASES,
    cutoff: float = RESAMPLE_CUTOFF,
    beta: float = RESAMPLE_KAISER_BETA,
) -> np.ndarray:
    """Fractional-delay FIR bank, shape (phases, taps).

    Row p is the filter for an output instant p/phases of a sample after the
    integer input index. Each row is normalised to unity DC gain so no phase can
    have a different gain from any other: that equality is the whole point, since
    a gain that varies with phase is exactly what modulates the audio.
    """
    half = taps // 2
    # Distance from every tap to the output instant, per phase.
    offset = np.arange(taps)[None, :] - (half - 1)
    frac = np.arange(phases)[:, None] / phases
    distance = offset - frac
    kernel = np.sinc(cutoff * distance) * cutoff
    # Kaiser window as a function of that distance rather than of the tap index,
    # so the window stays centred on the output instant as the phase moves.
    shape = np.clip(distance / half, -1.0, 1.0)
    kernel = kernel * (
        np.i0(beta * np.sqrt(np.maximum(0.0, 1.0 - shape * shape))) / np.i0(beta)
    )
    kernel /= kernel.sum(axis=1, keepdims=True)
    return kernel.astype(np.float32)


_RESAMPLE_BANK = _resample_bank()
# Input frames the filter reaches back before the output instant. The caller's
# buffer keeps this many already-consumed frames in front of the next output
# position, which happens naturally because only the integer advance is deleted.
_RESAMPLE_HISTORY = RESAMPLE_TAPS // 2

# Upper bound on the pre-key wait for the sender process to report ready. It
# covers interpreter spawn and module import, not audio latency.
NETWORK_TX_READY_TIMEOUT = 3.0
# Set Q900_TX_RECORD to a path prefix to capture exactly what leaves the host.
# The sender writes every transmitted payload to <prefix>.tx.raw (48 kHz stereo
# S16LE) and one 8-byte little-endian nanosecond send timestamp per packet to
# <prefix>.tx.time. Analyse with `--analyze-tx <prefix>`. This distinguishes a
# host-side defect from a radio-side or network-side one: if the recording is
# clean, nothing above the socket is responsible.
TX_RECORD_PREFIX = os.environ.get("Q900_TX_RECORD") or None

# Transmit gain staging. The radio does not scale network audio to suit itself:
# for stream format 1 the conversion at 0x080397BC multiplies by exactly 2**-16,
# which cancels the ring consumer's `<< 16` and leaves the DSP working with the
# raw int16 value. Unity. What follows is the radio's own TX gain chain
# (0x08039898): a pre-gain of `0.5 + 0.5 * state[0x1A0]`, then a per-sample ALC
# whose knee is 30000.
#
# state[0x1A0] is selected by CAT 0x10 (COMPRESSOR): the handler at 0x080585CC
# stores `state[0x140] = payload - 1` and looks the gain up in the table at
# 0x080DAF14. So COMPRESSOR is not a ratio, it is a pre-ALC gain of up to 13x.
# The chain is calibrated for the codec's microphone input, which peaks well
# below full scale; sending int16 full scale instead drove the default setting
# (9 => 8.00x) to 262136 against a 30000 knee, 18.8 dB into the limiter, and the
# ALC then held ~19 dB of gain reduction and modulated it at audio rate. That is
# what made network transmit audio rough from the first moment of transmission.
#
# Indexed by the CAT 0x10 payload, 0..14. Payload 0 leaves state[0x140] negative
# and 0x080398A4 then bypasses pre-gain and ALC together, so unity with no
# limiter. Payload 14 selects a runtime value we cannot read, so assume the
# worst case rather than overdriving.
TX_PREGAIN_BY_COMPRESSOR = (
    1.00, 1.00, 1.50, 2.50, 3.50, 4.00, 4.50, 5.50,
    6.50, 8.00, 9.00, 10.50, 13.00, 13.00, 13.00,
)
TX_ALC_THRESHOLD = 30000.0
# Sit just under the knee. At 1.0 the loudest sample lands exactly on it; less
# than 1.0 keeps the path linear and predictable, which is what we want while
# the host has no compressor of its own. Raising this toward and past 1.0 is how
# you hand dynamics back to the radio.
TX_LEVEL_MARGIN = 0.97


def network_tx_ceiling(compressor: int) -> int:
    """Peak int16 magnitude to send for a given radio COMPRESSOR setting.

    Chosen so `peak * pregain` lands just below the radio's 30000 ALC knee, which
    is where the microphone input sits and therefore what the whole TX chain is
    calibrated for. The radio never reports this setting back, so the caller is
    passing the host's own record of it.
    """
    index = min(max(int(compressor), 0), len(TX_PREGAIN_BY_COMPRESSOR) - 1)
    ceiling = TX_ALC_THRESHOLD * TX_LEVEL_MARGIN / TX_PREGAIN_BY_COMPRESSOR[index]
    return int(min(ceiling, 32767.0))


def tx_pacing(radio_rate: float) -> tuple[float, float]:
    """Return (send period, conversion ratio) for a measured radio packet rate.

    Emitting at the radio's clock and converting the host stream to it is the
    only arrangement that leaves neither end's buffer drifting. A rate of zero
    means the clock has not been measured yet, so fall back to nominal and
    convert nothing.
    """
    if radio_rate <= 0.0:
        return NETWORK_TX_PERIOD, 1.0
    return 1.0 / radio_rate, (1.0 / NETWORK_TX_PERIOD) / radio_rate


def resample_ratio(
    depth_frames: int,
    target_frames: int,
    trim: float,
    base_ratio: float,
    smooth: tuple[float, float] = (0.0, 0.0),
) -> tuple[float, float, tuple[float, float]]:
    """Return (ratio, new trim, new smoothing state) holding the buffer at target.

    The base ratio comes from the radio's measured clock, so this only has to
    absorb the host audio clock's own error. A deeper buffer than wanted must
    consume faster, hence a larger ratio.

    The error is smoothed before it reaches either gain. Capture arrives in 20 ms
    blocks, so the raw depth is a sawtooth a whole block deep, and feeding that
    to a proportional term modulates the conversion ratio at the block rate --
    which is frequency modulation of the transmitted audio, not rate control.
    See RESAMPLE_SMOOTH_PACKETS.
    """
    error = (depth_frames - target_frames) / target_frames if target_frames else 0.0
    alpha = 1.0 / RESAMPLE_SMOOTH_PACKETS
    first = smooth[0] + alpha * (error - smooth[0])
    second = smooth[1] + alpha * (first - smooth[1])
    trim = min(
        max(trim + RESAMPLE_KI * second, -RESAMPLE_TRIM_LIMIT), RESAMPLE_TRIM_LIMIT
    )
    return base_ratio * (1.0 + RESAMPLE_KP * second + trim), trim, (first, second)


def resample_stereo(
    pending: bytearray, frames_out: int, ratio: float, phase: float
) -> tuple[bytes, float] | None:
    """Emit `frames_out` interleaved stereo S16LE frames from `pending`.

    Consumes `ratio` input frames per output frame using linear interpolation,
    deletes what it consumed, and returns the payload with the new fractional
    phase. Returns None if `pending` does not yet hold enough input.

    This exists because three clocks are involved and only two can be matched by
    pacing. The host produces audio on its own audio clock, the radio consumes on
    its crystal, and the two differ by hundreds of ppm. Sending at the host rate
    makes the radio's ring overflow; sending at the radio rate makes the host
    buffer overflow. Either way a whole millisecond of audio is eventually
    discarded, which reaches the air as a broadband click. Converting the rate
    spreads that difference across every sample instead.

    The conversion is a polyphase windowed-sinc bank, not linear interpolation:
    see RESAMPLE_TAPS. Both channels are filtered independently so this stays
    correct for the I/Q path, where the two words are not copies of each other.

    The first _RESAMPLE_HISTORY frames of a fresh buffer serve only as filter
    history and are never output, which costs a quarter of a millisecond once.
    """
    if frames_out < 1 or ratio <= 0.0:
        return None
    history = _RESAMPLE_HISTORY
    reach = RESAMPLE_TAPS - history
    last_position = history + phase + ratio * (frames_out - 1)
    required = int(last_position) + reach + 1
    if len(pending) < required * 4:
        return None
    data = np.frombuffer(bytes(pending[: required * 4]), dtype="<i2").reshape(-1, 2)
    position = history + phase + ratio * np.arange(frames_out)
    index = position.astype(np.int64)
    row = np.minimum(
        ((position - index) * RESAMPLE_PHASES).astype(np.int64), RESAMPLE_PHASES - 1
    )
    kernel = _RESAMPLE_BANK[row]
    window = data[index[:, None] + (np.arange(RESAMPLE_TAPS) - (history - 1))[None, :]]
    frames = np.einsum("ft,ftc->fc", kernel, window.astype(np.float32))
    payload = np.clip(np.rint(frames), -32768, 32767).astype("<i2").tobytes()
    advance = phase + ratio * frames_out
    consumed = int(advance)
    del pending[: consumed * 4]
    return payload, advance - consumed


class DcBlocker:
    """Single-pole DC blocker: y[n] = x[n] - x[n-1] + a*y[n-1].

    The firmware skips one of its TX filter stages when the stream format is 1
    (0x08039540 returns early), so DC and subsonic energy from the capture device
    reach the SSB modulator unfiltered. There DC becomes carrier leak and rumble
    burns headroom in the radio's ALC.

    Evaluated in closed form rather than sample by sample, because a Python loop
    over 960 samples in the microphone callback would hold the GIL for longer
    than the DSP is worth. The recursion y[n] = a*y[n-1] + d[n] has the solution
    y[n] = a**n * (a*y0 + cumsum(d * a**-n)[n]); with `a` this close to 1 the
    a**-n term only reaches ~12 across a block, so float64 carries it exactly.

    The corner is deliberately far below the voice band: at 300 Hz a 20 Hz
    single pole costs under 0.01 dB, so this cannot be blamed for thin audio.
    """

    def __init__(self, cutoff_hz: float = 20.0, sample_rate: int = 48_000) -> None:
        self._a = float(np.exp(-2.0 * np.pi * cutoff_hz / sample_rate))
        self._last_input = 0.0
        self._last_output = 0.0

    def reset(self) -> None:
        self._last_input = 0.0
        self._last_output = 0.0

    def process(self, block: np.ndarray) -> np.ndarray:
        samples = np.asarray(block, dtype=np.float64)
        if samples.size == 0:
            return samples.astype(np.float32)
        a = self._a
        diff = np.empty_like(samples)
        diff[0] = samples[0] - self._last_input
        np.subtract(samples[1:], samples[:-1], out=diff[1:])
        decay = a ** np.arange(samples.size, dtype=np.float64)
        out = decay * (a * self._last_output + np.cumsum(diff / decay))
        self._last_input = float(samples[-1])
        self._last_output = float(out[-1])
        return out.astype(np.float32)


def quantize_tx(mono: np.ndarray, ceiling: int) -> np.ndarray:
    """Scale float mono audio to int16 at `ceiling`, rounding rather than truncating.

    `ceiling` is the radio's expected peak, not int16 full scale: see
    network_tx_ceiling(). Rounding matters because the previous
    `(pcm * 32767).astype()` truncated toward zero on every sample, which is
    undithered and correlated with the signal.
    """
    scaled = np.asarray(mono, dtype=np.float32) * float(ceiling)
    np.clip(scaled, -float(ceiling), float(ceiling), out=scaled)
    return np.rint(scaled).astype("<i2")


def udp_audio_sender(
    audio_queue: mp.Queue,
    udp_socket: socket.socket,
    target: tuple[str, int],
    stop: mp.Event,
    keyed: mp.Event,
    packets: mp.Value,
    underruns: mp.Value,
    late_ms: mp.Value,
    trimmed: mp.Value,
    send_errors: mp.Value,
    ready: mp.Event,
    radio_rate: float = 0.0,
    repeats: mp.Value | None = None,
    ring_depth: mp.Value | None = None,
) -> None:
    """Pace TX audio outside the GUI process and its contended Python GIL."""
    packet_bytes = NETWORK_TX_PACKET_BYTES
    frames_per_packet = packet_bytes // 4
    packet_words = packet_bytes // 2
    # Emit at the radio's own clock when it is known, and convert the host stream
    # to it. Sending at the host rate instead leaves the radio's ring gaining or
    # losing a millisecond of audio every few seconds, which it resolves by
    # discarding a frame: a broadband click at exactly that period.
    period, base_ratio = tx_pacing(radio_rate)
    # refill() tops the buffer up whenever it falls below the low-water mark, so
    # the depth it actually holds is that mark plus up to one microphone block.
    # Aim at the middle of that band. Aiming at the mark itself, as this did,
    # means the measured error can never go negative, so the integrator winds up
    # against its limit and the servo runs permanently biased.
    target_frames = (
        NETWORK_TX_LOW_WATER_PACKETS * frames_per_packet
        + TransmitAudioRouter.BLOCK_SIZE // 2
    )
    resample_phase = 0.0
    ratio_trim = 0.0
    ratio_smooth = (0.0, 0.0)
    preroll_bytes = NETWORK_TX_PREROLL_PACKETS * packet_bytes
    low_water = NETWORK_TX_LOW_WATER_PACKETS * packet_bytes
    high_water = NETWORK_TX_HIGH_WATER_PACKETS * packet_bytes

    # Timebase first, because the send rate limiter below depends on it. Raising
    # the QoS class here rather than in transmit() covers the preroll too.
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
    period_ticks = int(period * ticks_per_second)
    burst_gap_ticks = int(NETWORK_TX_BURST_GAP * ticks_per_second)

    def pause(seconds: float) -> None:
        """Sleep `seconds`, using the mach timer when it is available."""
        if mach_time and mach_wait:
            mach_wait(mach_time() + int(seconds * ticks_per_second))
        else:
            time.sleep(seconds)

    pending = bytearray()
    while len(pending) < preroll_bytes and not stop.is_set():
        try:
            pending.extend(audio_queue.get(timeout=0.05))
        except queue.Empty:
            continue
    # Spawning this process costs a fresh interpreter and a full module import,
    # which is far longer than the preroll. Report readiness explicitly so the
    # caller keys the transmitter only once audio can actually leave the host;
    # a fixed pre-key sleep would either waste latency or open a dead-air gap.
    ready.set()
    while not keyed.wait(0.05) and not stop.is_set():
        pass

    record_stream = record_times = None
    if TX_RECORD_PREFIX:
        try:
            record_stream = open(f"{TX_RECORD_PREFIX}.tx.raw", "wb")
            record_times = open(f"{TX_RECORD_PREFIX}.tx.time", "wb")
        except OSError:
            if record_stream is not None:
                record_stream.close()
            record_stream = record_times = None

    last_send = [0]
    skipped = [0]
    repeated = [0]
    # Running estimate of the radio's ring depth, in int16 words. One scheduled
    # slot is exactly one packet's worth of consumption by construction, because
    # the slot period is 1/radio_rate, so the accounting is +/- one packet per
    # slot and does not depend on that rate being exactly right.
    ring_words = [0]
    last_payload = [b""]

    def send_scheduled() -> bool:
        """Emit one scheduled packet. Returns False only if nothing was sent.

        With no audio for the slot there are two ways to fail, and which one is
        right depends on the radio's ring depth.

        Skipping is inaudible while the ring has slack: it holds about 32 ms for
        exactly this, and the gap is covered. But skipping also spends that
        slack, and once the ring is empty every later hiccup becomes a hole on
        the air -- which is how a fix for one hole can produce more of them.

        So skip only while the estimate says the ring can afford it, and repeat
        the previous packet once it cannot. A repeat is a millisecond of
        duplicated audio, which is a small click, but it keeps the ring at depth
        and the schedule exact so the failure cannot cascade.
        """
        payload = next_payload()
        if payload is None:
            affordable = ring_words[0] - packet_words >= RADIO_RING_SHALLOW_WORDS
            if affordable or not last_payload[0]:
                skipped[0] += 1
                return False
            repeated[0] += 1
            if repeats is not None:
                repeats.value = repeated[0]
            send(last_payload[0])
            return True
        last_payload[0] = payload
        send(payload)
        return True

    def send(payload: bytes) -> None:
        # Enforce a floor on the spacing between datagrams here rather than at
        # each call site. Every path that can emit more than one packet in
        # succession -- the priming burst, catch-up after a late wake, debt
        # repayment, and a loop iteration whose deadline has already passed --
        # would otherwise hand the radio's Ethernet and lwIP receive path a
        # multi-thousand-packet-per-second burst, and a datagram lost there is a
        # millisecond of audio missing from the ring with nothing to resend it.
        if mach_time and mach_wait:
            now = mach_time()
            if last_send[0]:
                earliest = last_send[0] + burst_gap_ticks
                if now < earliest:
                    mach_wait(earliest)
                    # Record when the send actually happens, not when it was due.
                    # mach_wait can return late, and crediting the intended time
                    # would let the next gap close by however late it was.
                    now = mach_time()
            last_send[0] = now
        try:
            udp_socket.sendto(payload, target)
        except OSError:
            send_errors.value += 1
            return
        packets.value += 1
        ring_words[0] = min(ring_words[0] + packet_words, RADIO_RING_WORDS - 1)
        if ring_depth is not None:
            ring_depth.value = ring_words[0]
        if record_stream is not None:
            # Record after a successful send so the file is exactly the stream
            # the radio received, in order, with nothing the socket rejected.
            record_stream.write(payload)
            record_times.write(time.monotonic_ns().to_bytes(8, "little"))

    def refill() -> None:
        """Keep a cushion of captured audio inside this process.

        Refilling only once `pending` has run dry migrates the entire buffer
        across the feeder pipe and leaves the pacer with zero slack: it then has
        to complete a cross-process read inside a single packet period, once per
        microphone block, or emit silence. Drain opportunistically instead so
        the blocking read stays off the steady-state path.
        """
        while len(pending) < low_water:
            try:
                pending.extend(audio_queue.get_nowait())
            except queue.Empty:
                break
        if len(pending) < packet_bytes:
            try:
                # Capture arrives in 20 ms blocks through a feeder pipe. At
                # block boundaries get_nowait() can race that feeder and turn
                # available audio into a false underrun. Wait no longer than
                # one native packet before substituting silence.
                pending.extend(audio_queue.get(timeout=period))
            except queue.Empty:
                pass

    def next_payload() -> bytes | None:
        nonlocal resample_phase, ratio_trim, ratio_smooth
        refill()
        if len(pending) > high_water:
            # Trim the oldest whole packets rather than letting latency grow
            # without bound. With rate conversion working this should never fire;
            # it remains the backstop for a feeder that has run away.
            excess = (len(pending) - high_water) // packet_bytes
            del pending[: excess * packet_bytes]
            trimmed.value += excess
        # Hold the buffer at its target depth by trimming the conversion ratio.
        # This absorbs the host audio clock's error without needing to know it.
        ratio, ratio_trim, ratio_smooth = resample_ratio(
            len(pending) // 4, target_frames, ratio_trim, base_ratio, ratio_smooth
        )
        converted = resample_stereo(pending, frames_per_packet, ratio, resample_phase)
        if converted is None:
            underruns.value += 1
            # Send nothing rather than substituting silence. The radio's ring
            # holds about 32 ms precisely so a brief feeder hiccup costs nothing,
            # and skipping a packet spends that slack: measured on the air, a
            # transmitted silence packet is a hole in the audio, whereas the same
            # gap covered by the ring is inaudible. Debt repayment above then
            # restores the depth once capture catches up.
            #
            # Runs of these were the "faint pulsing": 8 dropouts of 2 to 12 ms in
            # 30 s, one every 3.5 s, at -38 dB, present on the network path and
            # absent from the radio's own USB path.
            return None
        payload, resample_phase = converted
        return payload

    # Drain the radio's residual ring, prime it, then pace. The transmitter is
    # already keyed by the time this runs: the parent sets `keyed` after CAT PTT.
    def transmit() -> None:
        # Drain whatever the previous transmission left in the radio's ring
        # before priming, or the depth accumulates across transmissions until the
        # firmware is correcting on every datagram. See NETWORK_TX_RING_DRAIN.
        # The transmitter is already keyed, so this is dead air immediately after
        # key-up, which is where an operator pauses anyway.
        drain_until = time.monotonic() + NETWORK_TX_RING_DRAIN
        while time.monotonic() < drain_until:
            if stop.is_set():
                return
            pause(0.002)
        # Start from a known buffer depth holding current audio. Everything
        # captured during the preroll and the drain is older than the audio the
        # operator is speaking now, and keeping it would put its whole duration
        # into the transmit path as standing latency. This is by design, so it is
        # not counted as a runaway trim.
        while True:
            try:
                pending.extend(audio_queue.get_nowait())
            except queue.Empty:
                break
        startup_bytes = (
            NETWORK_TX_PRIME_PACKETS + NETWORK_TX_LOW_WATER_PACKETS
        ) * packet_bytes
        if len(pending) > startup_bytes:
            del pending[: len(pending) - startup_bytes]

        # Prime the ring so the first scheduling jitter has something to eat into
        # rather than starving it, but pace the burst: see NETWORK_TX_BURST_GAP.
        for _ in range(NETWORK_TX_PRIME_PACKETS):
            if stop.is_set():
                # Teardown releases `keyed` so this process can exit. Do not
                # emit a burst into a transmitter that is already unkeyed.
                return
            send_scheduled()
            ring_words[0] = max(
                0,
                ring_words[0]
                - int(NETWORK_TX_BURST_GAP * RADIO_CONSUME_WORDS_PER_S),
            )

        deadline = mach_time() if mach_time else time.monotonic()
        debt_packets = 0
        while not stop.is_set():
            # One slot elapses per iteration, and the radio consumes exactly one
            # packet's worth in it. send() credits what actually goes out.
            ring_words[0] = max(0, ring_words[0] - packet_words)
            if not send_scheduled():
                # No audio for this slot. The ring covers it; owe it back.
                debt_packets = min(debt_packets + 1, NETWORK_TX_MAX_DEBT_PACKETS)
            if debt_packets:
                # Repay abandoned schedule debt one packet per period. Dropping
                # it instead makes the long-run send rate lower than the radio's
                # consume rate, and because nothing on this path can observe the
                # radio's ring depth, that deficit is never recovered: the ring
                # walks down past 1536 words and the firmware then duplicates a
                # frame on every single datagram. Repaying gradually keeps the
                # ring inside the corrector's dead zone without bursting.
                if send_scheduled():
                    debt_packets -= 1
            if mach_time and mach_wait:
                deadline += period_ticks
                mach_wait(deadline)
                lateness = (mach_time() - deadline) / ticks_per_second
                late_ms.value = max(late_ms.value, lateness * 1000)
                if lateness > period:
                    # Catch up on the missed schedule, bounded, instead of
                    # discarding it. Discarding makes the long-run send rate
                    # lower than the capture rate, so the buffer grows until the
                    # parent starts dropping whole 20 ms microphone blocks. This
                    # is only safe because refill() keeps a cushion in this
                    # process; with a dry buffer a stale deadline would turn one
                    # delayed wake into a run of audio underruns.
                    behind = int(lateness / period)
                    burst = min(behind, NETWORK_TX_MAX_CATCHUP_PACKETS)
                    for _ in range(burst):
                        ring_words[0] = max(0, ring_words[0] - packet_words)
                        send_scheduled()
                    deadline += burst * period_ticks
                    if (mach_time() - deadline) / ticks_per_second > period:
                        # Further behind than the catch-up bound allows. Resync
                        # rather than spiral into unbounded schedule debt, but
                        # carry the shortfall so it is repaid above.
                        now = mach_time()
                        shortfall = int(
                            (now - deadline) / ticks_per_second / period
                        )
                        ring_words[0] = max(
                            0, ring_words[0] - max(shortfall, 0) * packet_words
                        )
                        debt_packets = min(
                            debt_packets + max(shortfall, 0),
                            NETWORK_TX_MAX_DEBT_PACKETS,
                        )
                        deadline = now
            else:
                deadline += period
                lateness = time.monotonic() - deadline
                if lateness > 0:
                    late_ms.value = max(late_ms.value, lateness * 1000)
                    behind = int(lateness / period)
                    burst = min(behind, NETWORK_TX_MAX_CATCHUP_PACKETS)
                    for _ in range(burst):
                        ring_words[0] = max(0, ring_words[0] - packet_words)
                        send_scheduled()
                    deadline += burst * period
                    if time.monotonic() - deadline > period:
                        now = time.monotonic()
                        debt_packets = min(
                            debt_packets + max(int((now - deadline) / period), 0),
                            NETWORK_TX_MAX_DEBT_PACKETS,
                        )
                        deadline = now
                else:
                    time.sleep(-lateness)

    try:
        transmit()
    finally:
        if record_stream is not None:
            record_stream.close()
            record_times.close()


IQ_SAMPLE_RATE = 48_000
IQ_TX_LEVEL = 0.8
IQ_FM_DEVIATION = 2_500
IQ_PRE_EMPHASIS_ALPHA = 1.0 - float(np.exp(-1.0 / (48_000 * 750e-6)))
_HILBERT_LEN = 127
_HILBERT_DELAY = (_HILBERT_LEN - 1) // 2
_hilbert_index = np.arange(_HILBERT_LEN, dtype=np.float32) - _HILBERT_DELAY
_HILBERT_TAPS = np.zeros(_HILBERT_LEN, dtype=np.float32)
_hilbert_odd = (np.abs(_hilbert_index) % 2) == 1
_HILBERT_TAPS[_hilbert_odd] = 2.0 / (np.pi * _hilbert_index[_hilbert_odd])
_HILBERT_TAPS *= np.blackman(_HILBERT_LEN).astype(np.float32)


class IqEncoderState:
    """Streaming DSP state for encode_iq_block()."""

    __slots__ = ("phase", "level", "ssb_dc", "pre_prev", "hilbert_state", "sample_count")

    def __init__(self) -> None:
        self.phase = 0.0
        self.level = 0.0
        self.ssb_dc = 0.0
        self.pre_prev = 0.0
        self.hilbert_state = np.zeros(_HILBERT_LEN - 1, dtype=np.float32)
        self.sample_count = 0


def encode_iq_block(state: IqEncoderState, audio: np.ndarray, mode: str, offset_hz: int) -> np.ndarray:
    """Encode a 48 kHz mono audio block into complex I/Q samples.

    The Q900's network upconverter mirrors (conjugates) the complex baseband,
    so the returned samples are pre-conjugated; demodulation that simulates the
    radio must conjugate them back. Swap/invert calibration then stacks on top.
    """
    count = len(audio)
    if mode in ("USB", "LSB"):
        state.ssb_dc = 0.995 * state.ssb_dc + 0.005 * float(np.mean(audio))
        ssb_audio = np.clip(audio - state.ssb_dc, -0.45, 0.45)
        # The 127-tap Hilbert needs 63 future samples, so the streaming state
        # keeps the previous 126 samples and the whole output lags 63 behind.
        combined = np.concatenate((state.hilbert_state, ssb_audio))
        quadrature = np.convolve(combined, _HILBERT_TAPS, mode="valid")
        in_phase = combined[_HILBERT_DELAY : _HILBERT_DELAY + count]
        state.hilbert_state = combined[-(_HILBERT_LEN - 1):]
        # _HILBERT_TAPS realise H(w) = -j*sgn(w), so in_phase + 1j*quadrature is
        # the analytic signal: positive baseband frequencies only, which becomes
        # the upper sideband once the carrier and the radio's mirror are applied.
        # Do not flip this sign to correct an inverted sideband heard on air. It
        # mirrors USB and LSB together, so the mode labels swap and nothing is
        # actually corrected, and it leaves AM and NFM untouched because they
        # never reach this branch. A genuine whole-stream handedness error is a
        # property of the radio's mirror, so it belongs with the carrier offset
        # and the pack_iq_words() toggles, which act on every mode alike.
        baseband = in_phase + 1j * (quadrature if mode == "USB" else -quadrature)
    elif mode == "AM":
        state.ssb_dc = 0.995 * state.ssb_dc + 0.005 * float(np.mean(audio))
        baseband = 0.55 + np.clip(audio - state.ssb_dc, -0.45, 0.45).astype(np.complex64)
    else:  # NFM
        state.level = 0.95 * state.level + 0.05 * float(np.max(np.abs(audio)))
        fm_gain = float(np.clip(0.9 / max(state.level, 1e-4), 3.0, 20.0))
        fm_audio = np.clip(audio * fm_gain, -0.9, 0.9)
        previous = np.concatenate((np.array([state.pre_prev]), fm_audio[:-1]))
        emphasized = np.clip(fm_audio + IQ_PRE_EMPHASIS_ALPHA * (fm_audio - previous), -0.9, 0.9)
        state.pre_prev = float(fm_audio[-1])
        state.phase += np.cumsum(emphasized * (2 * np.pi * IQ_FM_DEVIATION / IQ_SAMPLE_RATE))
        baseband = np.exp(1j * state.phase)
        state.phase = float(state.phase[-1] % (2 * np.pi))
    index = np.arange(state.sample_count, state.sample_count + count)
    state.sample_count += count
    carrier = np.exp(1j * 2 * np.pi * offset_hz * index / IQ_SAMPLE_RATE)
    iq = np.conj(baseband * carrier)
    real = np.clip(iq.real, -1.0, 1.0) * IQ_TX_LEVEL
    imag = np.clip(iq.imag, -1.0, 1.0) * IQ_TX_LEVEL
    return real + 1j * imag


def pack_iq_words(iq: np.ndarray, swap_iq: bool, invert_q: bool) -> bytes:
    """Serialize complex I/Q samples as interleaved signed 16-bit words."""
    i_words = (iq.real * 32767).astype("<i2")
    q_words = (iq.imag * 32767).astype("<i2")
    if invert_q:
        q_words = -q_words
    if swap_iq:
        i_words, q_words = q_words, i_words
    words = np.empty(len(iq) * 2, dtype="<i2")
    words[0::2] = i_words
    words[1::2] = q_words
    return words.tobytes()


def udp_iq_sender(
    audio_queue: mp.Queue,
    udp_socket: socket.socket,
    target: tuple[str, int],
    stop: mp.Event,
    keyed: mp.Event,
    packets: mp.Value,
    underruns: mp.Value,
    late_ms: mp.Value,
    clipped: mp.Value,
    mode: str,
    offset_hz: int,
    swap_iq: bool,
    invert_q: bool,
) -> None:
    """Encode 48 kHz microphone audio into inferred raw network I/Q."""
    frames_per_packet = 48
    preroll_bytes = 9_600 * 2
    pending = bytearray()
    while len(pending) < preroll_bytes and not stop.is_set():
        try:
            pending.extend(audio_queue.get(timeout=0.05))
        except queue.Empty:
            continue
    while not keyed.wait(0.05) and not stop.is_set():
        pass

    state = IqEncoderState()
    period = 0.001
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

    def next_payload() -> bytes:
        needed = frames_per_packet * 2
        while len(pending) < needed:
            try:
                pending.extend(audio_queue.get(timeout=period))
            except queue.Empty:
                break
        if len(pending) < needed:
            underruns.value += 1
            # Carry the carrier continuously through a scheduling gap rather
            # than zeroing it: a zero packet pops the FM discriminator.
            audio = np.zeros(frames_per_packet, dtype=np.float32)
        else:
            audio = np.frombuffer(bytes(pending[:needed]), dtype="<i2").astype(np.float32) / 32768.0
            del pending[:needed]
        if np.any(np.abs(audio) >= 0.98):
            clipped.value += 1
        iq = encode_iq_block(state, audio, mode, offset_hz)
        return pack_iq_words(iq, swap_iq, invert_q)

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
        self._udp_trimmed: mp.Value | None = None
        self._udp_send_errors: mp.Value | None = None
        self._udp_overflows: mp.Value | None = None
        self._udp_dropped: mp.Value | None = None
        self._udp_repeats: mp.Value | None = None
        self._udp_ring: mp.Value | None = None
        self._udp_ready: mp.Event | None = None
        self._udp_ceiling = 0
        self._udp_compressor = 0

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

    def start_udp(
        self,
        microphone: int,
        target: tuple[str, int],
        network_audio: NetworkAudioMonitor,
        compressor: int = 9,
    ) -> None:
        self.stop()
        self._udp_target = target
        # The radio applies a pre-gain of up to 13x before its ALC, selected by
        # CAT 0x10, and never scales network audio down to compensate. Derive the
        # peak we may send from that setting. The radio does not report the value
        # back, so this is the host's own record of it.
        ceiling = network_tx_ceiling(compressor)
        self._udp_ceiling = ceiling
        self._udp_compressor = compressor
        dc_blocker = DcBlocker(sample_rate=self.NETWORK_SAMPLE_RATE)
        self._udp_queue = self._mp.Queue(maxsize=50)
        self._udp_stop = self._mp.Event()
        self._udp_keyed = self._mp.Event()
        self._udp_packets = self._mp.Value("L", 0, lock=False)
        self._udp_underruns = self._mp.Value("L", 0, lock=False)
        self._udp_late_ms = self._mp.Value("d", 0.0, lock=False)
        self._udp_clipped = self._mp.Value("L", 0, lock=False)
        self._udp_trimmed = self._mp.Value("L", 0, lock=False)
        self._udp_send_errors = self._mp.Value("L", 0, lock=False)
        self._udp_overflows = self._mp.Value("L", 0, lock=False)
        self._udp_dropped = self._mp.Value("L", 0, lock=False)
        self._udp_repeats = self._mp.Value("L", 0, lock=False)
        self._udp_ring = self._mp.Value("l", 0, lock=False)
        self._udp_ready = self._mp.Event()

        def callback(indata, frames, timing, status):  # type: ignore[no-untyped-def]
            if self._udp_overflows and status.input_overflow:
                # Capture samples were lost before they reached us. The byte
                # stream stays contiguous, so this is a splice rather than a
                # gap: no downstream counter can see it, and it reaches the air
                # as a broadband click. Almost always GIL starvation of this
                # callback by expensive work elsewhere in the GUI process.
                self._udp_overflows.value += 1
            raw = indata[:, 0]
            with self._level_lock:
                self._level = float(np.max(np.abs(raw)))
                self._output_level = self._level
            if self._udp_clipped and np.any(np.abs(raw) >= 0.98):
                self._udp_clipped.value += 1
            # Remove DC and subsonic energy the firmware will not filter for us.
            pcm = dc_blocker.process(raw)
            # Scale to the level the radio's TX gain chain expects rather than to
            # int16 full scale. Sending full scale overdrove the pre-gain that
            # CAT 0x10 selects and left the radio's ALC in permanent heavy
            # limiting, which is what made the audio rough.
            samples = quantize_tx(pcm, ceiling)
            # Only the first word of each stereo frame is read by the firmware
            # (0x08039846 takes element 0 and discards element 1). Duplicating
            # mono keeps the frame geometry the ring consumer requires and makes
            # a one-word ring misalignment inaudible instead of channel-swapping.
            payload = np.repeat(samples, 2).tobytes()
            if self._udp_queue:
                try:
                    self._udp_queue.put_nowait(payload)
                except queue.Full:
                    # Drop the oldest buffered block so the newest microphone
                    # audio is never silently lost. Count it: this discards a
                    # whole 20 ms of audio, which reaches the air as a splice,
                    # and it used to be the one event on this path that no
                    # counter could see.
                    if self._udp_dropped:
                        self._udp_dropped.value += 1
                    try:
                        self._udp_queue.get_nowait()
                    except queue.Empty:
                        pass
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
                self._udp_trimmed,
                self._udp_send_errors,
                self._udp_ready,
                # The radio's measured clock, so the sender can emit at it and
                # convert the host stream rather than letting the difference
                # accumulate in the radio's ring.
                network_audio.measured_packet_rate,
                self._udp_repeats,
                self._udp_ring,
            ),
            name="q900-udp-tx",
            daemon=True,
        )
        self._udp_sender.start()
        self._input_stream.start()
        radio_rate = network_audio.measured_packet_rate
        clock = (
            f", radio clock {radio_rate:.2f} pkt/s"
            if radio_rate
            else ", radio clock not yet measured"
        )
        state = (
            f"PTT audio: microphone -> Q900 UDP {target[0]}:{target[1]} "
            f"(48 kHz stereo S16LE{clock}, peak {ceiling} "
            f"for CMP {compressor} = {TX_PREGAIN_BY_COMPRESSOR[min(max(compressor, 0), 14)]:.2f}x)"
        )
        if not self._udp_ready.wait(timeout=NETWORK_TX_READY_TIMEOUT):
            state += " -- sender did not report ready, transmit may start late"
        self.signals.audio_state_changed.emit(state)

    def start_iq_udp(
        self,
        microphone: int,
        target: tuple[str, int],
        network_audio: NetworkAudioMonitor,
        mode: str,
        offset_hz: int,
        swap_iq: bool,
        invert_q: bool,
    ) -> None:
        self.stop()
        self._udp_target = target
        self._udp_queue = self._mp.Queue(maxsize=50)
        self._udp_stop = self._mp.Event()
        self._udp_keyed = self._mp.Event()
        self._udp_packets = self._mp.Value("L", 0, lock=False)
        self._udp_underruns = self._mp.Value("L", 0, lock=False)
        self._udp_late_ms = self._mp.Value("d", 0.0, lock=False)
        self._udp_clipped = self._mp.Value("L", 0, lock=False)
        # The I/Q sender does not report trims or send errors. Clear them so the
        # PTT line cannot show stale values left by a previous audio keying.
        self._udp_trimmed = None
        self._udp_send_errors = None
        self._udp_ready = None
        # Capture overflow applies to both TX paths: this one has an InputStream
        # sharing the GUI process GIL exactly as the audio path does.
        self._udp_overflows = self._mp.Value("L", 0, lock=False)

        def callback(indata, frames, timing, status):  # type: ignore[no-untyped-def]
            if self._udp_overflows and status.input_overflow:
                self._udp_overflows.value += 1
            pcm = np.clip(indata[:, 0], -1, 1)
            with self._level_lock:
                self._level = float(np.max(np.abs(pcm)))
                self._output_level = self._level
            if self._udp_queue:
                try:
                    self._udp_queue.put_nowait((pcm * 32767).astype("<i2").tobytes())
                except queue.Full:
                    # Drop the oldest buffered block so the newest microphone
                    # audio is never silently lost.
                    try:
                        self._udp_queue.get_nowait()
                    except queue.Empty:
                        pass
                    try:
                        self._udp_queue.put_nowait((pcm * 32767).astype("<i2").tobytes())
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
            target=udp_iq_sender,
            args=(
                self._udp_queue, network_audio.socket, target, self._udp_stop, self._udp_keyed,
                self._udp_packets, self._udp_underruns, self._udp_late_ms, self._udp_clipped, mode, offset_hz,
                swap_iq, invert_q,
            ),
            name="q900-iq-tx",
            daemon=True,
        )
        self._udp_sender.start()
        self._input_stream.start()
        time.sleep(self.NETWORK_PREROLL_SAMPLES / self.NETWORK_SAMPLE_RATE)
        self.signals.audio_state_changed.emit(
            f"SDR TX: microphone -> Q900 UDP {target[0]}:{target[1]} ({mode} I/Q, {offset_hz:+d} Hz)"
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
        self._udp_ready = None
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
        trimmed = self._udp_trimmed.value if self._udp_trimmed else 0
        errors = self._udp_send_errors.value if self._udp_send_errors else 0
        overflows = self._udp_overflows.value if self._udp_overflows else 0
        dropped = self._udp_dropped.value if self._udp_dropped else 0
        repeats = self._udp_repeats.value if self._udp_repeats else 0
        ring = self._udp_ring.value if self._udp_ring else 0
        return (
            f"UDP {packets} pkts  ovf {overflows}  drop {dropped}  skip {underruns}  "
            f"rep {repeats}  ring {ring / 96.0:.0f}ms  "
            f"trim {trimmed}  err {errors}  late {late_ms:.1f} ms  clip {clipped}  "
            f"peak {self._udp_ceiling}/CMP {self._udp_compressor}"
        )


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
                    if self.state.ptt:
                        # Do not ask the radio to compute and stream a 516-byte
                        # FFT frame while it is transmitting. The spectrum shows
                        # the receive passband and is not meaningful on air, and
                        # the request costs radio DSP time and a host repaint
                        # that competes with the microphone callback. Clear the
                        # pending marker so polling resumes on unkey instead of
                        # waiting out a stale request window.
                        spectrum_pending_at = 0.0
                    elif not spectrum_pending_at or now - spectrum_pending_at >= 0.15:
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


def waterfall_argb(rows: np.ndarray, width: int) -> np.ndarray:
    """Map stacked 8-bit spectrum rows to one ARGB scanline per row.

    Each row is normalised against its own minimum and maximum, exactly as the
    original per-pixel mapping did. Vectorising this is not cosmetic: a
    QImage.setPixel() loop over the same pixels costs 20-70 ms per repaint,
    which is one to three microphone callback periods, and the GUI process
    shares its GIL with that callback. Paint cost is transmit audio quality.
    """
    index = np.arange(width) * (rows.shape[1] - 1) // max(1, width - 1)
    full = rows.astype(np.int32)
    # Normalise against the whole row, not the resampled pixels. These coincide
    # once the widget is wider than the bin count but diverge when it is not.
    minimum = full.min(axis=1, keepdims=True)
    spread = np.maximum(1, full.max(axis=1, keepdims=True) - minimum)
    # Build the ARGB words in uint32: the opaque alpha byte does not fit int32.
    intensity = ((full[:, index] - minimum) * 255 // spread).astype(np.uint32)
    return (
        np.uint32(0xFF000000)
        | (intensity << 16)
        | ((80 + intensity * 175 // 255) << 8)
        | (40 + (255 - intensity) * 150 // 255)
    )


def should_autostart_audio(
    connected: bool,
    transport: str,
    audio_wanted: bool,
    usb_running: bool,
    network_running: bool,
) -> bool:
    """Whether an incoming status frame should start receive audio.

    Status frames arrive about twice a second, so this has to return False once
    the operator has stopped audio. Restarting on every frame made stopping it
    impossible. The radio's audio routing is a front-panel menu with no CAT
    equivalent, so releasing the media port is the only way to hand receive audio
    to Bluetooth or to another application on this machine.
    """
    if not connected or not audio_wanted:
        return False
    if transport == "USB":
        return not usb_running
    return not network_running


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
        self._repaint_timer = QTimer(self)
        self._repaint_timer.setSingleShot(True)
        self._repaint_timer.timeout.connect(self.update)

    def _schedule_update(self) -> None:
        """Coalesce repaints so frame arrival rate cannot drive paint rate 1:1."""
        if not self._repaint_timer.isActive():
            self._repaint_timer.start(1000 // SPECTRUM_MAX_REPAINT_HZ)

    def set_state(self, state: RadioState) -> None:
        tuned_hz = state.vfo_b_hz if state.active_vfo_b else state.vfo_a_hz
        self._tuned_hz = tuned_hz
        self._display_center_hz = self._tuned_hz
        self._mode = state.vfo_b_mode if state.active_vfo_b else state.vfo_a_mode
        self._span_hz = SPAN_HZ[state.span_index]
        self._schedule_update()

    def add_bins(self, bins: bytes) -> None:
        self._bins = bins
        self._rows.insert(0, bins)
        self._rows = self._rows[:140]
        self._schedule_update()

    def set_sdr(self, active: bool, offset_hz: int, mode: str) -> None:
        self._sdr_active = active
        self._sdr_offset_hz = offset_hz
        self._sdr_mode = mode
        self._schedule_update()

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
        # Deliberately a drawLine() loop, not drawPolyline(). The 1.3-width pen
        # makes Qt stroke a joined 1400-segment path, which measured 22.5 ms
        # against 0.73 ms for independent cosmetic lines.
        for first, second in zip(points, points[1:]):
            painter.drawLine(first, second)
        painter.setPen(QColor("#9aaab5"))
        painter.drawText(8, 18, f"{self._display_center_hz - self._span_hz // 2:,} Hz")
        painter.drawText(max(8, width - 180), 18, f"{self._display_center_hz + self._span_hz // 2:,} Hz")

    def _draw_waterfall(self, painter: QPainter, width: int, top: int, height: int) -> None:
        painter.fillRect(0, top, width, height, QColor("#02050c"))
        if not self._rows or width < 1:
            painter.setPen(QColor("#63727d"))
            painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, "Waiting for spectrum frames")
            return
        row_height = max(1, height // min(len(self._rows), 100))
        rows = [row for row in self._rows[: max(1, height // row_height)] if len(row) >= 2]
        if not rows:
            return
        bin_count = len(rows[0])
        rows = [row for row in rows if len(row) == bin_count]
        # Build every scanline at once and blit the waterfall in a single
        # drawImage. The previous per-pixel QImage.setPixel() loop held the GIL
        # for 20-70 ms per repaint, which starved the microphone callback in this
        # same process and put broadband clicks on the transmitted audio.
        stacked = np.frombuffer(b"".join(rows), dtype=np.uint8).reshape(len(rows), bin_count)
        scanlines = waterfall_argb(stacked, width)
        image = QImage(
            # tobytes() hands Qt an owned copy, so no backing buffer has to
            # outlive this call.
            scanlines.tobytes(),
            width,
            len(rows),
            width * 4,
            QImage.Format.Format_RGB32,
        )
        painter.drawImage(QRectF(0, top, width, len(rows) * row_height), image)

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
        # Whether the operator wants receive audio at all. Connecting starts it
        # automatically, but "Stop Audio" has to stick: status frames arrive twice
        # a second and each one used to restart the stream, so stopping it was
        # impossible. The radio's own audio routing is a front-panel menu, so
        # releasing the media port is the only way to hand the audio elsewhere.
        self._audio_wanted = True
        self._last_ptt_network_status = ""
        self._sdr_active = False
        self._sdr_switch_pending = False
        self._sdr_restore_pending = False
        self._sdr_restore_attempts = 0
        self._sdr_tx_offset_hz = 12_000
        self._sdr_tx_swap_iq = False
        self._sdr_tx_invert_q = False
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
        self.sdr_tx_calibrate = QPushButton("SDR TX Cal")
        self.sdr_tx_calibrate.setVisible(False)
        self.sdr_tx_calibrate.clicked.connect(self.configure_sdr_tx)
        header.addWidget(self.sdr_button)
        header.addWidget(self.sdr_mode_selector)
        header.addWidget(self.sdr_offset)
        header.addWidget(self.sdr_tx_calibrate)
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

    def _assert_network_audio_format(self) -> None:
        """Force the radio into audio stream format before network transmit.

        The radio's transmit path is gated on state[0x131], set only by CAT 0x33.
        The ingest at 0x0806C80C accepts a datagram only when it is 1 or 2, and
        the transmit DSP at 0x08039E8E treats 2 as raw I/Q: it copies the first
        word of each frame into the I array and the second into Q, bypassing the
        SSB modulator and every speech-processing stage. Feeding duplicated mono
        into that produces I == Q, a double-sideband signal with no filtering --
        recognisable audio, but rough, and immune to any amount of level or
        buffer correction on this side.

        Nothing here used to set it. The firmware substitutes 1 when it enables
        streaming (0x0806DA70) but only if the value is exactly 0, so a 2 left
        behind by an earlier SDR session survives indefinitely, and the value
        lives in .bss so it is whatever the last thing to touch it chose. Assert
        it explicitly rather than inheriting it.

        The radio's own receive stream reports the same byte back: it frames
        packets as type 0x67 in audio format and 0x68 in I/Q. So the state is
        observable, and a mismatch is worth saying out loud rather than silently
        correcting, because it means transmit audio up to this point was being
        interpreted as I/Q.
        """
        observed = self.network_audio.stream_type
        self.client.set_stream_format(0)
        if observed == 0x68:
            self.status.setText(
                "Radio was streaming I/Q (0x68): transmit audio would have been "
                "interpreted as I/Q. Forced audio format (CAT 0x33 = 0)."
            )

    def start_ptt(self) -> None:
        if self._sdr_switch_pending or self._sdr_restore_pending:
            self.status.setText("Wait for the SDR stream transition to complete before transmitting.")
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
                if self._sdr_active:
                    self.status.setText("SDR IQ TX currently requires the network transport.")
                    return
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
                if self._sdr_active:
                    self.tx_audio.start_iq_udp(
                        microphone,
                        target,
                        self.network_audio,
                        self.sdr_receiver.mode,
                        self._sdr_tx_offset_hz,
                        self._sdr_tx_swap_iq,
                        self._sdr_tx_invert_q,
                    )
                else:
                    # Force audio stream format before any audio leaves the host:
                    # the radio would otherwise transmit it as raw I/Q if a
                    # previous SDR session left the format at 2.
                    self._assert_network_audio_format()
                    self.tx_audio.start_udp(
                        microphone,
                        target,
                        self.network_audio,
                        self.client.state.compressor,
                    )
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
            try:
                self.client.set_ptt(False)
            except (ConnectionError, OSError, serial.SerialException):
                pass
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
        if not self._audio_wanted:
            self.rigctl_status.setText(
                "rigctl: client connected, receive audio is stopped"
            )
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
        if self.client.state.connected and self._audio_wanted:
            QTimer.singleShot(0, self.start_audio_default)

    def handle_rigctl_ptt(self, active: bool) -> None:
        if active and (self._sdr_switch_pending or self._sdr_restore_pending):
            self.rigctl_status.setText("rigctl: SDR stream transition in progress")
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
                if self._sdr_active:
                    self.tx_audio.start_iq_udp(
                        microphone,
                        target,
                        self.network_audio,
                        self.sdr_receiver.mode,
                        self._sdr_tx_offset_hz,
                        self._sdr_tx_swap_iq,
                        self._sdr_tx_invert_q,
                    )
                else:
                    # Force audio stream format before any audio leaves the host:
                    # the radio would otherwise transmit it as raw I/Q if a
                    # previous SDR session left the format at 2.
                    self._assert_network_audio_format()
                    self.tx_audio.start_udp(
                        microphone,
                        target,
                        self.network_audio,
                        self.client.state.compressor,
                    )
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
            try:
                self.client.set_ptt(False)
            except (ConnectionError, OSError, serial.SerialException):
                pass
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
            self._audio_wanted = False
            self.audio.stop()
            self.network_audio.stop()
            self._network_audio_timer.stop()
            self.network_audio_status.setText("")
            self.audio_button.setText("Start Audio")
            if self.client.state.transport == "TCP":
                self.status.setText(
                    "Receive audio stopped and UDP/8000 released. Network PTT and "
                    "SDR need it restarted."
                )
            else:
                self.status.setText("Receive audio monitor stopped.")
            return
        self._audio_wanted = True
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
        if not self._audio_wanted or self.audio_output.currentData() is None:
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
            # Requesting SDR is a request for the media stream.
            self._audio_wanted = True
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

    def configure_sdr_tx(self) -> None:
        if self._ptt_source:
            self.status.setText("Release PTT before changing SDR TX calibration.")
            return
        dialog = QDialog(self)
        dialog.setWindowTitle("SDR TX Calibration")
        layout = QVBoxLayout(dialog)
        layout.addWidget(QLabel("Use low power and an external receiver. Change one setting per test."))
        offset = QComboBox()
        for value in (12_000, 0, -12_000):
            offset.addItem(f"{value:+d} Hz", value)
        offset.setCurrentIndex(max(0, offset.findData(self._sdr_tx_offset_hz)))
        swap = QCheckBox("Swap I/Q")
        swap.setChecked(self._sdr_tx_swap_iq)
        invert = QCheckBox("Invert Q")
        invert.setChecked(self._sdr_tx_invert_q)
        current = QLabel("")

        def describe() -> None:
            current.setText(
                f"TX: {self.sdr_receiver.mode}, {offset.currentText()}, "
                f"{'Q,I' if swap.isChecked() else 'I,Q'}, "
                f"{'-Q' if invert.isChecked() else '+Q'}"
            )

        offset.currentIndexChanged.connect(describe)
        swap.toggled.connect(describe)
        invert.toggled.connect(describe)
        describe()
        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Cancel | QDialogButtonBox.StandardButton.Ok)
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addWidget(QLabel("Carrier offset"))
        layout.addWidget(offset)
        layout.addWidget(swap)
        layout.addWidget(invert)
        layout.addWidget(current)
        layout.addWidget(buttons)
        if dialog.exec() == QDialog.DialogCode.Accepted:
            self._sdr_tx_offset_hz = int(offset.currentData())
            self._sdr_tx_swap_iq = swap.isChecked()
            self._sdr_tx_invert_q = invert.isChecked()
            self.status.setText(f"SDR TX calibration set: {current.text()}")

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
        self.sdr_tx_calibrate.setVisible(True)
        self.spectrum.set_sdr(True, self.sdr_receiver.offset_hz, self.sdr_receiver.mode)
        self.status.setText("SDR RX active: 48 kHz network IQ at +12 kHz. Network PTT sends SDR I/Q TX.")

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
        if self._ptt_source == "gui":
            self.stop_ptt()
        elif self._ptt_source == "rigctl":
            self.handle_rigctl_ptt(False)
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
        self.sdr_tx_calibrate.setVisible(False)
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
            self._audio_wanted = True
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
        if should_autostart_audio(
            state.connected,
            state.transport,
            self._audio_wanted,
            self.audio.running,
            self.network_audio.running,
        ):
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
        if self._ptt_source == "gui":
            self.stop_ptt()
        elif self._ptt_source == "rigctl":
            self.handle_rigctl_ptt(False)
        if self._sdr_active or self._sdr_switch_pending:
            self.exit_sdr()
        self._sdr_restore_timer.stop()
        self._sdr_restore_pending = False
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
    # A network TX datagram is one 1 ms media frame: 48 interleaved stereo
    # frames, 96 int16 words, 192 bytes -- the same quantum the radio sends on
    # RX. Tie the assertion to the captured RX payload so the two directions
    # cannot silently drift apart again.
    mono_tx = np.arange(48, dtype="<i2")
    stereo_tx = np.repeat(mono_tx, 2)
    assert len(stereo_tx) == 96
    assert len(stereo_tx.tobytes()) == 192 == len(captured_audio[9:])
    assert np.array_equal(stereo_tx[0::2], stereo_tx[1::2])
    # Pin the TX geometry to the captured RX payload and to the 48 kHz stereo
    # byte rate. Either identity would have caught the 384-byte/2 ms datagram.
    assert NETWORK_TX_PACKET_BYTES == 192 == len(captured_audio[9:])
    assert NETWORK_TX_PACKET_BYTES / NETWORK_TX_PERIOD == 48_000 * 2 * 2
    # The firmware derives its word count as `bytes >> 1` and stages at most
    # 2560 bytes, so a datagram must be whole stereo frames and must fit.
    assert NETWORK_TX_PACKET_BYTES % 4 == 0
    assert NETWORK_TX_PACKET_BYTES <= NETWORK_TX_MAX_DATAGRAM_BYTES

    # Transmit gain staging. The radio multiplies network audio by a pre-gain of
    # up to 13x (CAT 0x10 -> state[0x140] -> table at 0x080DAF14) before an ALC
    # whose knee is 30000, and never scales it down to compensate. Sending int16
    # full scale at the default CMP 9 drove 8x into that knee: 18.8 dB of
    # permanent limiting, which is what made transmit audio rough.
    assert len(TX_PREGAIN_BY_COMPRESSOR) == 15
    assert TX_PREGAIN_BY_COMPRESSOR[0] == TX_PREGAIN_BY_COMPRESSOR[1] == 1.00
    assert TX_PREGAIN_BY_COMPRESSOR[9] == 8.00
    assert TX_PREGAIN_BY_COMPRESSOR[12] == 13.00
    for cmp_value in range(15):
        ceiling = network_tx_ceiling(cmp_value)
        assert 0 < ceiling <= 32767, (cmp_value, ceiling)
        # Whatever the setting, what the radio's ALC sees must land under its
        # knee. This is the invariant the old full-scale path violated.
        assert (
            ceiling * TX_PREGAIN_BY_COMPRESSOR[cmp_value] <= TX_ALC_THRESHOLD
        ), (cmp_value, ceiling)
    # Out-of-range settings must clamp, never index past the table.
    assert network_tx_ceiling(-5) == network_tx_ceiling(0)
    assert network_tx_ceiling(99) == network_tx_ceiling(14)
    # The default setting must no longer produce a full-scale stream.
    assert network_tx_ceiling(9) < 32767 // 4

    # The DC blocker is evaluated in closed form to keep the microphone callback
    # off a per-sample Python loop. It must match the recursion it replaces, and
    # must stay continuous across block boundaries: a discontinuity there is a
    # click at the block rate, which is exactly the class of defect that the
    # carried-state convolution elsewhere in this file was added to fix.
    def dc_reference(samples: np.ndarray, a: float) -> np.ndarray:
        out = np.empty(len(samples), dtype=np.float64)
        last_in = last_out = 0.0
        for i, value in enumerate(samples):
            last_out = float(value) - last_in + a * last_out
            last_in = float(value)
            out[i] = last_out
        return out

    rng = np.random.default_rng(1)
    signal = rng.standard_normal(2048).astype(np.float32) * 0.25 + 0.4
    blocker = DcBlocker()
    blocked = np.concatenate(
        [blocker.process(signal[start : start + 512]) for start in range(0, 2048, 512)]
    )
    assert np.allclose(blocked, dc_reference(signal, blocker._a), atol=2e-5)
    # A constant input must decay to nothing, and the removal must not eat the
    # voice band: a 20 Hz single pole is under 0.1 dB down at 300 Hz.
    steady = DcBlocker()
    steady.process(np.full(48_000, 0.5, dtype=np.float32))
    assert abs(float(steady.process(np.full(4_800, 0.5, dtype=np.float32))[-1])) < 1e-3
    tone_n = np.arange(48_000)
    tone = np.sin(2.0 * np.pi * 300.0 * tone_n / 48_000).astype(np.float32)
    passed = DcBlocker().process(tone)[24_000:]
    assert 0.99 < float(np.max(np.abs(passed))) <= 1.0, float(np.max(np.abs(passed)))

    # quantize_tx must round rather than truncate, must honour the ceiling, and
    # must never emit int16 full scale when the ceiling is lower.
    assert quantize_tx(np.array([0.5], dtype=np.float32), 30_000)[0] == 15_000
    assert quantize_tx(np.array([1.0 / 3.0], dtype=np.float32), 10)[0] == 3
    assert quantize_tx(np.array([2.0, -2.0], dtype=np.float32), 3_750).tolist() == [
        3_750,
        -3_750,
    ]
    tx_ceiling = network_tx_ceiling(9)
    words = quantize_tx(tone[:48], tx_ceiling)
    frames_out = np.repeat(words, 2)
    assert len(frames_out.tobytes()) == NETWORK_TX_PACKET_BYTES
    assert len(frames_out.tobytes()) % 4 == 0
    assert np.array_equal(frames_out[0::2], frames_out[1::2])
    assert int(np.max(np.abs(frames_out))) <= tx_ceiling
    assert int(np.max(np.abs(frames_out))) < 32767

    # The recording analysers lean on _analytic() to recover envelope and phase.
    # A steady tone must come back with a flat envelope and a straight phase ramp,
    # because the sensitive splice test measures departures from exactly that.
    tone_n = np.arange(8192)
    probe = np.sin(2.0 * np.pi * 1500.0 * tone_n / 48_000)
    analytic = _analytic(probe)
    core = np.abs(analytic)[512:-512]
    assert np.ptp(core) / np.mean(core) < 0.01, float(np.ptp(core) / np.mean(core))
    ramp = np.unwrap(np.angle(analytic))[512:-512]
    slope = np.polyfit(np.arange(len(ramp)), ramp, 1)[0]
    assert abs(slope * 48_000 / (2 * np.pi) - 1500.0) < 1.0, slope

    def phase_departure(samples: np.ndarray) -> float:
        resid = np.unwrap(np.angle(_analytic(samples)))[512:-512]
        index = np.arange(len(resid))
        return float(np.ptp(resid - np.polyval(np.polyfit(index, resid, 1), index)))

    # One dropped sample displaces the phase by one sample's worth of advance,
    # 2*pi*f0/fs, not by a whole cycle. That is the unit the analyser converts
    # back into a sample count, so pin the scale factor here: get it wrong and a
    # splice is reported as the wrong number of samples.
    per_sample = 2.0 * np.pi * 1500.0 / 48_000
    departure = phase_departure(np.delete(probe, 4096))
    assert 0.5 * per_sample < departure < 3.0 * per_sample, (departure, per_sample)
    # And it has to stand out from an undisturbed tone, or the test cannot see it.
    assert departure > 10.0 * phase_departure(probe), (
        departure,
        phase_departure(probe),
    )

    # The sender must start with more than it holds back, and must never be
    # asked to hold back more than the hard cap allows.
    assert (
        NETWORK_TX_LOW_WATER_PACKETS
        < NETWORK_TX_PREROLL_PACKETS
        <= NETWORK_TX_HIGH_WATER_PACKETS
    )
    assert NETWORK_TX_PRIME_PACKETS <= NETWORK_TX_PREROLL_PACKETS
    # The radio's ring corrector leaves 1536..4608 words alone and duplicates or
    # drops a frame on every datagram outside that. The consumer keeps running
    # while the priming burst is paced out, so the depth the ring actually settles
    # at is the burst less what was consumed during it. That figure, not the
    # packet count, is what has to land in the middle of the window.
    prime_words = NETWORK_TX_PRIME_PACKETS * NETWORK_TX_PACKET_BYTES // 2
    consumed_while_priming = int(
        NETWORK_TX_PRIME_PACKETS * NETWORK_TX_BURST_GAP * RADIO_CONSUME_WORDS_PER_S
    )
    settled = prime_words - consumed_while_priming
    assert RADIO_RING_SHALLOW_WORDS < settled < RADIO_RING_DEEP_WORDS, settled
    assert abs(settled - RADIO_RING_TARGET_WORDS) <= NETWORK_TX_PACKET_BYTES // 2, (
        settled,
        RADIO_RING_TARGET_WORDS,
    )
    # Debt is incurred by falling behind, which drains the ring, so the bound is
    # the drain headroom between where the ring settles and the duplication
    # threshold. Beyond that the firmware has already duplicated frames and
    # repaying only adds latency.
    assert (
        NETWORK_TX_MAX_DEBT_PACKETS * NETWORK_TX_PACKET_BYTES // 2
        <= settled - RADIO_RING_SHALLOW_WORDS
    ), NETWORK_TX_MAX_DEBT_PACKETS
    # Nothing resets the radio's ring indices and it is only consumed while PTT
    # is asserted, so the drain must outlast a completely full ring or depth
    # accumulates across transmissions.
    assert (
        NETWORK_TX_RING_DRAIN * RADIO_CONSUME_WORDS_PER_S > RADIO_RING_WORDS
    ), NETWORK_TX_RING_DRAIN
    # A paced burst has to be faster than real time to catch up at all, and
    # slower than line rate to survive the radio's receive path.
    assert 0.0 < NETWORK_TX_BURST_GAP < NETWORK_TX_PERIOD
    # The priming burst must still complete promptly once paced.
    assert NETWORK_TX_PRIME_PACKETS * NETWORK_TX_BURST_GAP < 0.020
    # The startup trim must leave the sender its full cushion plus the burst it
    # is about to emit, or priming immediately underruns.
    assert (
        NETWORK_TX_PRIME_PACKETS + NETWORK_TX_LOW_WATER_PACKETS
        <= NETWORK_TX_HIGH_WATER_PACKETS
    )
    # The priming burst is no longer drawn from the preroll alone. The sender
    # accumulates through the preroll, keys, then waits out the ring drain while
    # capture keeps arriving, and only then trims to the depth it wants. So the
    # audio on hand when priming starts is the preroll plus the drain, and the
    # cushion left afterwards is the low-water mark by construction.
    mic_block_packets = (
        TransmitAudioRouter.BLOCK_SIZE * 2 * 2 // NETWORK_TX_PACKET_BYTES
    )
    drain_packets = int(NETWORK_TX_RING_DRAIN / NETWORK_TX_PERIOD)
    available_at_prime = NETWORK_TX_PREROLL_PACKETS + drain_packets
    startup_packets = NETWORK_TX_PRIME_PACKETS + NETWORK_TX_LOW_WATER_PACKETS
    assert available_at_prime >= startup_packets, (
        available_at_prime,
        startup_packets,
    )
    # The cushion held after priming must cover more than a single microphone
    # callback or the buffer bottoms out every mic period and any late block
    # becomes an audible gap.
    assert NETWORK_TX_LOW_WATER_PACKETS >= 2 * mic_block_packets, (
        NETWORK_TX_LOW_WATER_PACKETS,
        mic_block_packets,
    )

    # The vectorized waterfall must reproduce the previous per-pixel mapping
    # exactly. That loop was replaced because it held the GIL for 20-70 ms per
    # repaint and starved the microphone callback in the same process.
    def waterfall_argb_reference(bins: bytes, width: int) -> list[int]:
        minimum, maximum = min(bins), max(bins)
        spread = max(1, maximum - minimum)
        out = []
        for x in range(width):
            index = int(x * (len(bins) - 1) / max(1, width - 1))
            intensity = (bins[index] - minimum) * 255 // spread
            out.append(
                0xFF000000
                | (intensity << 16)
                | ((80 + intensity * 175 // 255) << 8)
                | (40 + (255 - intensity) * 150 // 255)
            )
        return out

    patterns = [
        bytes(range(256)) * 2,
        bytes((0, 255) * 256),
        bytes((7,)) * 512,
        bytes((i * i // 512) % 256 for i in range(512)),
    ]
    for pattern in patterns:
        for test_width in (1, 2, 3, 511, 512, 513, 900, 1900):
            expected = waterfall_argb_reference(pattern, test_width)
            produced = waterfall_argb(
                np.frombuffer(pattern, dtype=np.uint8).reshape(1, -1), test_width
            )
            assert list(produced[0]) == expected, (test_width, len(pattern))
    # Multiple rows are normalised independently, exactly as the row loop was.
    multi = np.frombuffer(patterns[0] + patterns[3], dtype=np.uint8).reshape(2, 512)
    produced = waterfall_argb(multi, 640)
    assert list(produced[0]) == waterfall_argb_reference(patterns[0], 640)
    assert list(produced[1]) == waterfall_argb_reference(patterns[3], 640)
    assert produced.dtype == np.uint32
    iq_tx = np.empty(48 * 2, dtype="<i2")
    iq_tx[0::2] = 100
    iq_tx[1::2] = -100
    assert len(iq_tx.tobytes()) == 192
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

    # --- IQ encoder offline tests (no radio) ---
    rate = IQ_SAMPLE_RATE

    def encode_stream(audio: np.ndarray, mode: str, offset_hz: int, block: int = 48) -> np.ndarray:
        state = IqEncoderState()
        blocks = [encode_iq_block(state, audio[i : i + block], mode, offset_hz) for i in range(0, len(audio), block)]
        return np.concatenate(blocks)

    duration = 0.6
    time_axis = np.arange(int(rate * duration)) / rate

    # USB must be a true upper sideband: the wanted sideband dominates the
    # image by 40 dB and demodulates with positive polarity.
    tone = 0.3 * np.sin(2 * np.pi * 1000 * time_axis)
    usb_iq = encode_stream(tone, "USB", 12_000)
    usb_baseband = np.conj(usb_iq) * np.exp(-1j * 2 * np.pi * 12_000 * np.arange(len(tone)) / rate)
    usb_spectrum = np.fft.fft(usb_baseband * np.hanning(len(usb_baseband)))
    frequencies = np.fft.fftfreq(len(usb_baseband), 1 / rate)
    wanted_bin = int(np.argmin(np.abs(frequencies - 1000)))
    image_bin = int(np.argmin(np.abs(frequencies + 1000)))
    wanted_power = np.abs(usb_spectrum[wanted_bin])
    image_power = np.abs(usb_spectrum[image_bin])
    assert image_power < wanted_power * 10 ** (-40 / 20), (image_power, wanted_power)
    reference = tone[: len(usb_baseband) - _HILBERT_DELAY]
    measured = usb_baseband.real[_HILBERT_DELAY:]
    correlation = np.corrcoef(measured, reference)[0, 1]
    assert correlation > 0.9, correlation

    # LSB must be a true lower sideband. Without this, the USB assertion above
    # can be satisfied by inverting the Hilbert sign, which mirrors both modes
    # at once so USB transmits LSB and vice versa. Testing only one sideband
    # cannot distinguish a correct encoder from a fully swapped one.
    lsb_iq = encode_stream(tone, "LSB", 12_000)
    lsb_baseband = np.conj(lsb_iq) * np.exp(-1j * 2 * np.pi * 12_000 * np.arange(len(tone)) / rate)
    lsb_spectrum = np.fft.fft(lsb_baseband * np.hanning(len(lsb_baseband)))
    # The wanted and image bins are the mirror of the USB case.
    lsb_wanted_power = np.abs(lsb_spectrum[image_bin])
    lsb_image_power = np.abs(lsb_spectrum[wanted_bin])
    assert lsb_image_power < lsb_wanted_power * 10 ** (-40 / 20), (
        lsb_image_power,
        lsb_wanted_power,
    )
    # Both sidebands carry the audio in the real part with the same polarity, so
    # this also pins in_phase against an accidental overall sign inversion.
    lsb_correlation = np.corrcoef(lsb_baseband.real[_HILBERT_DELAY:], reference)[0, 1]
    assert lsb_correlation > 0.9, lsb_correlation

    # NFM deviation must reach 2 kHz even for quiet microphones, and the FM
    # phase must stay continuous across 48-sample packet boundaries.
    for peak in (0.05, 0.1, 0.3, 0.9):
        fm_tone = peak * np.sin(2 * np.pi * 1000 * time_axis)
        fm_iq = encode_stream(fm_tone, "NFM", 0)
        fm_phase = np.unwrap(np.angle(np.conj(fm_iq)))
        deviation_hz = np.abs(np.diff(fm_phase)) * rate / (2 * np.pi)
        assert np.max(deviation_hz) >= 2000, (peak, np.max(deviation_hz))
        assert np.max(np.abs(np.diff(fm_phase))) < 0.6

    # Full loopback through the actual receive demodulator, simulating the
    # radio's mirror (it conjugates what we transmit).
    loopback = 0.3 * np.sin(2 * np.pi * 500 * time_axis)
    for receive_mode, tx_mode in (("NFM", "NFM"), ("USB", "USB"), ("AM", "AM")):
        outputs: list[np.ndarray] = []
        receiver = SDRReceiver(outputs.append)
        receiver.mode = receive_mode
        receiver.offset_hz = 12_000
        receiver.SSB_OUTPUT_GAIN = 3.0
        receiver.NFM_OUTPUT_GAIN = 3.0
        receiver.AM_OUTPUT_GAIN = 3.0
        receiver.start()
        try:
            tx_iq = encode_stream(loopback, tx_mode, 12_000)
            words = pack_iq_words(tx_iq, False, False)
            complex_words = np.frombuffer(words, dtype="<i2").astype(np.float32).reshape(-1, 2)
            mirrored = np.conj(complex_words[:, 0] + 1j * complex_words[:, 1])
            mirrored_words = np.empty(complex_words.shape, dtype="<i2")
            mirrored_words[:, 0] = np.clip(mirrored.real, -32768, 32767).astype("<i2")
            mirrored_words[:, 1] = np.clip(mirrored.imag, -32768, 32767).astype("<i2")
            flat_words = mirrored_words.reshape(-1)
            block_words = SDRReceiver.BLOCK_FRAMES * 2
            for start in range(0, len(flat_words), block_words):
                receiver.feed(flat_words[start : start + block_words])
            time.sleep(0.15)
            assert outputs, receive_mode
            output_audio = np.concatenate(outputs)
            output_spectrum = np.fft.rfft(output_audio * np.hanning(len(output_audio)))
            output_frequencies = np.fft.rfftfreq(len(output_audio), 1 / rate)
            tone_bin = int(np.argmin(np.abs(output_frequencies - 500)))
            tone_power = np.abs(output_spectrum[tone_bin])
            other_power = np.abs(output_spectrum).copy()
            # Ignore the Hanning mainlobe around the tone and any DC settling
            # below 50 Hz, then compare against the strongest real spur.
            bin_width = rate / len(output_audio)
            guard = int(15 / bin_width)
            other_power[max(0, tone_bin - guard) : tone_bin + guard + 1] = 0
            other_power[: int(50 / bin_width)] = 0
            ratio = tone_power / (np.max(other_power) + 1e-9)
            assert ratio > 10, (receive_mode, ratio)
        finally:
            receiver.stop()

    # Stopping receive audio has to stick. Status frames arrive about twice a
    # second and each one asks whether audio should be started, so a rule that
    # ignores the operator's choice makes the stop button do nothing.
    assert should_autostart_audio(True, "TCP", True, False, False)
    assert should_autostart_audio(True, "USB", True, False, False)
    # Already running: nothing to do, per transport.
    assert not should_autostart_audio(True, "TCP", True, False, True)
    assert not should_autostart_audio(True, "USB", True, True, False)
    # The transports are independent: a running USB monitor must not satisfy the
    # network check, or network audio would never start.
    assert should_autostart_audio(True, "TCP", True, True, False)
    assert should_autostart_audio(True, "USB", True, False, True)
    # Stopped by the operator: never restart, whatever else is true.
    for transport in ("TCP", "USB"):
        for usb_running in (False, True):
            for network_running in (False, True):
                assert not should_autostart_audio(
                    True, transport, False, usb_running, network_running
                ), (transport, usb_running, network_running)
    # Not connected: nothing to start.
    assert not should_autostart_audio(False, "TCP", True, False, False)

    # Rate conversion. The host produces audio on its own clock and the radio
    # consumes on its crystal; pacing can match only one of them, so the other
    # end eventually discards a whole millisecond of audio and clicks. Converting
    # spreads the difference across every sample instead.
    #
    # Every loop below keeps the working buffer small and tops it up, exactly as
    # the sender does. Handing resample_stereo a multi-megabyte buffer instead
    # makes its del pending[:n] quadratic.
    frames_per_packet = NETWORK_TX_PACKET_BYTES // 4

    def tone_frames(count: int, start_index: int, hz: float = 1500.0) -> bytes:
        index = start_index + np.arange(count)
        mono = np.clip(
            np.rint(9000 * np.sin(2 * np.pi * hz * index / 48_000)), -32768, 32767
        ).astype("<i2")
        return np.repeat(mono, 2).tobytes()

    def convert(ratio: float, packets: int, keep: int = 200) -> tuple[bytes, int, int]:
        """Return (output, frames consumed, frames produced) for a steady ratio."""
        buffer = bytearray()
        supplied = 0
        output = bytearray()
        phase = 0.0
        produced = 0
        for _ in range(packets):
            while len(buffer) // 4 < keep:
                buffer += tone_frames(frames_per_packet, supplied)
                supplied += frames_per_packet
            before = len(buffer) // 4
            step = resample_stereo(buffer, frames_per_packet, ratio, phase)
            assert step is not None
            payload, phase = step
            output += payload
            produced += len(payload) // 4
            _ = before
        return bytes(output), supplied - len(buffer) // 4, produced

    # A ratio of exactly one must be bit-identical, so enabling conversion cannot
    # perturb a correctly clocked link. The output starts _RESAMPLE_HISTORY frames
    # into the input, because the filter reaches back that far and those frames
    # serve only as history: a quarter of a millisecond, once, at startup.
    identity, consumed, produced = convert(1.0, 60)
    assert consumed == produced, (consumed, produced)
    assert identity == tone_frames(produced, _RESAMPLE_HISTORY), (
        "ratio 1.0 must be transparent"
    )

    for ratio in (1.0005, 0.9995, 1.002, 0.998):
        _, consumed, produced = convert(ratio, 600)
        assert abs((consumed / produced) / ratio - 1.0) < 5e-4, (ratio, consumed, produced)

    # Left and right must stay identical: the radio expects duplicated mono and
    # interpolation must not decorrelate the pair.
    converted, _, _ = convert(1.0005, 200)
    words = np.frombuffer(converted, dtype="<i2")
    assert np.array_equal(words[0::2], words[1::2])

    # Converted audio must stay spectrally clean. A discarded millisecond is a
    # broadband click; interpolation must not trade it for comparable rubbish.
    for ratio in (1.0005, 1.005):
        converted, _, _ = convert(ratio, 1_200)
        signal = np.frombuffer(converted, dtype="<i2")[0::2].astype(np.float64)
        signal -= signal.mean()
        spectrum = np.abs(np.fft.rfft(signal * np.hanning(len(signal))))
        peak = int(np.argmax(spectrum))
        residue = spectrum.copy()
        residue[max(0, peak - 12) : peak + 13] = 0
        residue[:8] = 0
        assert np.max(residue) < spectrum[peak] * 10 ** (-45 / 20), (
            ratio, 20 * np.log10(np.max(residue) / spectrum[peak]),
        )

    # Channel order must survive interpolation. A duplicated-mono test signal
    # cannot detect a left/right swap, so use distinct channels here. The output
    # begins _RESAMPLE_HISTORY frames in, because the filter needs that much
    # history, so compare against the input from there.
    distinct = bytearray()
    for frame in range(400):
        distinct += int(frame % 1000).to_bytes(2, "little", signed=True)
        distinct += int(-(frame % 1000)).to_bytes(2, "little", signed=True)
    step = resample_stereo(distinct, frames_per_packet, 1.0, 0.0)
    assert step is not None
    swapped = np.frombuffer(step[0], dtype="<i2")
    first = _RESAMPLE_HISTORY
    assert swapped[0] == first and swapped[1] == -first, swapped[:4]
    assert swapped[2] == first + 1 and swapped[3] == -(first + 1), swapped[:4]

    # Pacing must follow the radio, and fall back cleanly when it is unknown.
    assert tx_pacing(0.0) == (NETWORK_TX_PERIOD, 1.0)
    for radio_rate in (999.3, 1_000.0, 1_000.7):
        period, base_ratio = tx_pacing(radio_rate)
        assert abs(1.0 / period - radio_rate) < 1e-9, radio_rate
        # One second of host audio must convert to exactly one second of radio
        # packets, which is what stops either buffer from drifting.
        assert abs(base_ratio * radio_rate - 1.0 / NETWORK_TX_PERIOD) < 1e-6, radio_rate

    # The servo must push back in the right direction and with a bounded trim.
    # Its error is now smoothed, so a single call barely moves: drive it for a
    # while and compare where it settles.
    target = NETWORK_TX_LOW_WATER_PACKETS * frames_per_packet

    def drive(depth: int, packets: int, base: float = 1.0) -> tuple[float, float]:
        """Hold `depth` constant for `packets` and return (ratio, trim)."""
        trim, smooth = 0.0, (0.0, 0.0)
        ratio = base
        for _ in range(packets):
            ratio, trim, smooth = resample_ratio(depth, target, trim, base, smooth)
        return ratio, trim

    deep, _ = drive(target * 2, 20_000)
    shallow, _ = drive(target // 2, 20_000)
    steady, steady_trim = drive(target, 20_000)
    assert deep > steady > shallow, (deep, steady, shallow)
    assert abs(steady - 1.0) < 1e-12 and steady_trim == 0.0
    # A single call must be a small correction, not a lurch: that is the property
    # that stops buffer granularity from frequency-modulating the audio.
    once, _, _ = resample_ratio(target * 2, target, 0.0, 1.0)
    assert abs(once - 1.0) < 1e-5, once
    # The trim must stay bounded however long the error persists.
    runaway, smooth = 0.0, (0.0, 0.0)
    for _ in range(200_000):
        _, runaway, smooth = resample_ratio(target * 10, target, runaway, 1.0, smooth)
    assert runaway <= RESAMPLE_TRIM_LIMIT + 1e-12, runaway

    # With the base ratio deliberately wrong, only the servo can stop the buffer
    # from running away. Model consumption arithmetically so this stays quick.
    for host_hz, radio_rate in ((48_024.0, 999.5), (47_976.0, 1_000.6)):
        emit_period = 1.0 / radio_rate
        depth = float(target)
        trim, smooth = 0.0, (0.0, 0.0)
        deepest, shallowest = depth, depth
        for _ in range(int(600.0 / emit_period)):
            ratio, trim, smooth = resample_ratio(int(depth), target, trim, 1.0, smooth)
            depth += host_hz * emit_period - ratio * frames_per_packet
            deepest, shallowest = max(deepest, depth), min(shallowest, depth)
        assert shallowest > frames_per_packet, (host_hz, radio_rate, shallowest)
        assert deepest < NETWORK_TX_HIGH_WATER_PACKETS * frames_per_packet, (
            host_hz, radio_rate, deepest,
        )
        # And it must actually settle near the target rather than merely staying
        # inside the limits.
        assert abs(depth - target) < target * 0.5, (host_hz, radio_rate, depth, target)

    # Depth-to-ratio is a double integrator, so the loop needs damping as well as
    # integral action. Start the buffer at twice its target and check it does not
    # ring down through empty.
    depth, trim, smooth = float(target * 2), 0.0, (0.0, 0.0)
    shallowest = depth
    for _ in range(900_000):
        ratio, trim, smooth = resample_ratio(int(depth), target, trim, 1.0, smooth)
        depth += 48_000.0 * NETWORK_TX_PERIOD - ratio * frames_per_packet
        shallowest = min(shallowest, depth)
    assert shallowest > frames_per_packet * 20, shallowest
    assert abs(depth - target) < target * 0.5, depth

    # The whole point of the smoothing: capture arrives in 20 ms blocks, so the
    # depth is a sawtooth one block deep. That granularity must not reach the
    # conversion ratio, because a ratio that moves at the block rate frequency-
    # modulates the transmitted audio. Measured at -28 dB on a real transmission
    # before this filter existed.
    ratios = []
    depth = float(target + TransmitAudioRouter.BLOCK_SIZE // 2)
    trim, smooth = 0.0, (0.0, 0.0)
    for packet in range(60_000):
        if packet % 20 == 0:
            depth += TransmitAudioRouter.BLOCK_SIZE
        ratio, trim, smooth = resample_ratio(int(depth), target, trim, 1.0, smooth)
        depth -= ratio * frames_per_packet
        ratios.append(ratio)
    settled = np.array(ratios[20_000:])
    wobble = settled / settled.mean() - 1.0
    # Isolate the block rate and above, where the audible sidebands were.
    spectrum = np.abs(np.fft.rfft(wobble * np.hanning(len(wobble))))
    spectrum *= 2.0 / np.sum(np.hanning(len(wobble)))
    bins = np.fft.rfftfreq(len(wobble), NETWORK_TX_PERIOD)
    audible = spectrum[(bins >= 2.0) & (bins <= 200.0)]
    assert np.sqrt(np.sum(audible ** 2) / 2) < 5e-6, float(
        np.sqrt(np.sum(audible ** 2) / 2) * 1e6
    )

    # The radio's media clock is measured from packet arrivals, and that figure
    # decides whether transmit audio has to be re-paced. Averaging over the whole
    # session made it climb towards the truth forever without settling, because
    # the stream pauses while transmitting and does not start with the socket.
    # Exercise the real accumulator, not a copy of it.
    class _ClockSignals:
        class _Slot:
            def emit(self, *args: object) -> None:
                pass

        audio_state_changed = _Slot()

    def measure_clock(arrivals_ns: list[int]) -> tuple[float, float]:
        monitor = NetworkAudioMonitor(_ClockSignals())
        for stamp in arrivals_ns:
            monitor._packet_count += 1
            monitor._note_arrival(stamp)
        return monitor.measured_packet_rate, monitor._clock_best_seconds

    def arrivals(start_ns: float, seconds: float, rate: float) -> list[int]:
        return [int(start_ns + i / rate * 1e9) for i in range(int(seconds * rate))]

    for true_rate in (998.10, 1000.00, 1001.30):
        clean = arrivals(0, 40, true_rate)
        # A stray packet then a long idle period must not drag the estimate.
        stray = [0] + arrivals(30e9, 40, true_rate)
        # A transmit pause splits the run; the surviving run is still exact.
        paused = arrivals(0, 25, true_rate) + arrivals(45e9, 40, true_rate)
        # A stalled reader does not shift later arrivals: the backlog drains and
        # the stream catches up, so the span is unchanged and only the interior
        # pacing is disturbed. Model it that way, not as a step in the timeline.
        def stall_reads(series: list[int], at_ns: list[float], held_ns: int) -> list[int]:
            delayed = list(series)
            for start in at_ns:
                delayed = [
                    max(stamp, int(start + held_ns))
                    if start <= stamp < start + held_ns
                    else stamp
                    for stamp in delayed
                ]
            return sorted(delayed)

        stalled = stall_reads(arrivals(0, 40, true_rate), [10e9], 30_000_000)
        # The observed failure mode: reads stall many times a second. Breaking
        # the run on each one never accumulates a usable window, so the estimate
        # must survive this.
        many = stall_reads(
            arrivals(0, 40, true_rate),
            [float(x) * 1e9 for x in range(1, 40)],
            25_000_000,
        )
        # If the radio sends its media in groups rather than evenly, both
        # endpoints must land on group boundaries. Endpoints falling mid-group
        # understate the span by up to one group, biasing the rate by the group
        # duration over the window: 32 ms in 40 s is 800 ppm.
        def grouped(seconds: float, rate: float, size: int) -> list[int]:
            period_ns = 1e9 / rate
            out: list[int] = []
            for index in range(int(seconds * rate) // size):
                base = index * size * period_ns
                out.extend(int(base + offset * 20_000) for offset in range(size))
            return out

        for label, series in (("clean", clean), ("stray+idle", stray),
                              ("paused", paused), ("stalled", stalled),
                              ("many stalls", many),
                              ("groups of 8", grouped(40, true_rate, 8)),
                              ("groups of 32", grouped(40, true_rate, 32)),
                              # Alignment points must not survive a pause. A
                              # boundary from before it paired with one after
                              # would span the pause with a packet count that
                              # excludes it, understating the rate several fold.
                              ("stalls either side of a pause",
                               stall_reads(arrivals(0, 10, true_rate),
                                           [float(x) * 1e9 for x in range(1, 10)],
                                           25_000_000)
                               + stall_reads(arrivals(60e9, 40, true_rate),
                                             [60e9 + float(x) * 1e9 for x in range(1, 40)],
                                             25_000_000))):
            measured, _ = measure_clock(series)
            assert measured > 0, (label, true_rate)
            error_ppm = abs(measured - true_rate) / true_rate * 1e6
            assert error_ppm < 50, (label, true_rate, measured, error_ppm)
        # A long run followed by a short one must still report the long run.
        # Discarding it on every break would lose the only usable measurement.
        long_then_short = arrivals(0, 40, true_rate) + arrivals(60e9, 3, true_rate)
        measured, best_seconds = measure_clock(long_then_short)
        assert best_seconds > 30, best_seconds
        assert abs(measured - true_rate) / true_rate * 1e6 < 50, (measured, true_rate)
        # A stalled receive thread draining its backlog delivers a burst of
        # packets with near-identical arrival stamps. Counting those would add
        # packet count with no elapsed time and over-read the rate by about
        # 1000 ppm, so the burst has to end the run instead.
        held_ns = 25_000_000
        burst_at = 15e9
        bursty = sorted(
            max(stamp, burst_at + held_ns)
            if burst_at <= stamp < burst_at + held_ns
            else stamp
            for stamp in arrivals(0, 40, true_rate)
        )
        measured, _ = measure_clock(bursty)
        assert measured > 0, "clean stretches either side must still measure"
        assert abs(measured - true_rate) / true_rate * 1e6 < 50, (measured, true_rate)
    # Too little data must report nothing rather than a wrong number.
    assert measure_clock(arrivals(0, 1, 1000.0))[0] == 0.0

    # The receive demodulator must actually select a sideband. A product
    # detector taking baseband.real passes both sides equally, which makes the
    # RX mode selector inert and hides transmit-side sideband errors from anyone
    # listening on this app. Feed a single complex exponential placed strictly
    # above or below the suppressed carrier and require real rejection.
    def demodulate_offset_tone(receive_mode: str, tone_offset_hz: int) -> np.ndarray:
        samples = int(rate * 0.5)
        axis = np.arange(samples) / rate
        wave = 0.4 * np.exp(1j * 2 * np.pi * (12_000 + tone_offset_hz) * axis)
        words = np.empty(samples * 2, dtype="<i2")
        words[0::2] = np.clip(wave.real * 32767, -32768, 32767).astype("<i2")
        words[1::2] = np.clip(wave.imag * 32767, -32768, 32767).astype("<i2")
        collected: list[np.ndarray] = []
        receiver = SDRReceiver(collected.append)
        receiver.mode = receive_mode
        receiver.offset_hz = 12_000
        receiver.SSB_OUTPUT_GAIN = 1.0
        receiver.start()
        try:
            block_words = SDRReceiver.BLOCK_FRAMES * 2
            for start in range(0, len(words), block_words):
                receiver.feed(words[start : start + block_words])
            time.sleep(0.25)
        finally:
            receiver.stop()
        return np.concatenate(collected) if collected else np.zeros(1, dtype=np.float32)

    def tone_power_at(audio: np.ndarray, hz: float) -> float:
        spectrum = np.abs(np.fft.rfft(audio * np.hanning(len(audio))))
        bins = np.fft.rfftfreq(len(audio), 1 / rate)
        return float(spectrum[int(np.argmin(np.abs(bins - hz)))])

    for receive_mode in ("USB", "LSB"):
        above = demodulate_offset_tone(receive_mode, 1_000)
        below = demodulate_offset_tone(receive_mode, -1_000)
        upper = tone_power_at(above, 1_000)
        lower = tone_power_at(below, 1_000)
        wanted_audio = above if receive_mode == "USB" else below
        wanted, image = (upper, lower) if receive_mode == "USB" else (lower, upper)
        assert wanted > 0, receive_mode
        assert image < wanted * 10 ** (-30 / 20), (receive_mode, image, wanted)
        # Every filter in this path must carry state across blocks. A per-block
        # mode="same" convolution zero-pads both edges, which puts a
        # discontinuity at each boundary and raises a comb at the block rate
        # (48000/960 = 50 Hz). That measured -53 dB before the carried state was
        # added and -138 dB after.
        block_rate = rate / SDRReceiver.BLOCK_FRAMES
        worst_spur = max(
            tone_power_at(wanted_audio, block_rate * harmonic) for harmonic in range(1, 7)
        )
        assert worst_spur < wanted * 10 ** (-80 / 20), (receive_mode, worst_spur, wanted)

    # The encoder bakes the radio mirror in by default. `Swap I/Q` sends
    # j*conj(z) and `Invert Q` sends conj(z), so each mirrors the whole 48 kHz
    # stream about its own DC rather than about the tuned carrier. At the default
    # +12 kHz offset that moves the signal 24 kHz, aliasing to +23 kHz, instead
    # of exchanging sidebands. Applying both cancels back to the default.
    def mirrored_spectrum(swap: bool, invert: bool) -> np.ndarray:
        packed = pack_iq_words(encode_stream(tone, "USB", 12_000), swap, invert)
        packed_complex = np.frombuffer(packed, dtype="<i2").astype(np.float32).reshape(-1, 2)
        baseband = np.conj(packed_complex[:, 0] + 1j * packed_complex[:, 1])
        baseband = baseband * np.exp(-1j * 2 * np.pi * 12_000 * np.arange(len(tone)) / rate)
        return np.fft.fft(baseband * np.hanning(len(baseband)))

    default_wanted = np.abs(mirrored_spectrum(False, False)[wanted_bin])
    for swap, invert in ((True, False), (False, True)):
        moved = mirrored_spectrum(swap, invert)
        peak_hz = frequencies[int(np.argmax(np.abs(moved)))]
        assert abs(peak_hz - 23_000) < 100, (swap, invert, peak_hz)
        # Assert the tone has left the carrier region entirely. Comparing the
        # two sideband bins against each other would pass on leakage alone.
        assert np.abs(moved[wanted_bin]) < default_wanted * 1e-4, (swap, invert)
        assert np.abs(moved[image_bin]) < default_wanted * 1e-4, (swap, invert)
    identity_spectrum = mirrored_spectrum(True, True)
    assert np.abs(identity_spectrum[wanted_bin]) > np.abs(identity_spectrum[image_bin]) * 100, (
        np.abs(identity_spectrum[image_bin]),
        np.abs(identity_spectrum[wanted_bin]),
    )
    print("Q900 protocol self-test passed")


def _analytic(signal: np.ndarray) -> np.ndarray:
    """Analytic signal of a real block, via the one-sided FFT.

    Used by the recording analysers to recover instantaneous phase and envelope.
    A tone's unwrapped phase is a straight line, and a sample the stream gains or
    loses displaces it by one sample's worth of phase advance, 2*pi*f0/fs. So a
    splice is measurable even when it is far too small to show up as a
    sample-to-sample step. Uniformly distributed slips tilt the line instead of
    displacing it, which is why the analyser reports both the residual and the
    phase-implied frequency.
    """
    length = len(signal)
    transform = np.fft.fft(signal)
    transform[length // 2 + 1:] = 0
    transform[1:(length + 1) // 2] *= 2
    return np.fft.ifft(transform)


def analyze_tx_recording(prefix: str) -> None:
    """Report defects in a Q900_TX_RECORD capture of the transmitted stream.

    The recording is exactly what left the socket, so it separates a host-side
    defect from a radio-side or network-side one. If this reports a clean stream,
    nothing above the socket is responsible for what is heard on the air.
    """
    rate = 48_000
    def cluster(indices: np.ndarray, gap: int = 64) -> np.ndarray:
        """Collapse runs of adjacent detections into one event each.

        A single splice trips several neighbouring samples, so the raw counts
        would report a rate that is a multiple of the real one. The repetition
        rate is the most useful clue available, so it has to be right.
        """
        if not len(indices):
            return indices
        breaks = np.flatnonzero(np.diff(indices) > gap)
        return indices[np.concatenate(([0], breaks + 1))]

    def report_events(indices: np.ndarray, label: str) -> None:
        events = cluster(indices)
        print(f"  {label}: {len(events):,} event(s) "
              f"(from {len(indices):,} flagged samples)")
        if not len(events):
            return
        seconds = events / rate
        print(f"  first 10 times (s): "
              f"{', '.join(f'{value:.4f}' for value in seconds[:10])}")
        if len(events) > 1:
            spacing = np.diff(seconds)
            print(f"  spacing: median {np.median(spacing) * 1000:.2f} ms "
                  f"-> {1 / np.median(spacing):.2f} Hz   "
                  f"min {spacing.min() * 1000:.2f} ms  max {spacing.max() * 1000:.2f} ms")
        for name, size in (("packet", NETWORK_TX_PACKET_BYTES // 4),
                           ("mic block", TransmitAudioRouter.BLOCK_SIZE)):
            offsets = events % size
            print(f"  aligned to {name} boundary ({size} frames): "
                  f"{int(np.count_nonzero(offsets == 0)):,} exactly, "
                  f"{len(np.unique(offsets))} distinct offset(s)")

    try:
        with open(f"{prefix}.tx.raw", "rb") as handle:
            raw = handle.read()
    except OSError as error:
        print(f"cannot read {prefix}.tx.raw: {error}")
        return
    try:
        with open(f"{prefix}.tx.time", "rb") as handle:
            stamps = np.frombuffer(handle.read(), dtype="<u8")
    except OSError:
        stamps = np.zeros(0, dtype="<u8")

    words = np.frombuffer(raw[: len(raw) // 4 * 4], dtype="<i2")
    left, right = words[0::2].astype(np.int32), words[1::2].astype(np.int32)
    frames = len(left)
    packets = len(raw) // NETWORK_TX_PACKET_BYTES
    print(f"transmitted stream : {len(raw):,} B  {packets:,} packets  "
          f"{frames / rate:.2f} s at {rate} Hz")

    print("\n-- framing --")
    print(f"  size is a whole number of packets : {len(raw) % NETWORK_TX_PACKET_BYTES == 0}")
    mismatched = int(np.count_nonzero(left != right))
    print(f"  L != R frames (mono is duplicated): {mismatched:,}")

    print("\n-- inserted silence (underrun substitution) --")
    silent = np.all(
        words[: packets * (NETWORK_TX_PACKET_BYTES // 2)].reshape(packets, -1) == 0, axis=1
    )
    runs = int(np.count_nonzero(np.diff(silent.astype(np.int8)) == 1)) + int(silent[:1].sum())
    print(f"  fully silent packets : {int(silent.sum()):,} of {packets:,}  in {runs} run(s)")

    print("\n-- sample continuity (a splice is a broadband click) --")
    # A discontinuity shows as a first difference far outside the local
    # distribution. Compare against a robust scale so a loud passage does not
    # mask a click and a quiet one does not manufacture them.
    diff = np.diff(left)
    scale = float(np.median(np.abs(diff))) or 1.0
    threshold = max(8.0 * scale, 64.0)
    events = np.flatnonzero(np.abs(diff) > threshold)
    print(f"  median |step| {scale:.1f}   threshold {threshold:.1f}")
    report_events(events, "sample-step outliers")

    print("\n-- phase continuity (catches splices the step test misses) --")
    # FT8 is a single tone at any instant, so a lost or repeated sample shows as
    # a phase discontinuity even when the sample-to-sample step stays small.
    # This is the sensitive test for tonal transmissions.
    phase_events: np.ndarray = np.zeros(0, dtype=np.int64)
    signal = left.astype(np.float64)
    signal -= signal.mean()
    if frames > 4096 and np.any(signal):
        spectrum = np.abs(np.fft.rfft(signal * np.hanning(frames))) ** 2
        dominant = int(np.argmax(spectrum))
        tonality = float(spectrum[dominant] / (spectrum.sum() + 1e-30))
        print(f"  dominant {dominant * rate / frames:8.1f} Hz   tonality {tonality:.4f}")
        if tonality > 0.005:
            analytic = _analytic(signal)
            step = np.angle(analytic[1:] * np.conj(analytic[:-1]))
            centre = float(np.median(step))
            deviation = np.abs(step - centre)
            robust = float(np.median(deviation)) or 1e-9
            phase_events = np.flatnonzero(deviation > max(25.0 * robust, 0.30))
            # The FFT-based analytic signal rings at both ends of the record.
            # Those edges are an artefact of the measurement, not the stream.
            edge = 256
            phase_events = phase_events[
                (phase_events >= edge) & (phase_events < len(step) - edge)
            ]
            report_events(phase_events, "phase-step outliers")
        else:
            print("  not tonal enough for this test; rely on the step test above")
    else:
        print("  recording too short or silent")

    tone_clean = True
    # Restrict the tone measurements to the longest sustained run of audio.
    # A capture normally opens and closes with silence -- the ring drain, and
    # whatever was keyed but unspoken -- and silence has no envelope and no phase,
    # so including it manufactures enormous ripple and slip figures that say
    # nothing about the tone itself.
    voiced = np.flatnonzero(~silent)
    if len(voiced):
        splits = np.flatnonzero(np.diff(voiced) > 1)
        runs = np.split(voiced, splits + 1)
        longest = max(runs, key=len)
        span = slice(
            int(longest[0]) * (NETWORK_TX_PACKET_BYTES // 4),
            (int(longest[-1]) + 1) * (NETWORK_TX_PACKET_BYTES // 4),
        )
    else:
        span = slice(0, 0)
    sustained = signal[span]
    # Then restrict to the steady part of it. A Tune transmission ramps its tone up
    # and down on purpose to avoid key clicks, and a ramp inside the analysis
    # window is a genuine amplitude modulation: it puts sidebands within a few Hz
    # of the carrier and dominates any envelope figure, which is how a tone that is
    # flat to half a per cent reads as a hundred per cent modulated. Keep the
    # longest run that stays near the median level, then stand clear of its edges
    # so the analytic transform's own ringing is excluded too.
    if len(sustained) > 8192:
        envelope = np.abs(_analytic(sustained))
        level = float(np.median(envelope))
        if level > 0.0:
            inside = np.flatnonzero(envelope > 0.7 * level)
            if len(inside):
                cuts = np.flatnonzero(np.diff(inside) > 1)
                longest_run = max(np.split(inside, cuts + 1), key=len)
                margin = 512
                start = int(longest_run[0]) + margin
                stop = int(longest_run[-1]) + 1 - margin
                if stop - start > 8192:
                    sustained = sustained[start:stop]
    if len(sustained) > 8192 and np.any(sustained):
        spectrum = np.abs(np.fft.rfft(sustained * np.hanning(len(sustained)))) ** 2
        dominant = int(np.argmax(spectrum))
        tonality = float(spectrum[dominant] / (spectrum.sum() + 1e-30))
        if tonality > 0.05:
            print("\n-- steady tone analysis --")
            print("  A single tone makes every defect measurable. Drive this with")
            print("  WSJT-X 'Tune' or any constant carrier: distortion, amplitude")
            print("  modulation and lost or repeated samples all separate cleanly,")
            print("  which speech cannot do.")
            print(f"  measured over the longest unbroken run: "
                  f"{len(sustained) / rate:.2f} s of {frames / rate:.2f} s")
            # Work on a whole number of bins so the harmonic search is exact.
            usable = len(sustained) - (len(sustained) % 2)
            block = sustained[:usable] - float(np.mean(sustained[:usable]))
            window = np.hanning(usable)
            mag = np.abs(np.fft.rfft(block * window))
            freqs = np.fft.rfftfreq(usable, 1.0 / rate)
            peak = int(np.argmax(mag))
            f0 = float(freqs[peak])
            total = float(np.sum(mag**2))

            def band_power(centre: float, width: float = 30.0) -> float:
                sel = np.abs(freqs - centre) <= width
                return float(np.sum(mag[sel] ** 2))

            fundamental = band_power(f0)
            harmonics = 0.0
            lines = []
            for n in range(2, 11):
                fn = f0 * n
                if fn >= rate / 2:
                    break
                power = band_power(fn)
                harmonics += power
                lines.append((n, fn, 10 * np.log10(power / fundamental + 1e-30)))
            thd = 10 * np.log10(harmonics / fundamental + 1e-30)
            residual = total - fundamental - harmonics
            snr = 10 * np.log10(fundamental / (residual + 1e-30))
            print(f"  fundamental      {f0:9.3f} Hz")
            print(f"  THD              {thd:+9.1f} dB  (all harmonics vs fundamental)")
            print(f"  SNR              {snr:+9.1f} dB  (everything else vs fundamental)")
            for n, fn, level in lines[:5]:
                print(f"    harmonic {n} at {fn:8.1f} Hz  {level:+7.1f} dB")
            # Nearby spurs, not harmonics, are what a rate converter leaves behind.
            # A two-tap interpolator's response depends on its fractional phase, so
            # with the phase walking it modulates the audio and puts a comb around
            # the tone spaced at the wrap rate: |ratio - 1| * 48000 Hz. Reporting
            # the spacing identifies the mechanism, because that figure is the
            # conversion ratio expressed in ppm.
            skirt = 8.0
            near = (np.abs(freqs - f0) > skirt) & (np.abs(freqs - f0) < 500.0)
            if np.any(near):
                worst = float(np.max(mag[near]))
                at = float(freqs[near][int(np.argmax(mag[near]))])
                level = 20 * np.log10(worst / (np.max(mag) + 1e-30) + 1e-30)
                spacing = at - f0
                print(f"  worst spur       {level:+9.1f} dB  at {at:.2f} Hz "
                      f"({spacing:+.2f} Hz from the tone)")
                if abs(spacing) > 0.5:
                    print(f"    a comb at this spacing is rate-conversion residue: "
                          f"{abs(spacing) / rate * 1e6:.0f} ppm")
            else:
                level = -999.0
            # Envelope stability. Measure over the interior with percentiles: the
            # onset of the tone is a legitimate step, and letting it into a
            # peak-to-peak figure reports a flat tone as 109 per cent modulated.
            analytic_mag = np.abs(_analytic(block))
            core = analytic_mag[1024:-1024] if len(analytic_mag) > 4096 else analytic_mag
            envelope_ripple = float(
                (np.percentile(core, 99.9) - np.percentile(core, 0.1))
                / (np.mean(core) + 1e-30)
            )
            print(f"  envelope ripple  {envelope_ripple * 100:8.3f} %  "
                  f"(0.1..99.9 percentile of the interior)")
            # Lost or repeated samples. For a steady tone the unwrapped phase is a
            # straight line; a sample gained or lost displaces it by one sample's
            # worth of advance. Fitting the line and measuring the residual counts
            # isolated splices, which no amplitude test can do. Slips spread evenly
            # through the record tilt the line instead, so the fitted frequency is
            # reported next to the FFT peak: a gap between them is the signature of
            # a steady drip of corrections rather than a few discrete events.
            phase = np.unwrap(np.angle(_analytic(block)))
            index = np.arange(len(phase), dtype=np.float64)
            trim = slice(256, len(phase) - 256)
            slope, offset = np.polyfit(index[trim], phase[trim], 1)
            resid = phase[trim] - (slope * index[trim] + offset)
            samples_slipped = float(np.ptp(resid) / (2 * np.pi) * (rate / max(f0, 1e-9)))
            implied = slope * rate / (2 * np.pi)
            print(f"  phase-implied f0 {implied:9.3f} Hz  "
                  f"({(implied / max(f0, 1e-9) - 1) * 1e6:+.0f} ppm vs FFT peak)")
            print(f"  net sample slip  {samples_slipped:8.2f} samples across the record")
            tone_clean = (
                thd < -55.0 and snr > 45.0 and envelope_ripple < 0.01 and level < -75.0
            )
            print(f"  tone verdict     {'clean' if tone_clean else 'DEGRADED'}")

    if len(stamps) > 1:
        print("\n-- send pacing --")
        gaps = np.diff(stamps.astype(np.int64)) / 1e6
        print(f"  inter-packet: median {np.median(gaps):.3f} ms  "
              f"p99 {np.percentile(gaps, 99):.3f} ms  max {gaps.max():.3f} ms")
        stalls = np.flatnonzero(gaps > 5.0)
        print(f"  stalls over 5 ms: {len(stalls):,}")
        if len(stalls):
            at = stamps[stalls] - stamps[0]
            print(f"  first 10 stall times (s): "
                  f"{', '.join(f'{value / 1e9:.4f}' for value in at[:10])}")
        elapsed = (int(stamps[-1]) - int(stamps[0])) / 1e9
        print(f"  achieved rate: {(len(stamps) - 1) / elapsed:.2f} packets/s "
              f"(nominal {1 / NETWORK_TX_PERIOD:.0f})")
        # The run opens with a priming burst paced at NETWORK_TX_BURST_GAP, and is
        # preceded by the ring drain. Both are one-offs, so including them
        # understates the steady-state rate over a short recording.
        paced_floor = NETWORK_TX_BURST_GAP * 1000 * 1.5
        primed = NETWORK_TX_PRIME_PACKETS
        steady = stamps[primed:] if len(stamps) > primed + 2 else stamps
        if len(steady) > 2:
            span = (int(steady[-1]) - int(steady[0])) / 1e9
            rate_steady = (len(steady) - 1) / span
            print(f"  after the {primed}-packet priming burst: {rate_steady:.2f} packets/s "
                  f"over {span:.2f} s")
            paced = gaps[gaps > paced_floor]
            if len(paced):
                print(f"    median paced interval {np.median(paced):.4f} ms "
                      f"-> {1000 / np.median(paced):.2f} packets/s")

    print("\n-- verdict --")
    clean = (
        not mismatched
        and not int(silent.sum())
        and len(events) == 0
        and len(phase_events) == 0
        and tone_clean
    )
    if clean:
        print("  The stream that left this host is clean: contiguous samples, no")
        print("  inserted silence, correct framing. A defect heard on the air is")
        print("  therefore radio-side or network-side, not in this application.")
    else:
        print("  Defects are present in the stream before it leaves the host.")
        print("  Use the alignment and spacing figures above to localise them.")


def analyze_rx_recording(prefix: str) -> None:
    """Report the arrival pattern of the radio's media stream.

    The question this answers is whether large inter-arrival gaps come from the
    radio sending in bursts or from this host failing to read the socket in time.
    They need opposite fixes, and the timestamps alone distinguish them: a host
    stall leaves a backlog that drains as a run of near-zero intervals straight
    after the gap, whereas a radio burst puts the near-zero intervals before it.
    """
    try:
        with open(f"{prefix}.rx.time", "rb") as handle:
            raw = handle.read()
    except OSError as error:
        print(f"cannot read {prefix}.rx.time: {error}")
        return
    records = len(raw) // 12
    if records < 3:
        print(f"only {records} packets recorded; nothing to analyse")
        return
    block = np.frombuffer(
        raw[: records * 12],
        dtype=np.dtype([("ns", "<u8"), ("size", "<u2"), ("type", "<u2")]),
    )
    stamps = block["ns"].astype(np.int64)
    sizes, types = block["size"], block["type"]
    span = (stamps[-1] - stamps[0]) / 1e9
    print(f"received {records:,} packets over {span:.2f} s")
    print(f"  overall rate      : {(records - 1) / span:.2f} pkt/s")
    frames = NETWORK_TX_PACKET_BYTES // 4
    print(f"  implied sample rate: {(records - 1) / span * frames:,.0f} Hz "
          f"(payload {int(np.median(sizes))} B, "
          f"types {', '.join(hex(int(t)) for t in np.unique(types))})")

    delta = np.diff(stamps) / 1e6          # milliseconds
    print("\n-- inter-arrival distribution (ms) --")
    for lo, hi, label in ((0, 0.05, "< 0.05  (same instant: burst or backlog)"),
                          (0.05, 0.5, "0.05-0.5"),
                          (0.5, 1.5, "0.5-1.5  (paced ~1 ms)"),
                          (1.5, 4, "1.5-4"),
                          (4, 20, "4-20"),
                          (20, 50, "20-50"),
                          (50, 1e12, "> 50    (stream pause)")):
        count = int(np.count_nonzero((delta >= lo) & (delta < hi)))
        print(f"  {label:42s} {count:8,d}  ({count / len(delta) * 100:5.2f}%)")
    print(f"  median {np.median(delta):.3f} ms   p99 {np.percentile(delta, 99):.3f} ms   "
          f"max {delta.max():.1f} ms")

    nominal_ms = NETWORK_TX_PERIOD * 1000.0
    paced = int(np.count_nonzero((delta >= 0.5 * nominal_ms) & (delta <= 1.5 * nominal_ms)))
    grouped = int(np.count_nonzero(delta < 0.05))
    paced_fraction = paced / len(delta)
    big = np.flatnonzero(delta > 4.0)

    print("\n-- who is responsible for the gaps --")
    print(f"  intervals near the {nominal_ms:.0f} ms cadence : {paced_fraction * 100:5.1f}%")
    print(f"  intervals back-to-back (< 0.05 ms)  : {grouped / len(delta) * 100:5.1f}%")
    if len(big):
        print(f"  gaps over 4 ms                      : {len(big):,} "
              f"({len(big) / span:.1f}/s)")
        group = np.diff(np.concatenate(([-1], big)))
        print(f"  packets between gaps                : median {np.median(group):.0f}")

    # A radio that sends in groups produces almost no normally-paced intervals:
    # everything is either back-to-back within a group or the gap between groups.
    # A starved reader interrupts an otherwise paced stream, so most intervals
    # remain at the cadence and only the stalls stand out.
    if not len(big):
        diagnosis = "smooth"
    elif paced_fraction < 0.2 and grouped > paced:
        diagnosis = "radio-groups"
    else:
        diagnosis = "host-starved"

    print("\n-- clock estimate --")
    nominal = 1.0 / NETWORK_TX_PERIOD
    def report(rate: float, seconds: float, label: str) -> None:
        print(f"  {label}: {rate:.2f} pkt/s = {rate * frames:,.0f} Hz "
              f"({(rate / nominal - 1.0) * 1e6:+.0f} ppm) over {seconds:.2f} s")

    if diagnosis == "radio-groups":
        # Grouping is the radio's normal behaviour, so the average over the whole
        # recording is the meaningful figure; per-group runs are far too short.
        report((records - 1) / span, span, "whole recording")
    else:
        breaks = np.flatnonzero((delta > 4.0) | (delta <= 0.0))
        edges = np.concatenate(([0], breaks + 1, [len(stamps)]))
        best = (0, 0.0, 0.0)
        for run_start, run_end in zip(edges[:-1], edges[1:]):
            if run_end - run_start < 2:
                continue
            seconds = (stamps[run_end - 1] - stamps[run_start]) / 1e9
            if seconds > best[1]:
                best = (run_end - run_start, seconds,
                        (run_end - run_start - 1) / seconds if seconds else 0.0)
        packets, seconds, rate = best
        if packets >= 2 and seconds > 0:
            report(rate, seconds, "longest clean run")
            if seconds < 5:
                print("  run far too short to trust: the offset being sought is a "
                      "few hundred ppm")
        else:
            print("  no usable clean run")
        report((records - 1) / span, span, "whole recording  ")

    print("\n-- verdict --")
    if diagnosis == "smooth":
        print("  Smoothly paced. The rate above is the radio's media clock.")
    elif diagnosis == "radio-groups":
        print("  The radio sends its media in groups, which is its own pacing and")
        print("  not a fault. The whole-recording rate is the clock figure, and the")
        print("  live estimator must tolerate grouping instead of breaking on it.")
    else:
        print("  An otherwise paced stream is being interrupted, so this host is")
        print("  not reading the socket in time. That invalidates the arrival")
        print("  timestamps as a clock reference until the receive path is fixed.")


def main() -> None:
    if "--self-test" in sys.argv:
        self_test()
        return
    if "--analyze-rx" in sys.argv:
        index = sys.argv.index("--analyze-rx")
        if index + 1 >= len(sys.argv):
            print("usage: q900_control.py --analyze-rx <prefix>")
            return
        analyze_rx_recording(sys.argv[index + 1])
        return
    if "--analyze-tx" in sys.argv:
        index = sys.argv.index("--analyze-tx")
        if index + 1 >= len(sys.argv):
            print("usage: q900_control.py --analyze-tx <prefix>")
            return
        analyze_tx_recording(sys.argv[index + 1])
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
