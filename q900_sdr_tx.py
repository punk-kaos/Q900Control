#!/usr/bin/env python3
"""Improved Q900 SDR transmit DSP and pacing.

This module is intentionally small and installed by q900_control.py so the
large historical core stays untouched.  It fixes the SSB-specific DSP and the
I/Q sender's capture/clocking path without changing receive behavior.
"""

from __future__ import annotations

from collections import deque
import ctypes
import queue
import sys
import time

import numpy as np

import q900_control_core as core


SSB_LOW_HZ = 300.0
SSB_HIGH_HZ = 2800.0
SSB_FILTER_LEN = 511
HILBERT_LEN = 511
HILBERT_DELAY = (HILBERT_LEN - 1) // 2
IQ_PACKET_FRAMES = 48
IQ_PREROLL_FRAMES = 9_600
IQ_HIGH_WATER_FRAMES = 19_200


def _fir_bandpass_taps(low_hz: float, high_hz: float, count: int) -> np.ndarray:
    """Windowed-sinc voice bandpass, normalized at 1 kHz."""
    index = np.arange(count, dtype=np.float64) - (count - 1) / 2
    taps = (
        2 * high_hz / core.IQ_SAMPLE_RATE
        * np.sinc(2 * high_hz * index / core.IQ_SAMPLE_RATE)
        - 2 * low_hz / core.IQ_SAMPLE_RATE
        * np.sinc(2 * low_hz * index / core.IQ_SAMPLE_RATE)
    )
    taps *= np.blackman(count)
    omega = 2 * np.pi * 1000.0 / core.IQ_SAMPLE_RATE
    gain = abs(np.sum(taps * np.exp(-1j * omega * np.arange(count))))
    if gain:
        taps /= gain
    return taps.astype(np.float32)


SSB_FILTER_TAPS = _fir_bandpass_taps(SSB_LOW_HZ, SSB_HIGH_HZ, SSB_FILTER_LEN)

_hilbert_index = np.arange(HILBERT_LEN, dtype=np.float64) - HILBERT_DELAY
HILBERT_TAPS = np.zeros(HILBERT_LEN, dtype=np.float64)
_hilbert_odd = (np.abs(_hilbert_index) % 2) == 1
HILBERT_TAPS[_hilbert_odd] = 2.0 / (np.pi * _hilbert_index[_hilbert_odd])
HILBERT_TAPS *= np.blackman(HILBERT_LEN)
HILBERT_TAPS = HILBERT_TAPS.astype(np.float32)


class ImprovedIqEncoderState:
    """Streaming state for the corrected I/Q encoder."""

    __slots__ = (
        "phase", "level", "ssb_dc", "fm_dc", "pre_prev", "fm_filter_state",
        "ssb_filter_state", "hilbert_state", "ssb_gain", "sample_count",
    )

    def __init__(self) -> None:
        self.phase = 0.0
        self.level = 0.0
        self.ssb_dc = 0.0
        self.fm_dc = 0.0
        self.pre_prev = 0.0
        self.fm_filter_state = np.zeros(len(core.IQ_WFM_AUDIO_TAPS) - 1, dtype=np.float64)
        self.ssb_filter_state = np.zeros(SSB_FILTER_LEN - 1, dtype=np.float32)
        self.hilbert_state = np.zeros(HILBERT_LEN - 1, dtype=np.float32)
        self.ssb_gain = 1.0
        self.sample_count = 0


def _filter_ssb_audio(state: ImprovedIqEncoderState, audio: np.ndarray) -> np.ndarray:
    """Band-limit speech and apply a clean peak limiter instead of hard clipping."""
    # A slow DC estimate remains useful ahead of the explicit high-pass edge;
    # it keeps very large interface offsets from wasting FIR headroom.
    state.ssb_dc = 0.995 * state.ssb_dc + 0.005 * float(np.mean(audio))
    source = np.asarray(audio - state.ssb_dc, dtype=np.float32)
    combined = np.concatenate((state.ssb_filter_state, source))
    filtered = np.convolve(combined, SSB_FILTER_TAPS, mode="valid").astype(np.float32)
    state.ssb_filter_state = combined[-(SSB_FILTER_LEN - 1):]

    # Never boost quiet audio here.  Only reduce gain when a peak would exceed
    # 90% full scale; recover slowly so a single consonant does not pump speech.
    peak = float(np.max(np.abs(filtered))) if len(filtered) else 0.0
    target = min(1.0, 0.90 / max(peak, 1e-9))
    if target < state.ssb_gain:
        state.ssb_gain = target
    else:
        # encode_iq_block is normally called once per 1 ms I/Q packet.  This is
        # roughly a 200 ms release and is intentionally much slower than attack.
        state.ssb_gain += 0.005 * (target - state.ssb_gain)
    return np.clip(filtered * state.ssb_gain, -0.98, 0.98)


def encode_iq_block_improved(
    state: ImprovedIqEncoderState, audio: np.ndarray, mode: str, offset_hz: int
) -> np.ndarray:
    """Encode 48 kHz mono audio into corrected complex I/Q samples."""
    count = len(audio)
    if mode in ("USB", "LSB"):
        ssb_audio = _filter_ssb_audio(state, audio)
        combined = np.concatenate((state.hilbert_state, ssb_audio))
        quadrature = np.convolve(combined, HILBERT_TAPS, mode="valid")
        in_phase = combined[HILBERT_DELAY : HILBERT_DELAY + count]
        state.hilbert_state = combined[-(HILBERT_LEN - 1):]
        baseband = in_phase + 1j * (quadrature if mode == "USB" else -quadrature)
    elif mode == "AM":
        state.ssb_dc = 0.995 * state.ssb_dc + 0.005 * float(np.mean(audio))
        baseband = 0.55 + np.clip(audio - state.ssb_dc, -0.45, 0.45).astype(np.complex64)
    elif mode == "NFM":
        state.level = 0.95 * state.level + 0.05 * float(np.max(np.abs(audio)))
        fm_gain = float(np.clip(0.9 / max(state.level, 1e-4), 3.0, 20.0))
        fm_audio = np.clip(audio * fm_gain, -0.9, 0.9)
        previous = np.concatenate((np.array([state.pre_prev]), fm_audio[:-1]))
        emphasized = np.clip(
            fm_audio + core.IQ_NFM_PRE_EMPHASIS_ALPHA * (fm_audio - previous), -0.9, 0.9
        )
        state.pre_prev = float(fm_audio[-1])
        state.phase += np.cumsum(
            emphasized * (2 * np.pi * core.IQ_NFM_DEVIATION / core.IQ_SAMPLE_RATE)
        )
        baseband = np.exp(1j * state.phase)
        state.phase = float(state.phase[-1] % (2 * np.pi))
    elif mode == "WFM":
        highpassed = np.empty_like(audio, dtype=np.float64)
        for sample_index, sample in enumerate(audio):
            state.fm_dc += core.IQ_WFM_HIGHPASS_ALPHA * (float(sample) - state.fm_dc)
            highpassed[sample_index] = sample - state.fm_dc
        state.level = 0.95 * state.level + 0.05 * float(np.max(np.abs(highpassed)))
        fm_gain = float(np.clip(core.IQ_WFM_PEAK / max(state.level, 1e-4), 1.0, 20.0))
        fm_audio = np.clip(highpassed * fm_gain, -core.IQ_WFM_PEAK, core.IQ_WFM_PEAK)
        previous = np.concatenate((np.array([state.pre_prev]), fm_audio[:-1]))
        emphasized = (
            fm_audio - core.IQ_WFM_PRE_EMPHASIS_DECAY * previous
        ) / (1.0 - core.IQ_WFM_PRE_EMPHASIS_DECAY)
        state.pre_prev = float(fm_audio[-1])
        filter_input = np.concatenate((state.fm_filter_state, emphasized))
        state.fm_filter_state = filter_input[-(len(core.IQ_WFM_AUDIO_TAPS) - 1):]
        filtered = np.convolve(filter_input, core.IQ_WFM_AUDIO_TAPS, mode="valid")
        modulation = np.clip(filtered, -core.IQ_WFM_PEAK, core.IQ_WFM_PEAK)
        phase_scale = (
            2 * np.pi * core.IQ_WFM_DEVIATION
            / (core.IQ_WFM_PEAK * core.IQ_SAMPLE_RATE)
        )
        state.phase += np.cumsum(modulation * phase_scale)
        baseband = np.exp(1j * state.phase)
        state.phase = float(state.phase[-1] % (2 * np.pi))
    else:
        raise ValueError(f"unsupported SDR mode: {mode}")

    index = np.arange(state.sample_count, state.sample_count + count)
    state.sample_count += count
    carrier = np.exp(1j * 2 * np.pi * offset_hz * index / core.IQ_SAMPLE_RATE)
    iq = np.conj(baseband * carrier)
    real = np.clip(iq.real, -1.0, 1.0) * core.IQ_TX_LEVEL
    imag = np.clip(iq.imag, -1.0, 1.0) * core.IQ_TX_LEVEL
    return real + 1j * imag


def _resolve_input_device(device_name):  # type: ignore[no-untyped-def]
    if not isinstance(device_name, str):
        return device_name
    for index, info in enumerate(core.sd.query_devices()):
        if info["name"] == device_name and int(info["max_input_channels"]) > 0:
            return index
    return device_name


def _iq_radio_timing(radio_packet_rate: float) -> tuple[float, float]:
    """Return (48-frame packet period, host/input frames per radio/output frame)."""
    if radio_packet_rate <= 0.0:
        return IQ_PACKET_FRAMES / core.IQ_SAMPLE_RATE, 1.0
    radio_frames_per_second = (
        radio_packet_rate * 48.0 * (1.0 + core.TX_RATE_PPM * 1e-6)
    )
    return IQ_PACKET_FRAMES / radio_frames_per_second, core.IQ_SAMPLE_RATE / radio_frames_per_second


def udp_iq_sender_improved(
    device_name,
    udp_socket,
    target,
    stop,
    keyed,
    packets,
    underruns,
    late_ms,
    clipped,
    mode,
    offset_hz,
    swap_iq,
    invert_q,
    radio_packet_rate,
    overflows,
    level,
    ready,
    failure,
) -> None:
    """Capture, rate-match, modulate and pace SDR I/Q entirely in one process."""
    incoming: deque[bytes] = deque()
    pending = bytearray()
    stream = None

    def callback(indata, frames, timing, status):  # type: ignore[no-untyped-def]
        if status.input_overflow:
            overflows.value += 1
        mono = np.clip(indata[:, 0], -1.0, 1.0)
        peak = float(np.max(np.abs(mono))) if len(mono) else 0.0
        level.value = peak
        if peak >= 0.98:
            clipped.value += 1
        words = np.rint(mono * 32767.0).astype("<i2")
        stereo = np.empty((len(words), 2), dtype="<i2")
        stereo[:, 0] = words
        stereo[:, 1] = words
        incoming.append(stereo.tobytes())

    try:
        stream = core.sd.InputStream(
            device=_resolve_input_device(device_name),
            samplerate=core.IQ_SAMPLE_RATE,
            blocksize=core.TransmitAudioRouter.BLOCK_SIZE,
            channels=1,
            dtype="float32",
            latency="low",
            callback=callback,
        )
        stream.start()
    except Exception as error:  # noqa: BLE001 - communicate host audio failures
        failure.value = f"microphone: {error}".encode()[:255]
        ready.set()
        return

    def refill() -> None:
        while incoming:
            pending.extend(incoming.popleft())
        if len(pending) // 4 > IQ_HIGH_WATER_FRAMES:
            # Keep recent speech if the host outruns the radio badly.  Preserve
            # enough filter history for the shared sinc rate converter.
            excess_frames = len(pending) // 4 - IQ_HIGH_WATER_FRAMES
            del pending[: excess_frames * 4]

    deadline_preroll = time.monotonic() + core.NETWORK_TX_READY_TIMEOUT
    while len(pending) // 4 < IQ_PREROLL_FRAMES and not stop.is_set():
        refill()
        if time.monotonic() >= deadline_preroll:
            if not pending:
                failure.value = b"microphone delivered no audio"
            break
        time.sleep(0.005)
    ready.set()
    while not keyed.wait(0.05) and not stop.is_set():
        pass
    if stop.is_set():
        try:
            stream.stop(); stream.close()
        except Exception:
            pass
        return

    state = ImprovedIqEncoderState()
    period, base_ratio = _iq_radio_timing(radio_packet_rate)
    ratio_trim = 0.0
    ratio_smooth = (0.0, 0.0)
    resample_phase = 0.0
    target_frames = IQ_PREROLL_FRAMES // 2

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
        nonlocal ratio_trim, ratio_smooth, resample_phase
        refill()
        ratio, ratio_trim, ratio_smooth = core.resample_ratio(
            len(pending) // 4,
            target_frames,
            ratio_trim,
            base_ratio,
            ratio_smooth,
        )
        converted = core.resample_stereo(
            pending, IQ_PACKET_FRAMES, ratio, resample_phase
        )
        if converted is None:
            underruns.value += 1
            audio = np.zeros(IQ_PACKET_FRAMES, dtype=np.float32)
        else:
            payload, resample_phase = converted
            frames = np.frombuffer(payload, dtype="<i2").reshape(-1, 2)
            audio = frames[:, 0].astype(np.float32) / 32768.0
        iq = encode_iq_block_improved(state, audio, mode, offset_hz)
        return core.pack_iq_words(iq, swap_iq, invert_q)

    try:
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
                late_ms.value = max(late_ms.value, lateness * 1000.0)
                if lateness > period:
                    deadline = mach_time()
            else:
                deadline += period
                remaining = deadline - time.monotonic()
                if remaining < 0:
                    late_ms.value = max(late_ms.value, -remaining * 1000.0)
                    deadline = time.monotonic()
                else:
                    time.sleep(remaining)
    finally:
        try:
            stream.stop(); stream.close()
        except Exception:  # noqa: BLE001 - teardown must not hide the real failure
            pass


def start_iq_udp_improved(
    self,
    microphone: int,
    target: tuple[str, int],
    network_audio,
    mode: str,
    offset_hz: int,
    swap_iq: bool,
    invert_q: bool,
) -> None:
    """Start corrected SDR TX with capture owned by the sender process."""
    self.stop()
    self._udp_target = target
    self._udp_queue = None
    self._udp_stop = self._mp.Event()
    self._udp_keyed = self._mp.Event()
    self._udp_packets = self._mp.Value("L", 0, lock=False)
    self._udp_underruns = self._mp.Value("L", 0, lock=False)
    self._udp_late_ms = self._mp.Value("d", 0.0, lock=False)
    self._udp_clipped = self._mp.Value("L", 0, lock=False)
    self._udp_overflows = self._mp.Value("L", 0, lock=False)
    self._udp_level = self._mp.Value("d", 0.0, lock=False)
    self._udp_failure = self._mp.Array("c", 256, lock=False)
    self._udp_ready = self._mp.Event()
    self._udp_trimmed = None
    self._udp_send_errors = None
    self._udp_dropped = None
    self._udp_repeats = None
    self._udp_ring = None
    self._input_stream = None

    try:
        device_name = core.sd.query_devices(microphone)["name"]
    except Exception:  # noqa: BLE001
        device_name = microphone

    radio_rate = network_audio.measured_packet_rate
    self._udp_sender = self._mp.Process(
        target=udp_iq_sender_improved,
        args=(
            device_name,
            network_audio.socket,
            target,
            self._udp_stop,
            self._udp_keyed,
            self._udp_packets,
            self._udp_underruns,
            self._udp_late_ms,
            self._udp_clipped,
            mode,
            offset_hz,
            swap_iq,
            invert_q,
            radio_rate,
            self._udp_overflows,
            self._udp_level,
            self._udp_ready,
            self._udp_failure,
        ),
        name="q900-iq-tx",
        daemon=True,
    )
    self._udp_sender.start()

    state = (
        f"SDR TX: microphone -> Q900 UDP {target[0]}:{target[1]} "
        f"({mode} I/Q, {offset_hz:+d} Hz, clock-matched"
        + (f" {radio_rate:.2f} pkt/s" if radio_rate else " nominal 48 kHz")
        + ")"
    )
    if not self._udp_ready.wait(timeout=core.NETWORK_TX_READY_TIMEOUT):
        state += " -- sender did not report ready"
    problem = bytes(self._udp_failure.value if self._udp_failure else b"")
    if problem:
        state += f" -- {problem.decode(errors='replace')}"
    self.signals.audio_state_changed.emit(state)


def install(core_module=core) -> None:
    """Install the corrected transmit encoder and sender into Q900Control."""
    core_module.IqEncoderState = ImprovedIqEncoderState
    core_module.encode_iq_block = encode_iq_block_improved
    core_module.TransmitAudioRouter.start_iq_udp = start_iq_udp_improved


def self_test() -> None:
    """Verify SSB band limiting, sideband rejection and non-unity rate conversion."""
    def response(taps: np.ndarray, hz: float) -> float:
        omega = 2 * np.pi * hz / core.IQ_SAMPLE_RATE
        return float(abs(np.sum(taps * np.exp(-1j * omega * np.arange(len(taps))))))

    assert 20 * np.log10(max(response(SSB_FILTER_TAPS, 100.0), 1e-12)) < -40.0
    assert abs(20 * np.log10(response(SSB_FILTER_TAPS, 1000.0))) < 0.2
    assert 20 * np.log10(max(response(SSB_FILTER_TAPS, 4000.0), 1e-12)) < -60.0

    # The Hilbert pair itself must have ample rejection at the lower passband
    # edge.  This catches accidental regression to the old 127-tap transformer.
    h300 = response(HILBERT_TAPS, 300.0)
    rejection = 20 * np.log10((1.0 + h300) / max(abs(1.0 - h300), 1e-12))
    assert rejection > 60.0, rejection

    # Exercise the shared polyphase rate converter at a realistic crystal error.
    frames = np.arange(12_000, dtype=np.int16)
    stereo = np.column_stack((frames, frames)).astype("<i2")
    pending = bytearray(stereo.tobytes())
    converted = core.resample_stereo(pending, 48, 1.000493, 0.0)
    assert converted is not None
    payload, phase = converted
    out = np.frombuffer(payload, dtype="<i2").reshape(-1, 2)
    assert out.shape == (48, 2)
    assert np.array_equal(out[:, 0], out[:, 1])
    assert 0.0 <= phase < 1.0

    print(f"SDR SSB TX self-test passed; Hilbert rejection at 300 Hz {rejection:.1f} dB")
