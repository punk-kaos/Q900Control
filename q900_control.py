#!/usr/bin/env python3
"""Q900 Control entrypoint with raw SDR I/Q audio passthrough support.

The main application remains in q900_control_core.py.  Keeping the existing
implementation there byte-for-byte lets this entrypoint add the raw-IQ path
without rewriting the large, heavily tested DSP module through GitHub's
whole-file contents API.
"""

from __future__ import annotations

from collections import deque
import threading

import numpy as np

import q900_control_core as core


RAW_IQ_MODE = "RAW IQ"


# --- Audio sink: retain mono fan-out, but preserve two-channel I/Q verbatim. ---
_original_audio_sink_init = core.AudioSink.__init__


def _audio_sink_init(self, device: int, sample_rate: int, blocksize: int,
                     max_queued_frames: int) -> None:
    _original_audio_sink_init(self, device, sample_rate, blocksize, max_queued_frames)
    info = core.sd.query_devices(device, "output")
    self.output_channels = min(2, int(info["max_output_channels"]))


def _audio_sink_callback(self, outdata, frames, timing, status):  # type: ignore[no-untyped-def]
    if status.output_underflow:
        self.underflows += 1
    outdata.fill(0)
    offset = 0
    with self._lock:
        while offset < frames and self._queue:
            block = self._queue[0]
            count = min(frames - offset, len(block))
            chunk = block[:count]
            if chunk.ndim == 1:
                # Existing receive audio is mono and is intentionally copied to
                # every output channel, exactly as before this feature.
                outdata[offset : offset + count, :] = chunk[:, np.newaxis]
            else:
                channels = min(chunk.shape[1], outdata.shape[1])
                outdata[offset : offset + count, :channels] = chunk[:, :channels]
            offset += count
            if count == len(block):
                self._queue.popleft()
            else:
                self._queue[0] = block[count:]
            self._queued_frames -= count


def _audio_sink_enqueue(self, samples: np.ndarray) -> None:
    block = np.asarray(samples, dtype=np.float32)
    if block.ndim not in (1, 2):
        raise ValueError("audio blocks must be mono or frame-by-channel arrays")
    if block.ndim == 2 and block.shape[1] > self.output_channels:
        # A mono device cannot carry both I and Q.  Silence is safer than
        # silently discarding Q and pretending the stream is still raw I/Q.
        return
    with self._lock:
        while self._queue and self._queued_frames + len(block) > self._max_queued_frames:
            self._queued_frames -= len(self._queue.popleft())
        self._queue.append(block)
        self._queued_frames += len(block)


core.AudioSink.__init__ = _audio_sink_init
core.AudioSink._callback = _audio_sink_callback
core.AudioSink.enqueue = _audio_sink_enqueue


# --- SDR receive: RAW IQ bypasses every demodulation/calibration transform. ---
class RawIqSDRReceiver(core.SDRReceiver):
    """SDRReceiver with a receive-only raw I/Q audio mode.

    The Q900 sends signed 16-bit interleaved I,Q words at 48 kHz.  RAW IQ only
    converts those words to PortAudio's float32 range and emits a two-channel
    frame array: channel 0/left is I and channel 1/right is Q.  Offset tuning,
    DC removal, I/Q swap/inversion, filtering, gain, and demodulation are all
    intentionally bypassed.
    """

    def feed(self, words: np.ndarray) -> None:
        if self.mode != RAW_IQ_MODE:
            super().feed(words)
            return
        if len(words) < 2:
            return
        # Keep the same 960-complex-frame block geometry as normal SDR receive,
        # but do not put raw IQ through the demodulator worker or its preroll.
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
        iq = block.astype(np.float32).reshape(-1, 2) / 32768.0
        self._output(iq)


core.SDRReceiver = RawIqSDRReceiver


# --- UI integration and receive-only safety. ---
_original_top_panel = core.MainWindow._top_panel
_original_set_sdr_mode = core.MainWindow.set_sdr_mode
_original_start_ptt = core.MainWindow.start_ptt
_original_handle_rigctl_ptt = core.MainWindow.handle_rigctl_ptt
_original_configure_sdr_tx = core.MainWindow.configure_sdr_tx
_original_poll_sdr_stream = core.MainWindow.poll_sdr_stream
_original_draw_sdr_cursor = core.SpectrumWaterfall._draw_sdr_cursor


def _top_panel(self):
    panel = _original_top_panel(self)
    self.sdr_mode_selector.addItem(RAW_IQ_MODE)
    self.sdr_mode_selector.setToolTip(
        "Host SDR demodulator; RAW IQ sends I to left and Q to right at 48 kHz (RX only)"
    )
    return panel


def _raw_iq_output_available(window) -> bool:
    with window.network_audio._sink_lock:
        sinks = tuple(window.network_audio._sinks)
    return any(getattr(sink, "output_channels", 1) >= 2 for sink in sinks)


def _show_raw_iq_state(window) -> None:
    window.sdr_offset.setEnabled(False)
    window.sdr_tx_calibrate.setEnabled(False)
    if _raw_iq_output_available(window):
        window.status.setText(
            "SDR RAW IQ: 48 kHz unprocessed I/Q -> audio device (I left, Q right). Receive only."
        )
    else:
        window.status.setText(
            "SDR RAW IQ needs a stereo output device to preserve I and Q; current output is mono."
        )


def _set_sdr_mode(self, mode: str) -> None:
    _original_set_sdr_mode(self, mode)
    raw = mode == RAW_IQ_MODE
    self.sdr_offset.setEnabled(not raw)
    self.sdr_tx_calibrate.setEnabled(not raw)
    if raw:
        _show_raw_iq_state(self)


def _start_ptt(self) -> None:
    if self._sdr_active and self.sdr_receiver.mode == RAW_IQ_MODE:
        self.status.setText(
            "RAW IQ is receive-only. Select USB, LSB, NFM, WFM, or AM before transmitting."
        )
        return
    _original_start_ptt(self)


def _handle_rigctl_ptt(self, active: bool) -> None:
    if active and self._sdr_active and self.sdr_receiver.mode == RAW_IQ_MODE:
        self.rigctl_status.setText("rigctl: RAW IQ is receive-only; select a transmit-capable SDR mode")
        return
    _original_handle_rigctl_ptt(self, active)


def _configure_sdr_tx(self) -> None:
    if self.sdr_receiver.mode == RAW_IQ_MODE:
        self.status.setText("RAW IQ is receive-only; SDR TX calibration does not apply.")
        return
    _original_configure_sdr_tx(self)


def _poll_sdr_stream(self) -> None:
    _original_poll_sdr_stream(self)
    if self._sdr_active and self.sdr_receiver.mode == RAW_IQ_MODE:
        _show_raw_iq_state(self)


def _draw_sdr_cursor(self, painter, width: int, height: int) -> None:
    if self._sdr_mode != RAW_IQ_MODE:
        _original_draw_sdr_cursor(self, painter, width, height)
        return
    # RAW IQ exposes the entire complex Nyquist span rather than a demodulator
    # passband.  On the IQ waterfall that is exactly the full widget width.
    iq_display = self._active_history() == "iq"
    if iq_display:
        x = width / 2
    else:
        x = self._frequency_to_x(self._tuned_hz + core.FFT_TUNED_OFFSET_HZ, width)
    painter.setPen(core.Qt.PenStyle.NoPen)
    painter.setBrush(core.QColor(238, 174, 99, 38))
    painter.drawRect(core.QRectF(0, 0, width, height))
    painter.setPen(core.QPen(core.QColor("#eeae63"), 2))
    painter.drawLine(round(x), 0, round(x), height)
    painter.setPen(core.QColor("#eeae63"))
    label_x = min(max(8, int(x + 10)), max(8, width - 260))
    painter.drawText(label_x, 38, "SDR RAW IQ  48 kHz  I=L Q=R")


core.MainWindow._top_panel = _top_panel
core.MainWindow.set_sdr_mode = _set_sdr_mode
core.MainWindow.start_ptt = _start_ptt
core.MainWindow.handle_rigctl_ptt = _handle_rigctl_ptt
core.MainWindow.configure_sdr_tx = _configure_sdr_tx
core.MainWindow.poll_sdr_stream = _poll_sdr_stream
core.SpectrumWaterfall._draw_sdr_cursor = _draw_sdr_cursor


def _raw_iq_self_test() -> None:
    # The raw receiver must preserve ordering and scale even when the normal SDR
    # calibration toggles are deliberately set to values that would alter it.
    frames = np.empty((RawIqSDRReceiver.BLOCK_FRAMES, 2), dtype="<i2")
    frames[:, 0] = np.arange(RawIqSDRReceiver.BLOCK_FRAMES, dtype=np.int16) - 480
    frames[:, 1] = 12345 - np.arange(RawIqSDRReceiver.BLOCK_FRAMES, dtype=np.int16)
    words = frames.reshape(-1)
    outputs: list[np.ndarray] = []
    receiver = RawIqSDRReceiver(outputs.append)
    receiver.mode = RAW_IQ_MODE
    receiver.offset_hz = -19_000
    receiver.swap_iq = True
    receiver.invert_q = True
    receiver.feed(words)
    assert len(outputs) == 1
    assert outputs[0].shape == (RawIqSDRReceiver.BLOCK_FRAMES, 2)
    expected = words.astype(np.float32).reshape(-1, 2) / 32768.0
    assert np.array_equal(outputs[0], expected), "RAW IQ must not apply SDR calibration or DSP"

    # Exercise the sink callback without opening PortAudio: stereo must stay
    # stereo, while legacy mono receive audio must still duplicate to L/R.
    class Status:
        output_underflow = False

    sink = object.__new__(core.AudioSink)
    sink.underflows = 0
    sink._lock = threading.Lock()
    sink._queue = deque()
    sink._queued_frames = 0
    sink._max_queued_frames = 100
    sink.output_channels = 2

    stereo = np.array([[0.1, -0.2], [0.3, -0.4]], dtype=np.float32)
    sink.enqueue(stereo)
    out = np.zeros((2, 2), dtype=np.float32)
    sink._callback(out, 2, None, Status())
    assert np.array_equal(out, stereo), (out, stereo)

    mono = np.array([0.25, -0.5], dtype=np.float32)
    sink.enqueue(mono)
    out.fill(0)
    sink._callback(out, 2, None, Status())
    assert np.array_equal(out[:, 0], mono)
    assert np.array_equal(out[:, 1], mono)
    print("RAW IQ passthrough self-test passed")


# Preserve q900_control's historical import surface for callers that import the
# script as a module rather than executing it.
for _name in dir(core):
    if not _name.startswith("__"):
        globals()[_name] = getattr(core, _name)


def _entrypoint() -> None:
    if "--self-test" in core.sys.argv:
        core.self_test()
        _raw_iq_self_test()
        return
    core.main()


if __name__ == "__main__":
    _entrypoint()
