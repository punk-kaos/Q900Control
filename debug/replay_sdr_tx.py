#!/usr/bin/env python3
"""Replay a recorded SDR transmission through the audited Q900 firmware.

Requires numpy and the optional unicorn package. Executes the real firmware's
network ring writer, DSP reader, corrected I/Q conversion and output stage.
Host send timestamps are used as assumed arrival times; this is not evidence of
delivery to the physical radio. The ring starts empty and aligned. Runtime
calibration and hardware deadlines are not captured: power/IQ gains are set to
unity and phase correction to zero.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import struct

import numpy as np

from patch_sdr_iq_gain import IMAGE_BASE, verify_image


def replay(prefix: str, image_path: Path, dropped_packets: set[int]) -> None:
    import unicorn
    from unicorn import arm_const as arm

    image = image_path.read_bytes()
    verify_image(image)
    words = np.fromfile(prefix + ".iq.tx.raw", dtype="<i2").reshape(-1, 2)
    stamps = np.fromfile(prefix + ".iq.tx.time", dtype="<u8")
    metadata = json.loads(Path(prefix + ".iq.tx.json").read_text())
    if not len(stamps) or len(words) % len(stamps) or np.any(stamps[1:] <= stamps[:-1]):
        raise ValueError("recording must have constant packet geometry and increasing timestamps")
    packet_frames = len(words) // len(stamps)
    if not 48 <= packet_frames <= 640 or packet_frames != metadata["frames_per_packet"]:
        raise ValueError("unsupported or inconsistent packet geometry")
    if any(index < 0 or index >= len(stamps) for index in dropped_packets):
        raise ValueError("dropped packet index outside recording")
    radio_rate = metadata["radio_packet_rate"] or 1000.0
    rate = radio_rate * 48 * (1 + metadata["tx_rate_ppm"] * 1e-6)
    if not np.isfinite(rate) or rate <= 0:
        raise ValueError("invalid radio rate")
    arrivals = (stamps - stamps[0]).astype(float) / 1e9

    emulator = unicorn.Uc(unicorn.UC_ARCH_ARM, unicorn.UC_MODE_THUMB | unicorn.UC_MODE_MCLASS)
    emulator.ctl_set_cpu_model(arm.UC_CPU_ARM_CORTEX_M7)
    emulator.mem_map(IMAGE_BASE, (len(image) + 4095) // 4096 * 4096)
    emulator.mem_write(IMAGE_BASE, image)
    emulator.mem_map(0x24000000, 0x80000)
    emulator.reg_write(arm.UC_ARM_REG_C1_C0_2, 0x00F00000)
    emulator.reg_write(arm.UC_ARM_REG_FPEXC, 0x40000000)
    state = 0x2403E784
    emulator.mem_write(state + 0xAF, b"\x01")  # Radio is already transmitting.
    emulator.mem_write(state + 0x131, b"\x02")  # Raw I/Q stream.
    emulator.mem_write(state + 0xB3, b"\x06")
    emulator.mem_write(state + 0x1F8, struct.pack("<I", 12000))
    for offset in (0x2C, 0x30, 0x34, 0x38, 0x44):
        emulator.mem_write(state + offset, struct.pack("<f", 1.0))

    def call(address: int, *args: int) -> int:
        registers = (arm.UC_ARM_REG_R0, arm.UC_ARM_REG_R1, arm.UC_ARM_REG_R2, arm.UC_ARM_REG_R3)
        for register, value in zip(registers, args):
            emulator.reg_write(register, value)
        emulator.reg_write(arm.UC_ARM_REG_SP, 0x24070000)
        emulator.reg_write(arm.UC_ARM_REG_LR, IMAGE_BASE + 0x101)
        emulator.emu_start(address | 1, IMAGE_BASE + 0x100, count=100000)
        if emulator.reg_read(arm.UC_ARM_REG_PC) != IMAGE_BASE + 0x100:
            raise RuntimeError(f"firmware call 0x{address:08X} did not return")
        return emulator.reg_read(arm.UC_ARM_REG_R0)

    ring_output: list[np.ndarray] = []
    codec_output: list[np.ndarray] = []
    corrected = empty_ticks = tick = 0
    minimum, maximum = 6144, 0
    for index, arrived in enumerate(arrivals):
        while tick * 32 / rate < arrived:
            before = call(0x0806C7DC)
            call(0x0806C954, 0x24061000, 32)
            ring_output.append(np.frombuffer(emulator.mem_read(0x24061000, 256), dtype="<i4").copy() >> 16)
            if index > 10 and before < 64:
                empty_ticks += 1
            emulator.reg_write(arm.UC_ARM_REG_R8, 0x24061000)
            emulator.reg_write(arm.UC_ARM_REG_R3, 0)
            emulator.reg_write(arm.UC_ARM_REG_R4, 32)
            emulator.emu_start(0x08039EE3, 0x08039F1C, count=1000)
            if emulator.reg_read(arm.UC_ARM_REG_PC) != 0x08039F1C:
                raise RuntimeError("raw-I/Q conversion loop did not finish")
            emulator.reg_write(arm.UC_ARM_REG_S0, emulator.reg_read(arm.UC_ARM_REG_S16))
            call(0x08039C38, 0, 0x24004EE8, 0x24062000, 32)
            codec_output.append(np.frombuffer(emulator.mem_read(0x24062000, 256), dtype="<i4").copy())
            tick += 1
        if index in dropped_packets:
            continue
        before = call(0x0806C7DC)
        if index > 10:
            corrected += before < 1536 or before > 4608
            minimum = min(minimum, before)
        payload = words[index * packet_frames:(index + 1) * packet_frames].tobytes()
        emulator.mem_write(0x24060000, payload)
        call(0x0806C80C, 0x24060000, packet_frames * 2)
        if index > 10:
            maximum = max(maximum, call(0x0806C7DC))

    print("Firmware replay: recorded HOST times used as assumed radio arrivals")
    print(f"Injected packet losses (zero-based): {sorted(dropped_packets)}")
    print(f"Radio rate: {rate:.3f} frames/s; power/IQ gains 1; phase correction 0")
    overflow = struct.unpack("<H", emulator.mem_read(0x240391B4, 2))[0]
    print(
        f"Post-priming frame corrections: {corrected}; empty DSP blocks: {empty_ticks}; "
        f"overflow words: {overflow}; ring depth range: {minimum}..{maximum} words"
    )
    for label, blocks, full_scale in (
        ("ring playback", ring_output, 32767), ("codec output", codec_output, 2**31),
    ):
        if not blocks:
            continue
        frames = np.concatenate(blocks).reshape(-1, 2).astype(float)[48000:]
        if len(frames) < 2:
            print(f"{label}: too short for steady-state analysis")
            continue
        signal = (frames[:, 0] + 1j * frames[:, 1]) / full_scale
        envelope = np.abs(signal)
        step = np.angle(signal[1:] * np.conj(signal[:-1]))
        residual = np.angle(np.exp(1j * (step - np.median(step))))
        ripple = np.std(envelope) / max(np.mean(envelope), 1e-15) * 100
        print(
            f"{label}: envelope ripple {ripple:.5f}%; phase RMS {np.std(residual):.6f} rad; "
            f"max {np.max(np.abs(residual)):.6f} rad; zero frames {np.count_nonzero(envelope == 0)}"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("prefix", help="recording prefix, including .iq.tx.json metadata")
    parser.add_argument("--image", type=Path, default=Path(__file__).with_name("q900_fw_3.7.6_sdr_iq_gain_fix.bin"))
    parser.add_argument("--drop-packet", type=int, action="append", default=[], help="inject a loss at this zero-based index")
    args = parser.parse_args()
    try:
        replay(args.prefix, args.image, set(args.drop_packet))
    except (OSError, ValueError, KeyError, ImportError) as error:
        parser.exit(1, f"error: {error}\n")


if __name__ == "__main__":
    main()
