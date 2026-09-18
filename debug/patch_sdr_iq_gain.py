#!/usr/bin/env python3
"""Offline, version-locked Q900 3.7.6 raw-I/Q normalization patch.

Only two Thumb/VFP conversion instructions change. No flashing is performed.
See SDR_TX_GAIN_AUDIT.md for the data path and image-format limitations.
"""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import struct
import tempfile
import unittest


IMAGE_BASE = 0x08020000
IMAGE_SIZE = 0x114F24
SOURCE_SHA256 = "f5fd15145c61578cad14ddbce666373888d43d50161e5783db9886996c624915"
SITES = (("I", 0x08039EF4), ("Q", 0x08039F0A))
# GNU Arm as, .cpu cortex-m7 / .fpu fpv5-d16 / .thumb:
# eef8 7ae7  vcvt.f32.s32 s15, s15
# eefa 7ac8  vcvt.f32.s32 s15, s15, #16
# Byte order below is little-endian within each Thumb halfword.
ORIGINAL = bytes.fromhex("f8 ee e7 7a")
CORRECTED = bytes.fromhex("fa ee c8 7a")


def sha256(image: bytes) -> str:
    return hashlib.sha256(image).hexdigest()


def _replace(image: bytes, expected: bytes, replacement: bytes) -> bytes:
    if len(image) != IMAGE_SIZE:
        raise ValueError(f"wrong image length: {len(image)}; expected {IMAGE_SIZE}")
    result = bytearray(image)
    for channel, address in SITES:
        offset = address - IMAGE_BASE
        found = image[offset:offset + len(expected)]
        if found != expected:
            raise ValueError(
                f"{channel} instruction at 0x{address:08X}: "
                f"expected {expected.hex(' ')}, found {found.hex(' ')}"
            )
        result[offset:offset + len(expected)] = replacement
    return bytes(result)


def patch_image(image: bytes) -> bytes:
    """Accept exactly the audited original, not merely a matching version label."""
    digest = sha256(image)
    if digest != SOURCE_SHA256:
        raise ValueError(f"unsupported source SHA-256: {digest}; expected {SOURCE_SHA256}")
    patched = _replace(image, ORIGINAL, CORRECTED)
    verify_image(patched)
    return patched


def verify_image(image: bytes) -> None:
    """Undo only the two edits and require a byte-exact audited original."""
    restored = _replace(image, CORRECTED, ORIGINAL)
    if sha256(restored) != SOURCE_SHA256:
        raise ValueError("image has changes outside the two audited I/Q instructions")


def patch_file(source: Path, destination: Path) -> bytes:
    image = patch_image(source.read_bytes())
    # Exclusive creation preserves the source, existing outputs and symlinks.
    with destination.open("xb") as handle:
        handle.write(image)
    written = destination.read_bytes()
    verify_image(written)
    return written


class PatchTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.original = Path(__file__).with_name("q900_fw.bin").read_bytes()
        cls.patched = patch_image(cls.original)

    def test_only_the_two_instructions_change(self) -> None:
        self.assertEqual(len(self.original), len(self.patched))
        changed = [i for i, (before, after) in enumerate(zip(self.original, self.patched)) if before != after]
        self.assertEqual(changed, [address - IMAGE_BASE + byte for _, address in SITES for byte in (0, 2)])
        self.assertEqual(self.original[:0x200], self.patched[:0x200])
        verify_image(self.patched)

    def test_wrong_source_and_double_patch_are_rejected(self) -> None:
        for candidate in (self.original[:-1], b"", self.patched):
            with self.subTest(length=len(candidate)), self.assertRaises(ValueError):
                patch_image(candidate)
        altered = bytearray(self.original)
        altered[0x100] ^= 1
        with self.assertRaises(ValueError):
            patch_image(bytes(altered))

    def test_verify_rejects_partial_patch_and_unrelated_changes(self) -> None:
        with self.assertRaises(ValueError):
            verify_image(self.original)
        partial = bytearray(self.patched)
        offset = SITES[0][1] - IMAGE_BASE
        partial[offset:offset + 4] = ORIGINAL
        with self.assertRaises(ValueError):
            verify_image(bytes(partial))
        altered = bytearray(self.patched)
        altered[-1] ^= 1
        with self.assertRaises(ValueError):
            verify_image(bytes(altered))

    def test_existing_files_are_not_overwritten(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "original.bin"
            output = Path(directory) / "patched.bin"
            source.write_bytes(self.original)
            with self.assertRaises(FileExistsError):
                patch_file(source, source)
            self.assertEqual(source.read_bytes(), self.original)
            self.assertEqual(patch_file(source, output), self.patched)
            with self.assertRaises(FileExistsError):
                patch_file(source, output)
            self.assertEqual(output.read_bytes(), self.patched)

    def test_fixed_point_conversion_preserves_every_int16_value(self) -> None:
        # Model the ring's left shift and the two conversion semantics. All
        # 65536 inputs (including -32768 and zero) must recover exactly. These
        # shifted integers are also exactly representable in single precision.
        for word in range(-32768, 32768):
            raw_s32 = word << 16
            old_float = struct.unpack("<f", struct.pack("<f", raw_s32))[0]
            corrected_float = struct.unpack("<f", struct.pack("<f", raw_s32 / 65536.0))[0]
            self.assertEqual(old_float, float(word * 65536))
            self.assertEqual(corrected_float, float(word))


class EmulatedConversionTests(unittest.TestCase):
    def test_actual_raw_iq_loop_for_every_int16_value(self) -> None:
        # Optional independent execution check, using the real firmware loop
        # and literal pool on a Cortex-M7 emulator, not a replacement DSP model.
        import unicorn
        from unicorn import arm_const as arm

        original = Path(__file__).with_name("q900_fw.bin").read_bytes()
        for image, scale in ((original, 65536), (patch_image(original), 1)):
            emulator = unicorn.Uc(unicorn.UC_ARCH_ARM, unicorn.UC_MODE_THUMB | unicorn.UC_MODE_MCLASS)
            emulator.ctl_set_cpu_model(arm.UC_CPU_ARM_CORTEX_M7)
            emulator.mem_map(IMAGE_BASE, (len(image) + 4095) // 4096 * 4096)
            emulator.mem_write(IMAGE_BASE, image)
            emulator.mem_map(0x24000000, 0x80000)
            emulator.reg_write(arm.UC_ARM_REG_C1_C0_2, 0x00F00000)
            emulator.reg_write(arm.UC_ARM_REG_FPEXC, 0x40000000)
            for start in range(-32768, 32768, 32):
                i_words = list(range(start, start + 32))
                q_words = [-word - 1 for word in i_words]
                wire = [sample << 16 for pair in zip(i_words, q_words) for sample in pair]
                emulator.mem_write(0x24010000, struct.pack("<64i", *wire))
                emulator.reg_write(arm.UC_ARM_REG_R8, 0x24010000)
                emulator.reg_write(arm.UC_ARM_REG_R3, 0)
                emulator.reg_write(arm.UC_ARM_REG_R4, 32)
                emulator.emu_start(0x08039EE3, 0x08039F18, count=1000)
                self.assertEqual(emulator.reg_read(arm.UC_ARM_REG_PC), 0x08039F18)
                floats = struct.unpack("<64f", emulator.mem_read(0x24004EE8, 256))
                self.assertEqual(floats[:32], tuple(float(word * scale) for word in i_words))
                self.assertEqual(floats[32:], tuple(float(word * scale) for word in q_words))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    patch = commands.add_parser("patch", help="create a new corrected raw application image")
    patch.add_argument("source", type=Path)
    patch.add_argument("destination", type=Path)
    verify = commands.add_parser("verify", help="verify the exact two-instruction correction")
    verify.add_argument("image", type=Path)
    tests = commands.add_parser("self-test", help="test against the repository's original firmware")
    tests.add_argument("--emulate", action="store_true", help="also execute the real raw-I/Q loop (requires unicorn)")
    args = parser.parse_args()
    if args.command == "self-test":
        suite = unittest.defaultTestLoader.loadTestsFromTestCase(PatchTests)
        if args.emulate:
            suite.addTests(unittest.defaultTestLoader.loadTestsFromTestCase(EmulatedConversionTests))
        result = unittest.TextTestRunner(verbosity=2).run(suite)
        return 0 if result.wasSuccessful() else 1
    try:
        if args.command == "patch":
            image = patch_file(args.source, args.destination)
            print(f"Created {args.destination}")
        else:
            image = args.image.read_bytes()
            verify_image(image)
            print(f"Verified {args.image}")
    except (OSError, ValueError) as error:
        parser.exit(1, f"error: {error}\n")
    for channel, address in SITES:
        print(f"{channel}: 0x{address:08X} / file offset 0x{address - IMAGE_BASE:06X}: vcvt.f32.s32 s15, s15, #16")
    print(f"Size: {len(image)} bytes; SHA-256: {sha256(image)}")
    print("Verification covers application bytes; see SDR_TX_GAIN_AUDIT.md for reported hardware results.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
