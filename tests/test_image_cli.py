"""Offline CLI regressions for destructive image and dry-run failure modes.

These use synthetic files, never HID discovery or device access. The fixtures
encode the established container/vector format independently of the tools.
"""
import hashlib
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
# Established content/trailer layout, 64-byte alignment and graphics boundary.
CHECKSUM = 0x3340C
MARKER = 0x33410
PAYLOAD_BYTES = 0x33440
GRAPHICS = 0x24000
# PAC vector layout and the firmware's permitted custom interrupt lines.
CUSTOM_IRQS = (4, 8, 13, 86, 87, 88, 110, 111, 112, 113)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def container(payload):
    payload[CHECKSUM:CHECKSUM + 4] = (sum(payload[:CHECKSUM]) & 0xFFFFFFFF).to_bytes(4, "big")
    return b"\x00\x0a" + payload


class ImageCli(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.work = Path(self.directory.name)
        self.app = self.work / "app.bin"
        self.baseline = self.work / "baseline.X1C"
        self.output = self.work / "candidate.X1C"
        # A 1 KiB synthetic application with vectors inside its own code region.
        app = bytearray(1024)
        vectors = [0x2000EF00] + [0] * 15 + [0x10301] * 144
        vectors[1], vectors[15] = 0x10281, 0x10305
        for line in CUSTOM_IRQS:
            vectors[16 + line] = 0x10303
        struct.pack_into("<160I", app, 0, *vectors)
        self.app.write_bytes(app)
        payload = bytearray([0xA5]) * PAYLOAD_BYTES
        payload[MARKER] = 0x80
        payload[MARKER + 1:] = bytes(PAYLOAD_BYTES - MARKER - 1)
        self.baseline.write_bytes(container(payload))
        self.baseline_hash = digest(self.baseline.read_bytes())

    def run_cli(self, tool, *arguments):
        return subprocess.run([sys.executable, str(ROOT / "tools" / tool), *map(str, arguments)],
                              cwd=self.work, capture_output=True, text=True)

    def package(self, baseline_hash=None):
        return self.run_cli("package_app.py", "--app", self.app, "--baseline", self.baseline,
                            "--baseline-sha256", baseline_hash or self.baseline_hash,
                            "--out", self.output)

    def audit(self):
        return self.run_cli("check_image.py", "--image", self.output,
                            "--confirm-sha256", digest(self.output.read_bytes()),
                            "--app", self.app, "--baseline", self.baseline,
                            "--baseline-sha256", self.baseline_hash)

    def test_roundtrip_preserves_every_non_application_byte_except_checksum(self):
        result = self.package()
        self.assertEqual(result.returncode, 0, result.stderr)
        image, prior = self.output.read_bytes()[2:], self.baseline.read_bytes()[2:]
        n = self.app.stat().st_size
        self.assertEqual(image[:n], self.app.read_bytes())
        self.assertEqual(image[n:CHECKSUM], prior[n:CHECKSUM])
        self.assertEqual(image[CHECKSUM + 4:], prior[CHECKSUM + 4:])
        self.assertEqual(self.audit().returncode, 0)
        result = self.run_cli("flash_app.py", "--dry-run", "--image", self.output,
                              "--confirm-sha256", digest(self.output.read_bytes()))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("26 sectors = 0x10000..0x44000", result.stdout)
        self.assertIn("no device was opened", result.stdout)

    def test_unconfirmed_baseline_does_not_create_output(self):
        self.assertNotEqual(self.package("0" * 64).returncode, 0)
        self.assertFalse(self.output.exists())

    def test_invalid_binary_is_rejected_before_writing(self):
        good = self.app.read_bytes()
        wrong_irq = bytearray(good)
        struct.pack_into("<I", wrong_irq, (16 + CUSTOM_IRQS[0]) * 4, 0x10301)
        for app in [good[:639], good + bytes(GRAPHICS + 1 - len(good)), wrong_irq]:
            with self.subTest(length=len(app)):
                self.app.write_bytes(app)
                self.assertNotEqual(self.package().returncode, 0)
                self.assertFalse(self.output.exists())

    def test_malformed_or_overlong_baseline_is_rejected(self):
        good = self.baseline.read_bytes()
        bad_checksum = bytearray(good)
        bad_checksum[2 + CHECKSUM] ^= 1
        bad_trailer = bytearray(good)
        bad_trailer[-1] = 1
        for data in [b"xx" + good[2:], good[:-1], bad_checksum, bad_trailer,
                     good + bytes(4096)]:
            with self.subTest(length=len(data)):
                self.baseline.write_bytes(data)
                self.assertNotEqual(self.package(digest(data)).returncode, 0)
                self.assertFalse(self.output.exists())

    def test_existing_output_is_not_overwritten(self):
        self.output.write_bytes(b"keep this file")
        self.assertNotEqual(self.package().returncode, 0)
        self.assertEqual(self.output.read_bytes(), b"keep this file")

    def test_audit_rejects_graphics_change_even_with_valid_checksum_and_hash(self):
        self.assertEqual(self.package().returncode, 0)
        payload = bytearray(self.output.read_bytes()[2:])
        payload[GRAPHICS] ^= 1
        self.output.write_bytes(container(payload))
        result = self.audit()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("outside the application/checksum", result.stderr)

    def test_dry_run_refuses_info_and_unconfirmed_image(self):
        result = self.run_cli("flash_app.py", "--info", "--dry-run")
        self.assertEqual(result.returncode, 2)
        self.assertIn("cannot be combined", result.stderr)
        self.assertEqual(self.package().returncode, 0)
        result = self.run_cli("flash_app.py", "--dry-run", "--image", self.output,
                              "--confirm-sha256", "0" * 64)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("not the confirmed", result.stderr)


if __name__ == "__main__":
    unittest.main()
