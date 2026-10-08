#!/usr/bin/env python3
"""Package a raw application over an explicitly hash-confirmed baseline, offline."""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import sys

# Bootloader container layout retained from the existing image format.
APP_BASE = 0x10000
PAYLOAD_LEN = 0x33420
CHECKSUM_OFFSET = 0x3340C
MARKER_OFFSET = 0x33410
CONTAINER_PREFIX = b"\x00\x0a"
# Executable bytes stop before the installed graphics bank at 0x34000.
APP_LIMIT = 0x24000
# Container trailer marker from the existing image format.
MARKER = 0x80
# cortex-m-rt emits 16 exceptions plus the PAC's 144 interrupt entries.
VECTOR_COUNT = 160
FIRST_IRQ = 16
SYSTICK_EXCEPTION = 15
# Only the board's observation/self-test, tilt, SPI3 and I2C3 IRQs differ.
CUSTOM_IRQS = {4, 8, 13, 86, 87, 88, 110, 111, 112, 113}
# Startup places the stack strictly inside SRAM1, not on its exclusive end.
SRAM1_START = 0x20000000
SRAM1_END = 0x20010000
# Existing HID transport uses 64-byte chunks and 8 KiB erase sectors.
CHUNK_BYTES = 64
SECTOR_BYTES = 0x2000
APP_ERASE_CEILING = 0x44000


class ImageError(ValueError):
    """An image violates the framing, layout or preservation contract."""


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def confirmed_bytes(path, digest):
    data = path.read_bytes()
    actual = sha256(data)
    if actual != digest:
        raise ImageError(f"SHA-256 is {actual}, not the confirmed {digest}")
    return data


def additive_checksum(content):
    return sum(content) & 0xFFFFFFFF


def erase_sectors_for(payload_len):
    # Preserve the existing flasher convention: chunks // 128 + 1.
    chunks = (payload_len + CHUNK_BYTES - 1) // CHUNK_BYTES
    return chunks // (SECTOR_BYTES // CHUNK_BYTES) + 1


def verify_container(data):
    if data[:len(CONTAINER_PREFIX)] != CONTAINER_PREFIX:
        raise ImageError("Unexpected container prefix")
    payload = data[len(CONTAINER_PREFIX):]
    if len(payload) < PAYLOAD_LEN:
        raise ImageError("Container is shorter than the established payload")
    if len(payload) % CHUNK_BYTES:
        raise ImageError("Payload must be aligned to 64-byte HID chunks")
    sectors = erase_sectors_for(len(payload))
    end = APP_BASE + sectors * SECTOR_BYTES
    if end > APP_ERASE_CEILING:
        raise ImageError("Erase envelope exceeds the protected application ceiling")
    checksum = additive_checksum(payload[:CHECKSUM_OFFSET])
    if int.from_bytes(payload[CHECKSUM_OFFSET:CHECKSUM_OFFSET + 4], "big") != checksum:
        raise ImageError("Container checksum mismatch")
    if payload[MARKER_OFFSET] != MARKER or any(payload[MARKER_OFFSET + 1:]):
        raise ImageError("Invalid marker or nonzero trailer/alignment bytes")
    return {
        "sha256": sha256(data),
        "payload_bytes": len(payload),
        "erase_sectors": sectors,
        "erase_envelope": [APP_BASE, end],
    }


def check_application(app):
    if not VECTOR_COUNT * 4 <= len(app) <= APP_LIMIT:
        raise ImageError("Application is truncated or overlaps the graphics bank")
    vectors = struct.unpack_from(f"<{VECTOR_COUNT}I", app)
    if not SRAM1_START < vectors[0] < SRAM1_END:
        raise ImageError("Initial stack pointer is not strictly inside SRAM1")
    for index in [1, SYSTICK_EXCEPTION, *range(FIRST_IRQ, VECTOR_COUNT)]:
        address = vectors[index]
        if not address & 1 or not APP_BASE < address < APP_BASE + len(app):
            raise ImageError(f"Vector {index} is not a Thumb address inside the application")
    default = vectors[FIRST_IRQ]
    custom = {i - FIRST_IRQ for i in range(FIRST_IRQ, VECTOR_COUNT)
              if vectors[i] != default}
    if custom != CUSTOM_IRQS:
        raise ImageError(f"Unexpected custom IRQ lines: {sorted(custom)}")
    if vectors[SYSTICK_EXCEPTION] == default:
        raise ImageError("SysTick uses the default interrupt handler")
    return {"initial_sp": vectors[0], "reset_vector": vectors[1],
            "systick_vector": vectors[SYSTICK_EXCEPTION], "custom_irq_lines": sorted(custom)}


def package(app, baseline):
    verify_container(baseline)
    check_application(app)
    payload = bytearray(baseline[len(CONTAINER_PREFIX):])
    payload[:len(app)] = app
    payload[CHECKSUM_OFFSET:CHECKSUM_OFFSET + 4] = additive_checksum(
        payload[:CHECKSUM_OFFSET]
    ).to_bytes(4, "big")
    candidate = CONTAINER_PREFIX + bytes(payload)
    audit(candidate, baseline, app)
    return candidate


def audit(candidate, baseline, app):
    report = verify_container(candidate)
    verify_container(baseline)
    check_application(app)
    if len(candidate) != len(baseline):
        raise ImageError("Candidate length differs from the baseline")
    payload = candidate[len(CONTAINER_PREFIX):]
    prior = baseline[len(CONTAINER_PREFIX):]
    if payload[:len(app)] != app:
        raise ImageError("Candidate application differs from the supplied binary")
    if (payload[len(app):CHECKSUM_OFFSET] != prior[len(app):CHECKSUM_OFFSET]
            or payload[CHECKSUM_OFFSET + 4:] != prior[CHECKSUM_OFFSET + 4:]):
        raise ImageError("Bytes outside the application/checksum changed")
    report.update(application_bytes=len(app), baseline_sha256=sha256(baseline),
                  application_sha256=sha256(app), graphics_preserved=True)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--app", type=Path, required=True, help="Raw application binary")
    parser.add_argument("--baseline", type=Path, required=True, help="Preserved container")
    parser.add_argument("--baseline-sha256", required=True, help="Exact approved baseline hash")
    parser.add_argument("--out", type=Path, required=True, help="New container path; never overwritten")
    args = parser.parse_args()
    try:
        app = args.app.read_bytes()
        baseline = confirmed_bytes(args.baseline, args.baseline_sha256)
        candidate = package(app, baseline)
        report = audit(candidate, baseline, app)
        args.out.parent.mkdir(parents=True, exist_ok=True)
        with args.out.open("xb") as output:
            output.write(candidate)
        report.update(candidate=str(args.out), status="prepared, not flashed")
        print(json.dumps(report, indent=2))
        return 0
    except (OSError, ImageError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
