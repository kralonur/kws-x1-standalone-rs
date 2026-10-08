#!/usr/bin/env python3
"""Flash an exact hash-confirmed firmware image over USB HID.

This is the only tool in this project that touches the device. It is a reviewed
reimplementation of the archived flasher's protocol, with the safety rules below
enforced in code rather than in documentation:

* the exact image SHA-256 must be passed with ``--confirm-sha256``;
* images on the deny list are refused by hash;
* the container framing is verified before anything is written;
* images built by this project must also pass the structural checks in
  ``tools/check_image.py``;
* the payload must be chunk-aligned and its requested erase envelope must stay
  within the established application range [0x10000, 0x44000);
* the device must answer the info command and report bootloader mode;
* ``--dry-run`` performs every check and prints the plan without opening a device.

Protocol (traced from the archived flasher, reimplemented here):

* HID writes are ``report id 0x00`` followed by 64 payload bytes; reads are 64 bytes;
* a command packet is ``05 11 09 <cmd> [args...]`` with ``byte 63 = cmd + sum(args)``;
* command 100 = info, 107 = erase, 101 = batch announce, 103 = batch checksum,
  105 = finish/verify, 255 = reboot;
* response codes: 1 = success, 2 = batch checksum accepted, 4 = verification failed,
  245 = firmware matching error; codes >= 240 are errors, and other unexpected codes
  are informational packets that must be skipped while waiting for the real answer
  (code 5 has been observed between a batch write and its checksum response).
"""
import argparse
import glob
from pathlib import Path
import select
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))
from check_image import check as structural_check  # noqa: E402
from package_app import (  # noqa: E402
    APP_BASE, APP_ERASE_CEILING, CHUNK_BYTES, CHECKSUM_OFFSET, CONTAINER_PREFIX,
    SECTOR_BYTES, ImageError, additive_checksum,
)

TARGET_VID = 0x1105
TARGET_PID = 0x000A

# Retain the source flasher's known-bad image hashes, regardless of file name.
DENY_SHA256 = {
    "6339868eecc09a204ae9f70d00c09c53d46327d800288bb949ad1466224286ea":
        "startup-data corruption negative control",
    "d26f4ba7f66dc0217b4a731e5a7034b3b834e165cd2a01f2182366835e82b60c":
        "implausible voltage and resets",
    "294b268130a6192cd898544fc14aedd78d0b0f69a3b5067830bd8839136284a8":
        "legacy patched image, not built here",
}
PROTECTED = (
    (0x00000, 0x10000, "bootloader and sector 0 (ICG/security configuration)"),
    (APP_ERASE_CEILING, 0x80000, "everything above the application erase envelope, including stock settings"),
)

HEADER = bytes([0x05, 0x11, 0x09])
BATCH_CHUNKS = 128
# Existing transport pacing: five milliseconds between HID data reports.
CHUNK_DELAY = 0.005


class FlashError(RuntimeError):
    """A safety check failed or the device rejected the update."""


def find_hidraw_device(requested=None):
    """Resolve only a matching tester; refuse ambiguous automatic selection."""
    matches = set()
    for uevent in glob.glob("/sys/class/hidraw/hidraw*/device/uevent"):
        try:
            text = Path(uevent).read_text()
        except OSError:
            continue
        for line in text.splitlines():
            if line.startswith("HID_ID="):
                parts = line.split("=", 1)[1].split(":")
                try:
                    matched = len(parts) >= 3 and int(parts[1], 16) == TARGET_VID and int(parts[2], 16) == TARGET_PID
                except ValueError:
                    continue
                if matched:
                    matches.add(f"/dev/{Path(uevent).parent.parent.name}")
            elif line.startswith("PRODUCT="):
                parts = line.split("=", 1)[1].split("/")
                try:
                    matched = len(parts) >= 2 and int(parts[0], 16) == TARGET_VID and int(parts[1], 16) == TARGET_PID
                except ValueError:
                    continue
                if matched:
                    matches.add(f"/dev/{Path(uevent).parent.parent.name}")
    if requested is not None:
        path = str(Path(requested).resolve())
        if path not in matches:
            raise FlashError(f"Device override is not a {TARGET_VID:04x}:{TARGET_PID:04x} tester: {requested}")
        return path
    if len(matches) > 1:
        raise FlashError("Multiple testers found; select one explicitly with --device")
    return next(iter(matches), None)


def command_packet(command, arguments=b""):
    """Build a 64-byte command packet with the protocol checksum."""
    if not 0 <= command <= 255 or len(arguments) > 59:
        raise FlashError("Command or arguments do not fit a 64-byte control report")
    packet = bytearray(64)
    packet[0:3] = HEADER
    packet[3] = command
    checksum = command
    for index, byte in enumerate(arguments):
        packet[4 + index] = byte
        checksum = (checksum + byte) & 0xFF
    packet[63] = checksum
    return bytes(packet)


def chunk_checksum(chunks):
    """16-bit additive checksum over a batch, as the bootloader computes it."""
    total = 0
    for chunk in chunks:
        for byte in chunk:
            total = (total + byte) & 0xFFFF
    return total


class Device:
    """hidraw transport: 64-byte reports, report id 0x00 on writes."""

    def __init__(self, path):
        self.path = path
        self.fd = None

    def open(self):
        import os
        self.path = find_hidraw_device(self.path)
        try:
            self.fd = os.open(self.path, os.O_RDWR)
        except PermissionError as error:
            raise FlashError(
                f"Permission denied opening {self.path}; use sudo or a udev rule for "
                f'{TARGET_VID:04x}:{TARGET_PID:04x}'
            ) from error

    def close(self):
        import os
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def write_report(self, payload):
        import os
        if len(payload) > CHUNK_BYTES:
            raise FlashError("Report larger than 64 bytes")
        padded = bytes(payload).ljust(CHUNK_BYTES, b"\x00")
        written = os.write(self.fd, b"\x00" + padded)
        if written != CHUNK_BYTES + 1:
            raise FlashError(f"Short HID write: {written}/65 bytes; stop without retry")

    def read_report(self, timeout):
        packets = self.read_reports(timeout, 1)
        return packets[0] if packets else None

    def read_reports(self, timeout, limit):
        import os
        deadline = time.monotonic() + timeout
        found = []
        while len(found) < limit and time.monotonic() < deadline:
            ready, _, _ = select.select([self.fd], [], [], 0.05)
            if not ready:
                continue
            packet = os.read(self.fd, CHUNK_BYTES)
            if len(packet) >= CHUNK_BYTES and packet[0:3] == HEADER:
                found.append(packet)
        return found

    def drain(self):
        self.read_reports(0.2, 64)


def expect_response(device, timeout, codes, what, log=None):
    """Wait for one of `codes`, skipping informational packets.

    Codes below 240 that are not expected are informational: the bootloader emits
    them mid-sequence (code 5 has been observed between a batch write and its
    checksum response) and the archived flasher skips them. Only codes >= 240 are
    errors. Aborting on the first unexpected packet leaves the device mid-update.
    """
    deadline = time.monotonic() + timeout
    ignored = []
    while time.monotonic() < deadline:
        packet = device.read_report(max(0.05, deadline - time.monotonic()))
        if packet is None:
            continue
        code = packet[3]
        if code >= 240:
            raise FlashError(f"{what}: device reported error code {code}")
        if code in codes:
            return code
        ignored.append(code)
        if log is not None:
            log(f"{what}: ignoring informational response code {code}")
    detail = f" (saw informational codes {ignored} instead)" if ignored else ""
    raise FlashError(f"{what}: no response within {timeout:.1f}s{detail}")


def query_info(device):
    """Ask for the mode byte, skipping informational packets as the flasher does."""
    device.drain()
    device.write_report(command_packet(100))
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        packet = device.read_report(max(0.05, deadline - time.monotonic()))
        if packet is None:
            continue
        if packet[3] >= 240:
            raise FlashError(f"Info command: device reported error code {packet[3]}")
        if packet[3] == 0:
            return packet[4]
    raise FlashError("Info command: no mode response (code 0); is the tester in Bootloader mode?")


def build_plan(image_path, confirm_sha256):
    """Validate the confirmed bytes before device discovery or I/O.

    Only this firmware's vector layout is accepted; no stock-profile bypass.
    The checked payload is retained in the plan, avoiding a second file read.
    """
    if not image_path.is_file():
        raise FlashError(f"Image must be a regular file: {image_path}")
    data = image_path.read_bytes()
    import hashlib
    digest = hashlib.sha256(data).hexdigest()
    if digest != confirm_sha256:
        raise FlashError(f"Image SHA-256 is {digest}, not the confirmed {confirm_sha256}")
    if digest in DENY_SHA256:
        raise FlashError(f"Refusing a known-bad image: {DENY_SHA256[digest]}")

    report = structural_check(data)
    payload = data[len(CONTAINER_PREFIX):]
    chunks = [payload[index:index + CHUNK_BYTES] for index in range(0, len(payload), CHUNK_BYTES)]
    sectors = report["erase_sectors"]
    return {
        "path": image_path,
        "sha256": digest,
        "prefix": data[:len(CONTAINER_PREFIX)],
        "payload": payload,
        "chunks": chunks,
        "erase_sectors": sectors,
        "erase_envelope": (APP_BASE, APP_BASE + sectors * SECTOR_BYTES),
        "container_checksum": f"{additive_checksum(payload[:CHECKSUM_OFFSET]):#010x}",
    }


def describe(plan):
    low, high = plan["erase_envelope"]
    print(f"image:      {plan['path']}")
    print(f"sha256:     {plan['sha256']}")
    print(f"prefix:     {plan['prefix'].hex()}")
    print(f"payload:    {len(plan['payload'])} bytes in {len(plan['chunks'])} chunks")
    print(f"checksum:   {plan['container_checksum']} (additive, big-endian, before the 0x80 marker)")
    print(f"erase:      {plan['erase_sectors']} sectors = {low:#x}..{high:#x}")
    print("will not touch:")
    for start, end, what in PROTECTED:
        print(f"  {start:#08x}..{end:#08x}  {what}")


def flash(plan, device, progress=print):
    """Run the update sequence. Any unexpected response aborts before the next step."""
    mode = query_info(device)
    if mode != 0:
        raise FlashError(f"Device reports mode byte {mode}; not in Bootloader mode")
    progress("device is in Bootloader mode")

    low, high = plan["erase_envelope"]
    progress(f"erasing {plan['erase_sectors']} sectors ({low:#x}..{high:#x})")
    device.write_report(command_packet(107, bytes([plan["erase_sectors"]]) + plan["prefix"]))
    expect_response(device, 5.0, (1,), "erase")

    chunks = plan["chunks"]
    batches = [chunks[index:index + BATCH_CHUNKS] for index in range(0, len(chunks), BATCH_CHUNKS)]
    for number, batch in enumerate(batches, start=1):
        device.write_report(command_packet(101, bytes([len(batch), number & 0xFF])))
        expect_response(device, 2.0, (1,), f"batch {number} announcement")
        for chunk in batch:
            device.write_report(chunk)
            time.sleep(CHUNK_DELAY)
        total = chunk_checksum(batch)
        device.write_report(command_packet(103, bytes([total & 0xFF, (total >> 8) & 0xFF])))
        expect_response(device, 3.0, (2,), f"batch {number} checksum")
        progress(f"batch {number}/{len(batches)} written and checksum accepted")

    progress("finishing: bootloader verifies the image")
    device.write_report(command_packet(105))
    code = expect_response(device, 5.0, (1, 4), "finish")
    if code == 4:
        raise FlashError(
            "Bootloader reported verification failure (code 4). The image was written but "
            "not accepted. Stop without reboot, retry or restoration; recovery requires "
            "separate exact-image authorization. Code 4 does not identify the cause."
        )
    progress("bootloader accepted the image (code 1)")

    device.write_report(command_packet(255))
    progress("reboot command sent; the tester should now run the new image")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--image", type=Path, help=".X1C image to flash")
    parser.add_argument("--confirm-sha256", help="Exact SHA-256 of that image; required to write")
    parser.add_argument("--dry-run", action="store_true", help="Check and describe only; never opens the device")
    parser.add_argument("--info", action="store_true", help="Query the device without flashing")
    parser.add_argument("--device", help="hidraw path override")
    args = parser.parse_args(argv)
    if args.info and args.dry_run:
        parser.error("--info performs device I/O and cannot be combined with --dry-run")
    if args.info and (args.image is not None or args.confirm_sha256 is not None):
        parser.error("--info cannot be combined with image programming arguments")

    try:
        if args.info:
            path = find_hidraw_device(args.device)
            if not path:
                raise FlashError("No tester found; hold BACK+OK while connecting the USB cable")
            device = Device(path)
            device.open()
            try:
                mode = query_info(device)
                print(f"device: {path}")
                print(f"mode byte: {mode} ({'Bootloader' if mode == 0 else 'application'})")
            finally:
                device.close()
            return 0

        if args.image is None or args.confirm_sha256 is None:
            parser.error("--image and --confirm-sha256 are required (or use --info)")

        plan = build_plan(args.image, args.confirm_sha256)
        describe(plan)
        if args.dry_run:
            print("dry run: no device was opened and nothing was written")
            return 0

        path = find_hidraw_device(args.device)
        if not path:
            raise FlashError(
                "No tester found. Disconnect it, hold BACK+OK, connect USB, release when "
                "'Bootloader' appears."
            )
        device = Device(path)
        device.open()
        try:
            flash(plan, device)
        finally:
            device.close()
        return 0
    except (FlashError, ImageError, OSError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
