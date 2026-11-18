# USB install

The standard build and packaging workflow still emits the same standalone image
if it fits. If the unpacker and both streams won't fit in the 26 bootloader
sectors, the packer chooses a two-stage image. The bootloader writes the USB
unpacker and the main stream. The unpacker then takes the upper region from the
flasher over USB HID and installs the main firmware as usual. It is one file and
one flasher run, with no manual reconnect.

A two-stage image requires the matching project flasher; don't load it into a
vendor tool or an older flasher. Tagged release publishing of two-stage images
is blocked pending hardware recovery validation and a final USB identity.

## Force a two-stage image for testing

For normal builds and automatic selection, see [Package and audit offline](../README.md#package-and-audit-offline).

```sh
rustup target add thumbv7em-none-eabihf
cargo build --release -p firmware --target thumbv7em-none-eabihf --locked \
  --features usb-install

ELF=target/thumbv7em-none-eabihf/release
mkdir -p build
cargo run --release -p firmware-pack --locked -- pack --usb-install \
  --firmware "$ELF/firmware" --unpacker "$ELF/unpacker" \
  --out build/firmware-usb.X1C > build/package-usb.json
hash=$(sha256sum build/firmware-usb.X1C | cut -d ' ' -f 1)
cargo run --release -p firmware-pack --locked -- audit --usb-install \
  --image build/firmware-usb.X1C --confirm-sha256 "$hash" \
  --firmware "$ELF/firmware" --unpacker "$ELF/unpacker" > build/audit-usb.json
cargo run --release -p flasher --locked -- \
  --dry-run --image build/firmware-usb.X1C --confirm-sha256 "$hash"
```

`pack --usb-install` refuses an unpacker built without the feature. The file
is the normal 210498-byte container, then a header with the unpacker's record
words, then the raw upper region. The flasher and the audit check that the
record in the container matches those words and that the region matches its
hash. A USB install container without its upper region is refused before any
device is opened, since it cannot complete.

## Flash

On Linux, the installer needs hidraw access like the bootloader. In
`/etc/udev/rules.d/70-kws-x1.rules`, then `sudo udevadm control --reload`:

```
KERNEL=="hidraw*", SUBSYSTEM=="hidraw", ATTRS{idVendor}=="1105", ATTRS{idProduct}=="000a", TAG+="uaccess"
KERNEL=="hidraw*", SUBSYSTEM=="hidraw", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="05df", TAG+="uaccess"
```

Connect only one meter during a USB install. `--device` selects the
bootloader the install starts from, not the installer after the reboot, and
with `--device` the flasher never resumes a waiting installer.

Hold **Back + OK** while connecting USB, then run the dry-run command above
without `--dry-run`. After the bootloader batches the flasher prints:

```text
Bootloader accepted the image; reboot command sent
Waiting for the meter's installer; keep the cable connected
Upper region sector written bytes=8192 total=...
...
Transfer complete: the meter verified the upper region
```

The flasher stops there and warns that installation is not complete. The
meter then writes the main firmware and restarts into the measurement screen.
Keep it connected until then.

Before sending anything, the flasher compares the installer's record with
the file's: data offset, upper length and hash, main length, hash and stream
length. A different build is refused, even with the same upper region. If
the meter already holds this build's upper region, it prints
`Transfer skipped: the meter already holds this build's upper region`.

## If the meter is left waiting in the installer

The backlight is on, the screen stays blank and the meter enumerates as
`16c0:05df`, product `KWS-X1 Installer`. This happens when the flasher stops
or the power is lost after the bootloader step.

- **Same file at hand:** connect the meter without holding any button and run
  the same flash command. The flasher prints
  `The meter is waiting in the installer; resuming` and finishes.
- **Several meters waiting:** the flasher lists them and stops. Disconnect
  all but one.
- **File lost, or a different build:** the flasher reports
  `The meter's installer is from a different build than this file` and sends
  nothing. Hold **Back + OK** while connecting USB and flash any release.
- **Upper region already this build's:** without a flasher, the installer
  waits 10 s, then installs the main firmware by itself.

A blinking backlight means the unpacker failed. Hold **Back + OK** while
connecting USB and flash again.
