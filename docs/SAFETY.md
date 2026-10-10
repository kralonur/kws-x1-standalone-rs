# Firmware safety contract

## Ownership and failure

The application owns clock startup, pins, interrupts, the display and I2C3.
The shared bus mutex covers whole transactions; PD AMS gating holds one lease
from status sampling through TXON without an executor turn. The HAL's bounded
transfer and bus-failure latch remain in place. Unexpected configuration, bus
errors, uncertain traffic and cancellation fail closed, without blind cleanup
writes, retries or a fresh receiver over an unknown FIFO boundary.

Controller configuration is checked against the supplied baseline and read back
after writes. Interrupt registers are read-clear, never written. FIFO reads are
consuming operations, not a recoverable stream: the receiver marks itself halted
before its first await and only clears that mark on successful completion.
Packets have private, immutable bytes and are CRC-checked before publication.
Frames remain bounded to 35 bytes; extended traffic supports single chunks with
at most 26 data bytes, not arbitrary reassembly.

## Independent PD gates

All of these must authorize a transaction:

1. The pinned `pd-protocol` policy wrapper and its receive/transmit allowlists.
2. `fusb302-pd/src/phy.rs`, including native receive authorization and `send_sop`.
3. The portable framing decoder and local `service.rs` FIFO boundary checks.
4. The pinned `usbpd` engine, protocol layer and device policy manager.

New message support must be reviewed at every gate, not just the engine.
Board limits, scheduling, voltage measurements and routing remain caller inputs.
The sink uses the unassigned identity convention; no VID or metadata is invented.
Protocol responses are not proof of physical fault detection or protection.

## Modes, resets and deviations

The slider is sampled three times at boot. Only three low samples select a
sink session; an uncertain level selects passthrough. Passthrough releases Rd
from a verified baseline and then listens without GoodCRC, transmission or
reset signaling. A changed configuration or any transmit evidence halts it.
The hold diagnostic keeps Rd but does not start the policy engine.

The sink selects the CC that alone carries Rp. When both do (the meter's plug
sits in the charger's receptacle, which drives Rp on both pins), it listens
300 ms on each CC, Rd kept and nothing transmitted, and uses the one CC with
BMC activity; no activity or activity on both refuses the session as before.
During that listen SOP'/SOP'' reception is on with AUTO_CRC off, so cable
packets are only counted, never acknowledged; it is switched off and the FIFO
flushed before the baseline is verified again.
When exactly one quiet CC is above the 2.1 V MDAC level (the charger powers
VCONN there; Rd lines stay below 2.04 V), the other CC is taken as the
talking line and the listen is skipped, as the source firmware decided from
its CC voltages, so the session can acknowledge the charger's early SOP'
Discover Identity requests.

SOP Soft_Reset is independently gated in policy and PHY. Hard Reset is allowed
only in validated mandatory cases and the user-started cable test restart
below, never the optional SinkWaitCapTimer case and never as generic fault
recovery. There is no Cable_Reset or power-role swap.
After sent/received Hard Reset, the checked transition-to-default restores the
baseline without changing Rd, observes VBUS below vSafe0V and back at vSafe5V,
then restarts the MCU. Because the meter is VBUS-powered, it may lose power first.
BIST handling retains its separately armed and bounded path.
A session that never heard the source (no packet received, nothing sent, no
PHY error), as on a port without PD, restores and verifies the same baseline
and keeps the shared bus open. Any other failed session leaves the PHY as it is
and latches the bus.

Sink-initiated AMS normally requires a quiet line, empty RX FIFO and, under PD3,
SinkTxOK. **The board explicitly retains a nonstandard stale-SinkTxNG override**
(USB PD R3.2 section 7.2): after a quiet 100 ms window, a final atomic quiet-line
and empty-FIFO sample permits sending. It is not a compliance claim or a default
for other callers. Missing GoodCRC still follows the gated protocol-error path. With the meter's plug in the
charger (Rp on both CCs) a quiet Rp default level is treated the same way: the
talking CC was measured at BC_LVL 1 for a whole PD3 contract.

Cable inspection retains explicit CC2 VCONN routing and bounded SOP'/SOP''
queries in the reviewed fixed-5 V scope, not general VCONN role negotiation. It never
starts in a session whose startup saw Rp on both CCs: CC2 is then the
charger's own CC pin, so VCONN would be driven into the charger.
The board's 51 ms receive and 1000 ms no-progress bounds, request limits,
refresh intervals and UI confirmation behavior are unchanged.

**The board also retains a nonstandard EPR offer request** (USB PD R3.2
section 6.5.14.1 reserves EPR_Get_Source_Cap for EPR capable Sinks). A long OK
on the charger page sends it once, only from an idle PD3 contract with a Source
whose first PDO sets EPR Mode Capable. Both gates allow exactly its bytes, the
Chunk request for the next Chunk of EPR_Source_Capabilities and that Chunk; the
answer is shown and never requested (no EPR_Request, no EPR Mode). This turns
on the engine's chunking layer: another multi-Chunk Message now fails closed
instead of getting Not_Supported. Not_Supported or another Message instead of
the answer is a Protocol Error (Soft_Reset); a deferred request returns to
Ready and is asked again after the cooldown, at most three unsent attempts.

**The board also retains a nonstandard cable identity** (`EMARKER` in
`pd_spr_phy`), as the source firmware did. Only when the meter's own plug sits
in the charger's receptacle (Rp on both CCs at startup) and the boot runs the
user-started 5 A cable test is SOP' reception on. The test starts and ends
with a long OK / Back on the charger page. The meter saves a one-boot flag,
ends the session at its idle contract (the stop path, held 1 s), and sends
Hard Reset Signaling through the PHY's own Hard Reset gate (`Session::restart`):
the charger cycles VBUS and VCONN and starts over as at attach, and the
checked transition-to-default follows. Releasing Rd did not make the charger
detach, so it is not used. The test runs only on a boot after a power-up (no reset cause flag other
than power-on, reset pin or multiple), once its flag was cleared and read
back before any task starts; any other boot runs normally and clears the
flag later, so neither a reset during the clear nor a later one can repeat
the test. The test boot stays on the charger page at its
5 V contract: navigation keeps that page and passes only the EPR offer request
and the restart, and the PD session drops any other command. While the test runs:
that plug is then the cable between charger and meter (board rating 4-30 V,
0-12 A). A SOP' Discover Identity from the charger is acknowledged by the
FUSB302 automatic GoodCRC (its Cable Plug bit is the sink role bit, clear) and
answered with a 5 A, 20 V passive cable identity; other SOP' Messages get no
reply. The reply keeps the port's own GoodCRC timer origin and transmit
evidence. A cable that follows the meter's receptacle is not covered by this
identity: the load behind the meter must be rated for what is requested. SOP'
reception is switched off again before the baseline is verified.

## Startup, memory and persistent storage

Clock startup parks on HRC before changing PLL factors and verifies the final
state before publishing frequencies. The display retains its initialization,
window offsets and wire byte order. SRAM2 holds only the 64.8 KiB framebuffer;
startup initializes it before tasks run. The stack stays in SRAMH/SRAM1, leaving
SRAM3 unused. The painted stack remains observable on the IRQ page.

The application starts at `0x10000`. Release code plus initialized data must
fit below the container checksum at `0x4340c` (209920 bytes, ending at
`0x43400`), enforced by the FLASH length in `memory.x` and by the packager. The
original firmware's graphics bank from `0x34000` is unused and may be
overwritten. The container and its `0x44000` erase ceiling are unchanged, so an
update writes the same range as before; packaging preserves every byte after the
application except the checksum.
Image framing uses the existing checksum and
trailer convention; those alone do not authorize flashing.

Normal firmware saves its settings (brightness, rotation) and does not program
the diagnostic log. Both use the same append-only writer: blank words only, in
blank or already-claimed sectors from `0x72000` through `0x7c000`, never erase,
and never the final `0x7e000` sector. Settings claim sectors with their own
claim word, searching from `0x72000` up; the log searches from `0x7c000` down
and never takes a sector the other claimed. A settings save appends one 8-byte
record when a changed value stops being edited; a sector holds about 1000
records, then the next blank sector is claimed. With no blank sector left the
settings stay in RAM. The flash programming window is narrowed and restored,
the programming routine runs from RAM with interrupts disabled, and error flags
fail the write without retries. These sectors are outside the update erase
range, so saved settings survive firmware updates.

Packaging requires the exact approved baseline hash, validates the built
application's vectors and size, preserves every byte outside the application
and checksum, and never overwrites an existing output. The offline audit checks
those properties again against the supplied binary and baseline.

The flashing tool checks the exact candidate hash, known-bad deny list, firmware
vector layout, chunk alignment and erase ceiling before discovering hardware.
It keeps the validated payload in memory rather than rereading the image after
approval. There is no structural-profile bypass. Bootloader mode is required
before erase; transport/verification errors stop without retries, restoration
or reboot. Only successful verification permits the final reboot command.
`--dry-run` never discovers/opens a device, and refuses combination with `--info`.

No flashing, probing or other device I/O is authorized by a build or migration.
Any hardware operation requires explicit approval; programming requires the
exact image and hash. `--info` is device I/O, not an offline validation command.
