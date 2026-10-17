# Firmware safety contract

## Ownership and failure

The application owns clock startup, pins, interrupts, the display and I2C3.
The shared bus mutex covers whole transactions; PD AMS gating holds one lease
from status sampling through TXON without an executor turn. The HAL's bounded
transfer and bus-failure latch remain in place. Unexpected configuration, bus
errors, uncertain traffic and cancellation fail closed, without blind cleanup
writes, retries or a fresh receiver over an unknown FIFO boundary.

The PD session runs on its own interrupt-mode executor, above the thread-mode
tasks, so a frame draw cannot delay a reply past the source's
SenderResponseTimer (a draw measured up to 42 ms; the timer is at least 27 ms).
Its idle receive polls pause `IDLE_POLL_MS` (4 ms) so it does not starve them in
turn: polling every 1 ms took 79% of the CPU, 4 ms about 20%.
Everything it shares with the other tasks is behind a critical-section mutex, a
channel or an atomic.

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

The silent boots (saved as the next boot's mode, kept until left) stop the
sink the same way after the CC check: verified baseline, Rd on, automatic
GoodCRC off, nothing transmitted. A source whose Source_Capabilities are never
acknowledged stops PD after nCapsCount tries and stays at 5 V, where a USB-C
charger may take Quick Charge requests on D+/D−. Under a PD contract the
Quick Charge pages' OK, and the charger page's long OK (the full scan),
enter one through the cable test's Hard Reset restart. After that Hard Reset
a source that hears nothing within NoResponseTimer (4.5-5.5 s) may send up to
nHardResetCount (2) more, then recover by detaching (USB PD R3.2 source
policy engine; the Linux TCPM source does the same). Each costs the
VBUS-powered meter a boot, and each boot stays silent. A Quick Charge
boot that lasted the source's whole PD window (nCapsCount Source_Capabilities
at most tTypeCSendSourceCap apart, 10 s) saves the next boot as normal again,
so only the charger's own restarts, not a later plug-in, stay silent.

A silent boot is left by a user action (OK on the PD profiles, PPS or AVS page,
or the scan's end) with `restart_unheard`: a session starts on the
boot's checked startup and sends Hard Reset Signaling at once, which the
source, having given up on PD, still answers by starting over as at attach.
The PHY refuses it once anything was received. The checked
transition-to-default follows. A start refused only because VBUS is still
outside its window is retried each read period; nothing was written then.
If the source does not answer, the meter stays at 5 V and a replug brings PD
back. The full scan retains QC, then runs read-only SCP type identification
before saving both results for the PD/cable boot. Its saved SCP resume screen
keeps that phase and QC evidence through source restarts. Without PD, both
probes finish directly on Charger info. The final PD/cable part also asks
for the EPR offer and shows the carried results once.

Detection is read-only: only the type registers (0x7E, legacy 0x80/0x81)
and the rating registers (0x90–0x97, 0x9C) are read. Register writes exist
only in the explicit output session, started by a confirmed request on the
SCP page and never by a scan. It writes only the control, watchdog,
boundary and setpoint registers, keeps D+ and temperature protection on,
reads every write back, never retries a write, and ends on any mismatch,
fault or missed setpoint by releasing the lines to the charger's D+
protection and 2 s watchdog. A stop restores ordinary output mode and holds
the lines from Quick Charge until VBUS is back at 5 V, for at most 4 s.
A ping alone is not identification. The adapter verifies 5 V before
entry, checks its D+ drive at 2.6–3.0 V and D− idle low before transmitting,
and verifies 5 V after releasing the lines. The opposite D+ resistor-pair
encoding is not assumed correct: the ADC gate must validate it each attempt.
The physical interface remains hardware-untested.

SCP shares QC's exclusive D+/D− owner and is suppressed during passthrough
or a live PD session. Generation-tagged requests reject stale completions.
A terminal-result watch preserves evidence independently of display timing.
During a synchronous transaction, an exclusive borrow of the idle ADC2 owner
makes PB0 a digital input for 40 µs sync detection; its exact control word is
restored by a drop guard before ADC conversions can resume. GPIO unlocks
are interrupt-guarded. No other pin is reconfigured.

The portable PHY uses a DWT-derived microsecond clock, not the millisecond
Embassy tick. Transmit overruns, slow samples, invalid sync, ping, parity,
CRC or ACK get up to three attempts per register, each after a 100 UI idle
gap or a completed closing ping, then fail; every attempt releases the data
line. A NACK is an answer and is not retried.
Only valid programmable-adapter type responses produce an SCP finding.
The compact finding uses the two formerly reserved settings bits with a
new record tag (0x5f). The QC-only 0x5e records retain their field positions
and remain readable; old records with foreign evidence bits or a foreign
resume screen are rejected, not mislabelled as SCP confirmation. `SCP detected, 5V`
on the overview means verified capability; an applied output shows its
setpoint instead.

The Quick Charge 3.0 steps on its page go up to 12 V, or 20 V after a
probe found Class B, and enter and leave continuous mode through a 5 V
request; Quick Charge 2.0 offers only the voltages the probe saw followed.
The Quick Charge probe runs when Quick Charge first starts in a boot (OK on
a Quick Charge page, or the switch out of PD that OK there asks for), on a
long OK on the charger page without PD, and in the full scan; each page
that starts it warns it tests up to 20 V. It requests continuous mode and its steps first, then 5 V, 9, 12
and up to 20 V, so the load behind the meter must be rated for it, and it
ends on a 5 V request. Its Quick Charge 3.0 step pulse is a 1 ms busy wait,
which only an interrupt can lengthen: its levels also code a fixed request,
which a charger takes after 20 ms at the earliest.

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
only in validated mandatory cases, the user-started cable test restart below
and leaving a silent boot (above), never the optional SinkWaitCapTimer case
and never as generic fault recovery. There is no Cable_Reset or power-role swap.
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

**EPR is explicitly confirmed and bounded** (USB PD 3.2 v1.1). The board
advertises EPR capability in its SPR RDO, EPR Sink Capabilities and SKEDB; the
reserved Fixed Sink PDO bits remain clear. Only fixed 28 V and AVS at or below
30 V/5 A are requestable. The EPR maximum PDP is bounded at 150 W. A real EPR
cable is required; the fake marker remains 20 V/non-EPR.

The offer query alone never enters EPR. Confirmation starts the ordered
Enter/Ack/Succeeded handshake under one continuous entry timer. Independent
policy and native wire gates review mode messages, bounded capability chunks,
exact RDO/PDO copies and keep-alive exchanges. Every request in EPR mode is an
EPR_Request. Exit first establishes an SPR-object contract; stopping first
requests 5 V and exits before reception is disabled. Hard Reset clears EPR
state and uses the EPR transition-to-default timing boundaries.

Keep-alive is scheduled at 375 ms on the interrupt executor, with native
receive cancellation protection. Missing Ack causes Hard Reset. Telemetry
records keep-alive count and maximum observed SOP silence; this is not proof
of compliance under load. Hardware timing with continuous rendering remains
unverified and requires an explicitly approved hardware test.

**The board also retains a nonstandard cable identity** (`EMARKER` in
`pd_spr_phy`), as the source firmware did. Only when the meter's own plug sits
in the charger's receptacle (Rp on both CCs at startup) and the boot runs the
user-started 5 A cable test is SOP' reception on. The test is the last part
of the full scan (a long OK on the charger page, above) and ends with Back on
the charger page. Entering the scan and ending the test, the meter saves the
next boot's mode, ends the session at its idle contract (the stop path, held
1 s), and sends Hard Reset Signaling through the PHY's own Hard Reset gate
(`Session::restart`): the charger cycles VBUS and VCONN and starts over as at
attach, and the checked transition-to-default follows. Releasing Rd did not
make the charger detach, so it is not used. The test runs only on a boot
after a power-up (no reset cause flag other than power-on, reset pin or
multiple), once its mode was cleared and read back before any task starts;
any other boot runs normally and clears the mode later, so neither a reset
during the clear nor a later one can repeat the test. The test boot stays on
the charger page at its 5 V contract: navigation keeps that page and passes only the EPR offer request
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
fit below the footer descriptor at `0x433fc` (209916 bytes), enforced by the
FLASH length in `memory.x` and by the packager. This layout reserves 16 bytes
before the checksum at `0x4340c`; it is not a measured bootloader size limit.
Standalone packaging uses the established checksum/trailer offsets and
`0x44000` erase ceiling. It generates the container directly from the release
application, zero fill and the fixed descriptor. No copied baseline application
code or data is embedded. The descriptor fields remain undecoded. The canonical
payload length is `0x33640` (210,496 bytes, 3,289 chunks), producing the 26-sector
erase range `[0x10000, 0x44000)`. Bootloader and settings regions remain outside
that range. The generated layout passed batch checks and final bootloader
verification on Linux.

The PD stack, protocol crates and FFT tables run from upper flash,
`[0x44000, 0x6c000)`, outside the update erase range. A release's application
is the unpacker followed by two Thumb-filtered LZMA2 streams: the upper
region's and the main firmware's. On its first boot the
unpacker runs entirely from RAM with interrupts disabled. It inflates both
streams and checks their lengths and hashes before writing anything, then
programs the upper region, then the main firmware over itself, and resets.
Writes are fenced to `[0x44000, 0x6c000)` and the application erase range; a
sector that already holds its bytes is skipped, and one is erased only when a
word is neither wanted nor blank. A main firmware sector is erased only when it
lies wholly below the stream bytes still to be read; `firmware-pack` replays
the same unpacking to prove this before it packs. The backlight is lit while it
works and blinks on a failed check or write. Device data at `0x6e000` and
above is never touched. Power loss during the upper writes is resumed on the
next boot; power loss during the main writes leaves the application unusable
until the release is flashed again from bootloader mode. Startup hashes the
upper region against the main firmware's record before anything calls into it,
and shows a missing-code screen rather than running a mismatched region. This result does not qualify every build's runtime
behavior. Image framing and matching hashes do not authorize flashing.

Normal firmware saves settings (brightness, rotation, energy-memory interval)
and does not program the diagnostic log. Settings and logs use the same
append-only writer: blank words only in blank or already-claimed sectors from
`0x72000` through `0x7c000`, never erase settings or active diagnostic sectors,
and never use the final `0x7e000` boot-swap sector. New allocations reserve
`0x78000` and `0x7a000` for the energy journal. Legacy diagnostic sectors inside
that pair are migrated at boot as described below; settings or unknown contents
block initialization instead of being erased. Diagnostic writes exclude the
pair even when a legacy log remains there; existing logs may be read before
migration. Settings search from `0x72000` up, the log from `0x7c000` down, each
with its own claim.
A settings save appends one 8-byte record once a changed value stops being
edited. Four sectors remain for new settings/log allocations; exhausted settings
stay in RAM. Failed attempts are cached as failures, never subsequently reported
as successful just because the attempted value is unchanged. The settings page
shows storage-full versus write-failure status. Old settings decode with energy
memory Off. Startup applies the saved brightness to PWM immediately, rather
than initializing PWM to 60% while the renderer assumes the saved level is
already applied.

Energy memory defaults Off, with no totals writes or restoration on boot.
Enabled intervals are 5, 10, 30 and 60 seconds. Charge/energy integrate fresh
voltage and current-magnitude samples, independent of navigation. Failed samples
or gaps over one second break integration; missing usage is not invented.
Elapsed time since the last reset counts on through them.
The selected interval checkpoints changed totals, and the elapsed time changes
every interval; enabling or changing an enabled interval, and resetting totals,
also checkpoint immediately. Disabling memory
preserves the live counters but ignores old checkpoints on the next boot.

Before I2C, panel DMA or protocol tasks start (and only after the upper code
check succeeds), energy preparation automatically migrates recognized `KWLG`
legacy logs inside the dedicated pair. It is not a blanket erase of unused
flash. Without a committed journal header or migration intent, both sectors
must be entirely blank or have that known legacy ownership tag; settings and
unknown data stay untouched. The legacy log's header has three unused words at
bytes 4..16. One word commits `ENMG` migration intent only after both sectors
were checked; a torn word is skipped in favor of another blank header word,
never reprogrammed. The survivor's tag/intent remain while its partner is erased
and given a complemented energy header. That energy header then remains while
the survivor is erased. A power cut during either erase or claim therefore
retains proof of the pair's ownership, and the next boot resumes preparation.
This one-time migration also runs with Energy memory Off. Blank or initialized
pairs incur no recurring cleanup write or erase; unknown data is never cleared.

Absent migration, the energy journal claims its pair only after verifying both
sectors blank. The complemented version/ownership header in either sector
establishes ownership of both, permitting recovery after a torn erase of its
partner. Boot recovery never erases a sector with a valid energy header, so
existing checkpoints are preserved. A pair in the previous record format
(`EN01`, without elapsed time) migrates once: its sector holding the newest
checkpoint keeps its header while the partner is erased, given that checkpoint
(elapsed time zero) and only then the current header, after which the old
sector is erased. A power cut at any step leaves either the old header and its
checkpoint or the new header and the carried one.
Each 60-byte record contains a sequence, three u64 counters (charge, energy,
elapsed time), their word-wise complements and a final nonblank commit word. Boot scans both claimed sectors,
skips incomplete records and chooses the newest valid sequence. A full sector
rotates to its partner, erasing only the older checkpoint sector. The newest
complete checkpoint and its ownership header remain intact until the replacement
is committed. A write/erase error stops further energy writes for that boot and
is shown on the Energy page, separately from occupied/unclaimable storage.
The read-only System storage section reports all six sector headers, their
erased-word counts, and the last failed flash operation's start address/error
flags this boot. Low bits retain EFM flags; bit 6 means a rejected software
precheck and bit 7 a read-back verification failure. These diagnostics do not
claim sectors or authorize erasing unknown data.
Power loss can lose usage since the last checkpoint;
there is no power-fail interrupt or guaranteed last-second flush.

Both programming and erase narrow and restore the EFM window, run from RAM with
interrupts disabled and read back their result. The sector erase routine is
shared with the unpacker, but the runtime adapter exposes only the energy pair.
The meter holds the I2C lease and waits for panel DMA completion before saving,
so no active I2C transaction or framebuffer transfer is suspended by erase.
SysTick coalesces masked interrupts; runtime erase uses DWT core-cycle counting
and the SysTick phase to replay missed millisecond driver ticks, leaving one
pending hardware tick to the handler. This preserves nominal integration time
but does not remove the interrupt pause. Datasheet rev 1.61 table 3-55 specifies
up to 20 ms plus six HCLK cycles for sector erase. Live PD response timing, the
cycle-counter compensation and power-cut recovery require explicitly approved
hardware validation; host fault-injection tests and a build do not qualify them.

The pair holds 136 checkpoints per sector. Datasheet table 3-56 specifies at
least 10,000 erase cycles (20-year retention at 85 C): two-sector rotation gives
about 2.72 million checkpoints, roughly 157 days at 5s or five years at 60s of
continuous running, not an endurance guarantee. All storage remains
outside the firmware update erase range and survives updates. Flash references:
[erase/program timing and endurance](https://oss-nc-beijing-2.cecloudcs.com/doc-ds/DS_HC32F460SeriesDatasheet_Rev1.61.pdf),
[EFM operations](https://oss-nc-beijing-2.cecloudcs.com/doc-rm/RM_HC32F460_F45x_A460SeriesReferenceManual_Rev1.5.pdf).

The Rust `firmware-pack` CLI takes the built firmware and unpacker, validates
their vectors and sizes, packs the release, generates the standalone layout using
`firmware-image::build_container`, and exclusively creates a new output file.
No original firmware or scripting runtime is required. Its offline audit checks
the confirmed image hash, repacks the supplied builds, and requires the
image to match the reconstructed canonical container byte-for-byte, including
length, zero fill, descriptor, checksum, marker and zero trailer/alignment.
The generated erase envelope is checked against the application ceiling.
These checks do not establish bootloader acceptance or runtime safety.

The Rust `flasher` is the sole host tool that accesses the device. It checks the
exact candidate hash, container framing/checksum and erase ceiling before
discovering hardware, and keeps the validated payload in memory rather than
rereading the image after approval. It has no hash deny list, accepts stock/custom
application layouts and zero-pads a partial final HID report. Neither acceptance
nor padding establishes runtime safety. Bootloader mode is required before erase;
transport/verification errors stop without retries, restoration or reboot.
Only successful verification permits the final reboot command.
`--dry-run` never discovers/opens a device, and refuses combination with `--info`.

No flashing, probing or other device I/O is authorized by a build or migration.
Any hardware operation requires explicit approval; programming requires the
exact image and hash. `--info` is device I/O, not an offline validation command.
