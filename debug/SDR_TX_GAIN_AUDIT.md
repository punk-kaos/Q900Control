# SDR transmit fixed-point scaling audit

## Scope

Audited `debug/q900_fw.bin`, SHA-256:

```text
f5fd15145c61578cad14ddbce666373888d43d50161e5783db9886996c624915
```

The operator confirmed that the radio runs **3.7.6** and this is its correct
firmware image. The symptom being investigated is rough SDR USB transmit audio
from JS8Call Tune that improves substantially when JS8Call's output level is
reduced.

The image loads at **0x08020000**. The checked-in `q900_dis.txt` disassembly uses
0x08000000 instead; add 0x20000 to its printed addresses. Using its printed
addresses directly against the application's firmware references examines the
wrong functions.

## Verified data path

1. **Network ring consumer, `0x0806C954`.** Instructions at `0x0806C990–0x0806C9A0`
   read each stereo/IQ halfword and shift it left by 16 before storing it as a
   32-bit sample. Both channels receive this conversion, without a stream-format
   exception:

   ```text
   input_s32 = signed_int16_word * 65536
   ```

2. **Normal-audio conversion, `0x080397BC`.** Stream format 1 selects unity at
   `0x0803981E`, then multiplies by the constant `0x37800000` at `0x08039890`,
   which is float `2**-16`. The vector scale call at `0x08039884` applies this
   factor. The speech/modulation path therefore receives samples in int16 units.

3. **Raw-I/Q branch, `0x08039E8E`.** With stream format 2 and the other raw-I/Q
   gates satisfied, `0x08039EE2–0x08039F16` copies the interleaved 32-bit samples
   into separate float I and Q buffers using `vcvt.f32.s32`. There is **no**
   `2**-16` scaling in this loop. It loads a mode factor of approximately 1.133
   from `0x0803A15C` and jumps to `0x0803A082`, bypassing speech processing and
   speech ALC.

4. **Shared output stage, `0x08039C38`.** The constant at `0x08039D04` is
   `0x47800000`, float 65536. It is multiplied by the mode factor, runtime power
   factor at radio-state offset 0x44, and I/Q calibration factors. The resulting
   gains are applied to the float buffers. After phase correction, the loop at
   `0x08039CCC–0x08039CF4` converts them to signed 32-bit output samples with
   `vcvt.s32.f32`.

The raw branch thus carries an **extra factor of 65536 (96.33 dB)** into the
shared output stage compared with int16-unit DSP samples. Exact overload on a
live radio depends on its runtime power/calibration values and selected path;
those values are not available from an offline image. A host payload free from
clipping does not rule out overflow/saturation at the radio's output conversion.

## Correction boundary

The missing normalization belongs in the firmware's raw-I/Q branch, before the
shared output stage, so both I and Q use the same units as the other modulation
paths. A corrected application image has now been generated as described below.
It has been verified offline, including instruction emulation. The operator has
now installed it and reports that the gain controls work, but received audio is
still scratchy. The scaling correction is therefore not a complete explanation
or resolution of the audio-quality problem.

Dividing the host's S16LE payload by 65536 is not a usable equivalent: even its
largest positive value becomes `32767 / 65536 = 0.4999847` of one integer count.
The current truncating serializer would produce silence, not a clean 16-bit
signal. A smaller arbitrary attenuation can reduce overload but is not a
correction of the format mismatch.

The host's SSB envelope limiter protects host I/Q headroom only. It cannot undo
this later gain. Likewise, applying the normal-audio COMPRESSOR/ALC model to SDR
is incorrect: this branch bypasses it. The host UI now shows measured outgoing
I/Q envelope dBFS rather than telling the operator that SDR is `alc UNDER`.

## Generated correction

File: `debug/q900_fw_3.7.6_sdr_iq_gain_fix.bin`

Size: **1,134,372 bytes**, the same as the original.

SHA-256:

```text
f99d158bfc47e590266faab2aa863519cbbf6a8db5f0961a2c8a27dcf54bb0fc
```

The patch changes these two instructions in the raw-I/Q branch:

| Channel | Flash address | File offset | Original bytes | Corrected bytes |
| --- | --- | --- | --- | --- |
| I | `0x08039EF4` | `0x019EF4` | `f8 ee e7 7a` | `fa ee c8 7a` |
| Q | `0x08039F0A` | `0x019F0A` | `f8 ee e7 7a` | `fa ee c8 7a` |

```asm
; Original: signed integer to float, retaining the ring's x65536 scale.
vcvt.f32.s32 s15, s15
; Corrected: signed fixed-point to float, with 16 fractional bits.
vcvt.f32.s32 s15, s15, #16
```

Both are four-byte Thumb/VFP instructions supported by the Cortex-M7 FPv5 FPU.
The two replacements change **four bytes total**. They preserve registers,
instruction addresses, branches, image length and the vector table. The
alternate USB ring reader at `0x0805BAB0` uses the same input left shifts, so the
conversion is appropriate for that raw-I/Q source too. Normal-audio modulation
does not execute this raw-I/Q loop.

### Reproduce and verify

The patcher requires the exact original SHA-256 and refuses to overwrite an
existing output. To regenerate, choose a new output filename if the generated
file already exists:

```bash
python3 -B debug/patch_sdr_iq_gain.py patch debug/q900_fw.bin debug/q900_fw_3.7.6_sdr_iq_gain_fix.bin
python3 -B debug/patch_sdr_iq_gain.py verify debug/q900_fw_3.7.6_sdr_iq_gain_fix.bin
python3 -B debug/patch_sdr_iq_gain.py self-test
```

Verification restores only those two instructions in memory and requires the
result to hash to the original image. Any other changed byte, a partial patch,
or a differently built 3.7.6 image is rejected.

Independent instruction checks performed:

- GNU Arm assembler generated `eefa 7ac8` for the fixed-point conversion using
  `.cpu cortex-m7` and `.fpu fpv5-d16`.
- GNU Arm objdump decoded both instructions in the generated image as the
  intended fixed-point conversions.
- Unicorn 2.1.4 emulated the **actual original and corrected firmware loops**,
  including their literal pool and I/Q stores, on its Cortex-M7 model. Every
  signed 16-bit value was exercised on both channels. The original output was
  `word * 65536`; the corrected output was exactly `float(word)` for all values.

The emulation test is reproducible with the optional `unicorn` Python package:

```bash
python3 -B debug/patch_sdr_iq_gain.py self-test --emulate
```

### Image format and on-radio validation

This file is a raw application image beginning with its vector table at
`0x08020000`. Its final bytes are application data/pointers, not an identified
update-container header or checksum field. The bootloader below `0x08020000` is
not included, so its update validation, filename requirements, or any external
package checksum/signature requirements cannot be established from this image.
The patch does not invent or alter an unidentified integrity field.

Use the radio's established update procedure for raw application images; any
packaging required by that procedure still needs to be handled by its update
tool. The patch utility only generates and verifies bytes and does not flash a
device. The original `debug/q900_fw.bin` is retained as the rollback image.
The firmware version label remains 3.7.6; distinguish the images by their hashes.

After installation, start with reduced drive/power and compare JS8Call Tune on
the second receiver while increasing the audio level gradually. Check a sustained
tone and then actual JS8 traffic. This correction restores the digital sample
units; it does not guarantee that a particular radio's analog gain/calibration
remains linear at maximum drive. The operator's installation succeeded and gain
control improved; clean on-radio audio has **not** yet been achieved.

### Follow-up after the gain fix

A 30-second local run of the current host SDR sender, with the internal 1500 Hz
source and a configured radio rate of 999.4 packets/s, produced zero capture
overflows, underruns, dropped blocks, failed sends and DSP clips. The recorded
complex waveform had approximately 0.002% envelope ripple and 0.000026 rad RMS
phase residual. Replaying its timestamps against the radio's 32-frame DSP
consumption and ring-correction thresholds predicted no post-priming frame drops,
duplicates or ring overflows. This used a mock send socket: it does not validate
the live network, JS8Call's virtual audio source or the radio's real-time DSP.

The next comparison is the internal tone against JS8Call Tune at matched outgoing
I/Q levels. `Q900_IQ_TX_TONE_LEVEL=0.1` provides a low-drive internal source, about
-23 dBFS outgoing I/Q for USB at 1500 Hz, so the comparison does not require the
original near-full-scale test tone. See the README's known-tone procedure.

### Live captures and the PTT startup race

The operator subsequently recorded `/tmp/q900-internal` and `/tmp/q900-js8` and
reported that both sounded similarly scratchy on the receiving radio.

| Capture | Duration | I/Q RMS | Steady-tone envelope ripple | Largest send gap |
| --- | --- | --- | --- | --- |
| Internal tone | 6.16 s | -22.94 dBFS | about 0.013% | 14.191 ms |
| JS8Call | 5.77 s | about -7.96 dBFS in steady sections | about 0.005% in steady sections | 15.138 ms |

The levels were not matched. JS8Call also contains a **21.438 ms run of exact
digital silence at 2.312 s**, in addition to startup and shutdown silence. That
interruption explains the high whole-recording phase/ripple figures for JS8Call;
its surrounding steady-tone sections are clean. It does not explain the reported
scratchiness of the internal tone, whose outgoing waveform is continuous.

Inspection found a separate startup race: `RadioClient.set_ptt()` immediately
sets optimistic local TX state after the TCP write, and the sender was released
on that local action. The firmware's ingest at `0x0806C80C` discards packets until
radio-state offset `0xAF` equals 1. Thus a delayed radio TX transition can discard
the entire priming burst without producing a host send error or a defect in the
recorded host waveform.

Replaying the internal capture's timestamps with a 32-frame DSP consumer, using
the steady send rate fitted from that capture:

- If every datagram is accepted: **zero** post-priming frame corrections.
- If the first three datagrams are discarded: **32** post-priming corrections.
- If the first six are discarded: **755** post-priming corrections, continuing
  until about **6.03 s** into the send timeline.

This demonstrates a failure consistent with the captures, not proof that the
live radio discarded those packets. The old recordings have no confirmed-PTT
timing or radio-side reception counters.

The host now explicitly queries status and waits for reported TX before releasing
the SDR sender; only then does the existing drain/prime sequence begin. GUI and
rigctl paths both use this handshake, and an unconfirmed start times out rather
than sending the priming burst. Regression checks cover a stale TX report,
intervening RX reports, malformed status and timeout. An integration test with
a simulated 150 ms TX transition accepted every generated datagram after the
change. This host correction uses the existing gain-fixed firmware image.

New captures include `.iq.tx.json` startup/source/counter metadata, and the
analyzer explicitly reports interior digital-silence runs. A new internal-tone
recording with that metadata is needed to determine whether confirmed startup
resolves the live scratchiness or whether packet delivery/radio playback still
needs investigation.

### Confirmed-start capture and actual firmware replay

The next live capture, `/tmp/q900-confirmed`, remained scratchy according to the
operator. Its metadata confirms:

- 2,181 datagrams of 368 frames: 16.72 seconds of samples.
- TX status confirmed in 26.138 ms; first send 76.632 ms after sender release.
- Configured radio media rate 1000.01984 packets/s (48000.952 frames/s).
- Zero capture overflows, skips, drops, trims, send errors and clipping counters.
- Outgoing tone -22.94 dBFS, about 0.013% envelope ripple and 0.000226 rad RMS
  phase residual; largest send gap 11.322 ms.

The handshake worked in this recording, and the outgoing samples remain clean.
To check more than a host-side ring-depth model, `debug/replay_sdr_tx.py` now
executes the actual gain-fixed firmware functions in a Cortex-M7 emulator:

| Function | Address |
| --- | --- |
| Network ingest, including drop/duplicate correction and ring write | `0x0806C80C` |
| Ring depth | `0x0806C7DC` |
| 32-frame DSP reader | `0x0806C954` |
| Corrected raw-I/Q conversion loop | `0x08039EE2` |
| Shared output scaling and conversion | `0x08039C38` |

With all packets accepted at their recorded host send times, the replay reports
**zero post-priming corrections, zero empty DSP blocks, zero ring overflows**,
and a depth range of 2790..4006 words. The codec output remains as clean as the
host stream: 0.01323% envelope ripple and 0.000226 rad RMS phase residual.

This replay assumes an initially empty/aligned ring, the configured sample
clock, unity runtime power/IQ gains and zero phase correction. It does not
simulate interrupt deadlines, DMA/cache/peripheral behavior, network delivery,
the analog/RF chain or the receiving radio. In particular, host send timestamps
are not radio receipt timestamps.

A sensitivity check illustrates why the host recording alone cannot rule out
network/radio ingress loss. Artificially dropping packets 200, 201 and 202 from
this same replay causes **367 subsequent frame corrections** and phase jumps
up to **3.137 rad**, while the envelope remains almost perfectly constant and
the ring never underflows. These are injected losses, not observed live losses.

Reproduce with numpy and the optional `unicorn` package available:

```bash
python3 -B debug/replay_sdr_tx.py /tmp/q900-confirmed
python3 -B debug/replay_sdr_tx.py /tmp/q900-confirmed --drop-packet 200 --drop-packet 201 --drop-packet 202
```

The remaining fault has not been identified. The next missing observation is
audio from the receiving radio, together with its signal level and RF coupling
setup. That can distinguish phase-discontinuity clicks from receiver/audio
overload or a noisy RF tone; another clean host recording cannot do so.

## Reproduce the disassembly

With GNU Arm binutils installed:

```bash
arm-none-eabi-objdump -D -b binary -m arm -M force-thumb --adjust-vma=0x08020000 --start-address=0x0806c954 --stop-address=0x0806c9dc debug/q900_fw.bin
arm-none-eabi-objdump -D -b binary -m arm -M force-thumb --adjust-vma=0x08020000 --start-address=0x080397bc --stop-address=0x08039898 debug/q900_fw.bin
arm-none-eabi-objdump -D -b binary -m arm -M force-thumb --adjust-vma=0x08020000 --start-address=0x08039e34 --stop-address=0x0803a16c debug/q900_fw.bin
arm-none-eabi-objdump -D -b binary -m arm -M force-thumb --adjust-vma=0x08020000 --start-address=0x08039c38 --stop-address=0x08039d0c debug/q900_fw.bin
```

## Capture the live symptom

Record separate JS8Call Tune transmissions at the original and reduced slider
settings. Use distinct prefixes or move a capture before the next transmission:
the sender opens its recording files anew on every key-up.

```bash
Q900_TX_RECORD=/tmp/js8-sdr-high python3 q900_control.py
# One JS8Call Tune transmission at the original level, then close the app.
python3 q900_control.py --analyze-iq-tx /tmp/js8-sdr-high

Q900_TX_RECORD=/tmp/js8-sdr-low python3 q900_control.py
# One transmission at the improved lower level, then close the app.
python3 q900_control.py --analyze-iq-tx /tmp/js8-sdr-low
```

The analyzer reports outgoing envelope peak/RMS dBFS, component counts, envelope
ripple, phase residual and packet timing. These measurements, the running
firmware version, and the second receiver's observations distinguish a clean
host stream from distortion introduced later in the radio or receiving setup.
