# Q900 Control

Standalone PyQt6 control application for the Q900 radio. It provides a local
front panel, Q900 CAT control over inbound TCP or USB serial, receive/transmit
audio routing, and a local Hamlib `rigctl` relay for other applications.

## Requirements

- Python 3.11 or newer
- A Q900 radio reachable over the local network or USB CAT serial
- For audio: operating-system audio devices supported by `sounddevice`

Install dependencies:

```bash
python3 -m pip install -r requirements.txt
```

Run:

```bash
python3 q900_control.py
```

Run the built-in protocol checks:

```bash
python3 q900_control.py --self-test
```

## Receive Audio And Bluetooth

`Start Audio` / `Stop Audio` controls whether this application consumes the
radio's media stream. Connecting starts it automatically, but stopping it is
final: it is not restarted until you ask for it, or until the transport is
reconnected.

Stopping network audio closes UDP/8000. Network PTT, SDR receive and the radio
clock measurement all need that socket, so they are unavailable until audio is
started again; the status line says so when you stop it.

The radio's own audio routing is a front-panel menu (`AUDIO_SOURCE` under
`SYS_SET`, alongside `BT_SET`) and no CAT command for it exists in the supported
set, so this application cannot switch the radio to Bluetooth for you. Releasing
UDP/8000 is what it can do: it stops consuming the network stream and frees the
port for another application. Note that the radio's `BT-MUSIC` entry is described
on the radio as "Open your phone music", which suggests Bluetooth there is for
playing a phone's audio through the radio rather than sending receive audio to a
headset.

## Diagnosing Transmit Audio

To find out whether a defect heard on the air originates in this application,
record exactly what leaves the socket and analyse it:

```bash
Q900_TX_RECORD=/tmp/q900 python3 q900_control.py
# transmit, then close the app
python3 q900_control.py --analyze-tx /tmp/q900
```

The sender writes every transmitted payload to `/tmp/q900.tx.raw` (48 kHz stereo
S16LE, playable directly) and one nanosecond send timestamp per packet to
`/tmp/q900.tx.time`. The analyser reports framing, inserted silence, sample-step
discontinuities, phase discontinuities, and send pacing, with the repetition rate
and buffer-boundary alignment of any events it finds.

The phase test is the sensitive one for FT8 and other tone-based modes: a lost or
repeated sample breaks phase without necessarily producing a large sample step,
and reaches the air as a broadband click. If the analyser reports a clean stream,
nothing above the socket is responsible and the cause is radio-side or
network-side.

Recording is off unless `Q900_TX_RECORD` is set.

### Radio Clock Offset

The network audio row shows the radio's measured media rate, for example
`radio 999.40 pkt/s = 47971 Hz (-600 ppm) over 40s`. The implied sample rate is
shown alongside so a wrong cadence assumption is distinguishable from a genuine
crystal offset: a figure near 48000 Hz confirms that a 192-byte payload is one
millisecond of 48 kHz stereo.

The rate is measured over the current run of arrivals, not over the whole
session. Averaging across a pause makes the figure climb towards the truth
indefinitely without settling, and the stream does pause: it stops while
transmitting and does not begin when the socket opens.

A late read is counted rather than discarded. It shifts an endpoint but does not
change the packet total, so packets over span stays unbiased as the window grows,
whereas restarting on every late read never accumulates a usable window. Only a
pause longer than 50 ms starts a new run, because the radio genuinely stops
producing audio during one.

If the radio sends its media in groups rather than evenly, both endpoints are
aligned to group boundaries. An endpoint falling mid-group understates the span
by up to one group, which for a 32 ms group in a 40 s window is an 800 ppm error.

`breaks` counts pauses, `stalls` counts late reads or group boundaries, and
`over Ns` is the window the figure came from: precision improves with window
length, so treat a short window with suspicion. Nothing is reported until five
thousand packets have accumulated.

### Arrival Pattern

If the radio clock figure looks unstable, record the arrival pattern:

```bash
Q900_RX_RECORD=/tmp/q900 python3 q900_control.py
python3 q900_control.py --analyze-rx /tmp/q900
```

This reports the inter-arrival distribution and distinguishes a radio that sends
in groups from a host that is not reading the socket in time. The two need
opposite fixes, and the timestamps tell them apart: a starved reader leaves an
otherwise evenly paced stream interrupted by stalls, whereas a grouping radio
produces almost no evenly paced intervals at all.

The Q900 is a grouping radio, and this is by design rather than a fault. Its DSP
runs 32-sample blocks at 48 kHz, so 1500 blocks per second, each pushing 64
words into a ring from which the sender emits one 96-word packet whenever 96
words are available. Two packets therefore leave for every three blocks, and the
gaps alternate between roughly 0.667 ms and 1.333 ms while averaging exactly
1.000 ms. A rate estimator that treats that alternation as jitter rather than
structure will misread the clock.

This matters because nothing rate-matches the two ends. The radio runs UHSDR on
an STM32H7 whose codec is clocked by the radio's own crystal, and its firmware
does not resample. Transmit audio is paced from the radio's measured clock when
one is available, and the host stream is rate-converted to it; without a
measurement it falls back to a nominal 1000 packets per second. Any residual
difference accumulates in the radio's transmit ring until it leaves the range
its corrector tolerates, after which the firmware duplicates or drops a frame on
every datagram until the depth returns.


A reading near `+0 ppm` rules that mechanism out. A reading of hundreds of ppm or
more means transmit audio has to be paced from the radio's clock rather than the
host's, or resampled to it.

## Radio Connection

### Network

1. Select `TCP Listener`.
2. Choose a local speaker destination in `USB RX Audio`.
3. Start the listener. It accepts the radio's inbound CAT connection on TCP
   port `8081` and binds UDP port `8000` for Q900 network audio.
4. Configure the Q900 to connect to this computer's listener address.

The application binds UDP/8000 exclusively. If another program already owns
that port, network audio cannot start.

### USB

1. Select `USB Serial` and choose the Q900 serial device.
2. Connect at `115200` baud.
3. For RX audio, select the Q900 USB audio input and a local output, then use
   `Start Audio`.

The app holds DTR/RTS inactive while opening the serial device to avoid an
unwanted hardware PTT assertion.

## Controls

Frequency entry, spectrum/waterfall tuning, mode selection, VFO selection,
split, PTT, ATU, span, RF/IF gain, squelch, AGC, preamp, speaker volume,
noise reduction/blanker, and CW controls relay Q900 CAT commands.

Numeric control tiles open a slider popup. CAT is sent only after selecting
`OK`.

The following controls are intentionally display-only because their Q900 CAT
mapping has not been confirmed: `REF`, `DISP`, `RIT`, and `XIT`.

`LTIME` maps to Q900 CAT `0x32`; its radio-side unit is not yet confirmed.

The spectrum is polled at roughly 8 Hz while receiving and not at all while
transmitting: the display shows the receive passband, so it is not meaningful on
air, and the request costs radio DSP time and a host repaint that competes with
the microphone callback.

## Audio And PTT

### SDR Receive

The `SDR Off` button beside the radio mode selector enables network I/Q receive.
It is independent of the Q900 CAT operating-mode selector. When enabled, the
app requests the alternate stream and only activates SDR after it observes a
Q900 UDP packet with type `0x68`.

The SDR mode selector currently provides host-side `USB`, `LSB`, `AM` and `NFM`
demodulation. I/Q packets are decoded as 48 kHz interleaved signed PCM16LE
complex frames and processed on a worker thread before being sent through the
normal selected speaker or rigctl virtual-microphone output.

`USB` and `LSB` use a phasing detector built on the same Hilbert transformer as
the transmit encoder: with `H` the Hilbert transform, `I - H{Q}` keeps only
positive baseband frequencies and `I + H{Q}` keeps only negative ones, measured
at better than 60 dB opposite-sideband rejection. The two modes are therefore
genuinely different. A product detector that simply takes the real part of the
baseband would fold both sidebands together, give no rejection at all, and make
the mode selector inert.

The receive offset control retunes within the 48 kHz stream. The `swap_iq` and
`invert_q` fields on the receiver mirror the whole stream about its own DC rather
than about the tuned carrier, so they detune instead of swapping sidebands and
are deliberately not exposed; use the offset control to retune and the mode
selector to choose a sideband.

SDR mode also supports experimental network I/Q transmit: 48 kHz interleaved
complex S16LE on UDP/8000. The Q900's network upconverter consumes these as raw
I/Q directly (the firmware reads the host stream into its digital-I/Q path
regardless of the CAT `0x67` TX-source menu; an earlier attempt to select the
source with the extended command `F2 29 02 04` was removed after firmware
analysis showed that command is inert — the dispatcher ignores any command
above `0x67`). The host encoder performs a true single-sideband Hilbert
transform for USB/LSB, AGC plus 750 µs pre-emphasis for NFM, and bakes in the
radio's I/Q mirror by default (the `Swap I/Q` / `Invert Q` calibration toggles
stack on top). The tuned carrier sits at +12 kHz. Use low power and an external
receiver or dummy load while validating the I/Q orientation and carrier offset.

### GUI PTT

Use `Hold To Talk` with the selected physical microphone.

- USB transport sends microphone audio to the selected Q900 USB TX-output
  device.
- Network transport sends raw Q900 UDP audio to the radio on UDP/8000, where
  the radio performs the modulation.
- SDR network transport sends 192-byte I/Q packets every 1 ms into the
  radio's digital-I/Q TX path.

Network TX uses 48 kHz, stereo, signed 16-bit little-endian PCM. A mono
microphone source is duplicated into interleaved left/right samples. Each
datagram is one native media frame: 48 stereo frames, 96 signed 16-bit words,
192 bytes, sent every 1 ms. That is the same quantum the radio uses for its own
RX audio payload. The sender runs in a separate process and uses macOS absolute
Mach timing to hold the 1 ms cadence.

Transmit audio is emitted at the radio's own measured clock, and the host stream
is rate-converted to it. Three clocks are involved: the host produces audio on
its audio clock, this application paces the send, and the radio consumes on its
crystal. Pacing can match only one of the other two. Sending at the host rate
leaves the radio's ring gaining a millisecond of audio every few seconds, which
it resolves by discarding a frame; sending at the radio rate leaves the host
buffer growing until it is trimmed instead. Either way a whole millisecond of
audio disappears periodically and reaches the air as a broadband click, audible
as a tap and visible as a line across a receiving waterfall. Converting the rate
spreads the difference across every sample.

Conversion uses linear interpolation with a fractional phase carried between
packets, so the output is continuous in both amplitude and phase. Worst-case
spurious content measures better than 50 dB below the wanted signal at the
offsets involved, against a discarded millisecond which is broadband. A ratio of
exactly one is bit-transparent, so a correctly clocked link is unaffected.

The ratio starts from the measured radio clock and is then trimmed by a servo on
buffer depth, which absorbs the host audio clock's own error without needing to
know it. Residual drift at the radio is bounded by the accuracy of the clock
measurement alone.

The sender buffers 80 ms before the transmitter is keyed, spends 20 ms of that
priming the radio's TX ring, and holds the remaining 60 ms as a cushion inside
its own process, topping it up from the capture feeder without blocking. The
cushion has to cover more than one 20 ms microphone callback: at a smaller
setting the buffer bottomed out on every mic period, and a callback arriving a
few milliseconds late became an audible gap. A late scheduler wake is made up
with a bounded catch-up burst rather than by discarding schedule, because
discarding makes the long-run send rate lower than the capture rate. Buffered
capture is capped at 200 ms by trimming the oldest whole packets, so transmit
latency cannot grow without bound.

The PTT line reports packet count, capture overflows (`ovf`), underruns
(`gaps`), packets trimmed at the buffer cap (`trim`), failed sends (`err`),
worst scheduling delay (`late`), and clipped microphone blocks (`clip`).

`ovf` is the one to watch. A PortAudio input overflow means capture samples
were lost before the sender ever saw them. The byte stream stays contiguous, so
this is a splice rather than a gap: no other counter can see it, and it reaches
the air as a broadband click. It is caused by something in the GUI process
holding the interpreter lock longer than one 20 ms microphone block, so the
spectrum and waterfall are drawn with vectorised array math, repaints are
coalesced to 15 Hz, and spectrum polling stops while transmitting.

### Rigctl Virtual Audio

Virtual audio is deliberately inactive until at least one local rigctl client is
connected. When a client connects:

- Receive audio is routed to the output endpoint named `Virtual Desktop Mic`.
- Rigctl PTT reads the input endpoint named `Virtual Desktop Speakers`.

These names follow the virtual-device endpoint convention: audio presented to
other applications as a microphone is written to the device's output side; audio
supplied by other applications through their speaker output is captured from the
device's input side. Both are settings rather than code -- override them with
`Q900_VIRTUAL_RX` and `Q900_VIRTUAL_TX` if your virtual audio setup differs. A
named endpoint that is absent is reported, because "no audio" and "wrong device"
are otherwise indistinguishable.

When the last rigctl client disconnects, rigctl PTT is released and receive audio
returns to the locally selected speaker. GUI PTT and rigctl PTT are mutually
exclusive.

### Receive Audio Routing

While a rigctl client is connected, the combo in the audio row chooses where
receive audio goes:

| setting | destination |
| --- | --- |
| RX: virtual only | the virtual endpoint, so only the decoder hears it |
| RX: speakers only | the selected output, so only you hear it |
| RX: both | both at once |

The control is disabled without a client connected, because there is then
nothing to route to and audio always follows the output selected beside it. The
choice survives a client disconnecting and reconnecting; it is not persisted
across runs, since the application stores no settings.

Changing the destination **re-routes in place and does not restart reception**.
That matters for more than a click: `stop()` closes the media socket and resets
the clock accumulator, the transmit sender process holds that same socket, and
transmit pacing depends on that measurement. Restarting to change a device would
release UDP/8000 where another process can take it, discard the radio clock
figure, and cut a transmission in progress. Devices already playing keep their
stream and their queued audio, so switching one destination does not interrupt
the other.

A request that cannot be met falls back rather than going silent -- losing
receive audio is worse than playing it somewhere other than asked -- and says so
in the status line and the control's tooltip.

`both` is two independent output devices. There is no resampling on the receive
path: each sink caps its queue and drops from the front if its device runs slow,
or counts an underflow if it runs fast, so each drifts against the radio
independently and clicks on its own schedule. The `drops` figure in the audio row
is the total across sinks. **A macOS Multi-Output Device is better for
simultaneous playback**, because Core Audio resamples the slaved device to the
master's clock and only one stream then drifts against the radio; build one in
Audio MIDI Setup and select it as the output.

USB receive plays one device only. It negotiates its sample rate against both the
input and the output, so it cannot fan out, and `both` falls back to the first
destination on that transport.

## Rigctl Relay

The embedded relay listens only on:

```text
127.0.0.1:4532
```

It is never exposed to the LAN. The GUI shows listener state and connected
client count in the audio row.

Use Hamlib's network rig model:

```bash
rigctl -m 2 -r 127.0.0.1:4532 f
```

Supported rigctl operations include frequency, mode, PTT, VFO, split, and
selected levels (`AF`, `RF`, `SQL`, and `MICGAIN`). The relay implements the
Hamlib connection handshake, including `\chk_vfo` and `\dump_state`.

All rigctl operations pass through the same serialized Q900 CAT client used
by the GUI. Unsupported operations return the normal Hamlib unsupported/error
response or a harmless success response where Hamlib expects a capability
probe.

## Shutdown

Close the window normally. `Ctrl+C` is handled through Qt shutdown so PTT,
audio streams, rigctl clients, listeners, and the radio transport are released
cleanly.

## Protocol Notes

Q900 CAT frames use:

```text
A5 A5 A5 A5 | length | command | payload | CRC-16/CCITT-FALSE (big-endian)
```

Network RX/TX audio uses UDP port `8000`. The radio-to-PC packets are Q900
framed: a nine-byte header whose fifth byte is the stream type (`0x67` audio,
`0x68` I/Q) and whose bytes 5..8 are the radio's device ID word, followed by a
192-byte payload. PC-to-radio payloads are raw stereo PCM16LE without an
application header, using the same 192-byte payload size and 1 ms cadence.

The radio will only accept a datagram whose **source port is also 8000** and
whose source address matches its configured `REMOTE_IP`: the firmware calls
`udp_connect` on its media socket, so lwIP drops anything else before the
application sees it. Payload length must be a whole number of stereo frames,
because the word count is derived as `bytes >> 1` and an odd word count
permanently shifts the ring's left/right parity. At most 2560 bytes of a
datagram are staged.

Only the **first word of each stereo frame is used** for audio. The firmware
takes element 0 and discards element 1, so the duplicated right channel exists
only to satisfy the ring consumer's frame geometry.

A larger datagram at a proportionally slower rate is not truncated -- the
receive callback pushes every staged word into the ring -- but it is still the
wrong choice, because the ring's rate corrector runs once per datagram and a
longer datagram therefore buys proportionally less correction authority.

### Transmit Level

The radio does not scale network audio to suit itself. For stream format 1 the
conversion applies exactly `2**-16`, which cancels the ring consumer's `<< 16`
and leaves the DSP working with the raw int16 value: unity. What follows is the
radio's own TX gain chain, a pre-gain of `0.5 + 0.5 * state[0x1A0]` and then a
per-sample ALC whose knee is 30000.

`state[0x1A0]` is selected by CAT `0x10`, the setting labelled COMPRESSOR. It is
not a ratio; it is a pre-ALC gain of up to 13x, looked up from a table in the
firmware.

How hard to drive that chain depends on what the audio is, so the application
picks the level from the transmit source rather than from the transport, and
reports which it chose in the audio status line as `peak N/CMP M/digital` or
`/voice`.

**Digital modes drive full scale**, because that is what the radio's own USB
digital input does. Both transports hand int16 to the same DSP ring at the same
gain, so the only difference between them is the number written there: the USB
input receives the application's samples at their native scale, and after the
pre-gain that saturates the knee for any level above about 11% of full scale, so
USB radiates full power almost regardless of the application's output slider.
Scaling to sit *below* the knee instead makes transmit power track that slider
linearly, and nothing downstream can recover the difference, because the ALC's
gain is clamped to a maximum of 1.0 at `0x0803990E` -- it attenuates and never
amplifies. Every dB below the knee is simply not transmitted. At the level
measured from a real capture, 0.222 of full scale, that was **13 dB of transmit
power discarded**, and at low slider settings up to 19 dB.

Driving a limiter hard costs a constant-envelope mode nothing. With constant
input magnitude the ALC gain converges and then holds, applying a fixed scale
factor and adding no distortion. Its time constant is 0.001 per sample, about
21 ms, and nothing in the firmware resets its gain on PTT, so only the first
transmission after power-on spends any time settling. Measured on the wire, the
full-scale level is 18.8 dB hotter with SNR 0.26 dB *better* and THD improved
from -85 to -101 dB, because the quantiser's noise moves 19 dB further below the
signal. A 3% margin is kept below int16 so that clipping, which splatters far
worse than the power it would buy, cannot happen.

**Speech keeps the linear level**, sitting just under the knee, because it has an
envelope for a limiter to act on and the dynamics should stay with the operator's
COMPRESSOR setting.

An earlier version of this document claimed that driving full scale was itself
the cause of rough transmit audio, via the ALC modulating its gain at audio rate.
That was wrong. The roughness was the capture bursting, the resampler's comb and
the steering loop's wobble, all documented below and all since fixed; the level
reduction was a defence against a misdiagnosis and it cost 13 dB. What the level
is actually worth was then measured, not assumed: FT8 coherent capture into one
6.25 Hz bin is -3.12 dB via USB against -3.13 dB via the network, so spectral
purity was never the difference between the two paths, and power was.

The radio never reports COMPRESSOR back, so the value used is the application's
own record of it. If the radio's compressor has been changed from the front
panel, set it from the host as well or the voice level will be wrong by the ratio
of the two gains.

Because under-driving is otherwise invisible -- a clip count of zero looks
healthy when it can equally mean the signal never came close -- the status line
leads with the drive against the knee as `alc +N dB limiting` or
`alc -N dB UNDER`. Zero or above means the radio is at full output. Below zero is
the number of dB being thrown away.

### Status Lines

The transmit and receive status lines are ordered by diagnostic priority rather
than by convenience, because they are elided to whatever width their row can
spare: the drive figure and anything actually wrong come first, so they are
readable without hovering, and the steady-state volume trails.

The transmit line's seven fault counters collapse to whichever are non-zero, and
read `clean` when none are. Printing `ovf 0  drop 0  skip 0  rep 0  trim 0
err 0  clip 0` spent about forty characters saying that nothing had happened and
pushed the figures that do change out of view. `clean` keeps the absence
explicit, so a quiet line still distinguishes healthy from not-yet-reporting.

Hovering any status label gives the untruncated string.

These labels cannot resize the window, and that is deliberate. A QLabel with word
wrap off reports its full text width as its *minimum* size hint, and the main
window's root layout turns a layout minimum into a window minimum, which Qt then
enlarges the window to satisfy -- and never shrinks again when the minimum falls.
Labels carrying counters therefore widened the window every time a counter gained
a digit, a one-way ratchet that ended up larger than the display; the transmit
label alone moved the window's minimum by 1063 px. They use an `Ignored`
horizontal size policy so their text cannot reach the layout at all, with a
constant minimum width so that a crowded row cannot hide them entirely either.
Do not change that policy back to `Preferred`.

The window is also clamped to the display it opens on, which nothing did before,
and which is what recovers a window that an earlier run left oversized.

A window that has become too wide cannot be clamped back by capping it. **A
layout minimum beats a maximum size**: a window whose layout demands more width
than `setMaximumWidth` allows simply ignores the cap, measured at 13806 px
against a 600 px limit. So there are two distinct faults that look identical, and
only one is recoverable:

- The window is merely too *wide*, with a sane minimum. This is the aftermath of a
  transient, because Qt enlarges a window when a layout minimum rises and never
  shrinks it when the minimum falls again -- one brief spike leaves the window
  permanently oversized. Recoverable.
- The window's layout *minimum* exceeds the display. Nothing can fix this from
  the outside; some widget is insisting on the width and that widget has to
  change.

A once-per-second check handles the first and diagnoses the second, naming the
widgets responsible on stderr with their text, so the culprit does not have to be
guessed at. It decides which case applies by resizing and seeing whether it
sticks, rather than by reading the minimum to predict it: the minimum is cached
and goes stale at exactly the moment a spike clears, and predicting from it
reported an unfixable minimum for a window that only needed shrinking. Clearing
that cache needs `invalidate()`, not just `activate()`, or the resize is clamped
to a figure that is no longer true.

`--self-test-ui` covers this, and is separate from `--self-test` because it
**cannot run offscreen**. The offscreen platform says so itself, "This plugin does
not support propagateSizeHints()", and propagating size hints to the window
manager is the whole mechanism. An earlier version of this check did run
offscreen, passed, and the window kept growing regardless -- so the test now
asserts that a planted spike *does* grow the window before asserting that the
window recovers, because a test that cannot observe the fault is worse than none.

### Transmit Rate Conversion

The host produces audio on its own clock and the radio consumes on its crystal,
so one end has to be converted to the other. Both the conversion filter and the
loop that steers it were putting measurable spurs on transmitted audio, and both
were invisible on the USB transmit path because that path does not convert at
all -- which is why USB sounded clean while the network path did not.

The conversion is a polyphase windowed-sinc bank, 24 taps and 512 phases, cut off
at Nyquist so the zero-phase row is an exact delta and a correctly clocked link
stays bit-identical. It replaced linear interpolation, whose response depends on
the fractional phase: a passthrough at phase 0 and a mild lowpass at phase 0.5.
With the phase walking continuously that difference became spectral modulation at
the wrap rate, `|ratio - 1| * 48000` Hz, measured on a transmitted tone as a
sideband comb 23.66 Hz either side of the carrier. Replacing the filter moved the
worst nearby spur from -61 dB to -91 dB.

The servo that trims the ratio smooths its error with two poles at about two
seconds. Capture arrives in 20 ms blocks, so the buffer depth is a sawtooth one
whole block deep, and a proportional term applied to that converted buffer
granularity directly into rate -- frequency modulation of the audio rather than
rate control. It measured 229 ppm rms of ratio movement in the 2 to 200 Hz band,
worth a -47 dB sideband family a few Hz either side of a tone; smoothing brings
that to 0.01 ppm. The loop's own natural frequency is near 0.006 Hz, so a two
second filter is orders of magnitude faster than anything it needs to do. Slow
drift below 2 Hz is deliberately left alone: that is the servo working, and at
these amplitudes it is inaudible pitch wander rather than roughness.

The servo also aims at the middle of the band the refill loop actually holds,
rather than at the low-water mark itself. Aiming at the mark meant the measured
depth could never fall below the target, so the error never changed sign and the
integrator wound up against its limit.

### Capture Latency And Where Capture Runs

Two things about the microphone stream mattered more than anything in the audio
path itself.

**The latency hint.** Requesting `latency="high"` makes CoreAudio hand over
several blocks back to back and then nothing for 85 ms. Measured over 20 s on a
USB input: 764 of 998 callbacks arrived less than 1 ms apart, and 233 gaps
exceeded 60 ms. Nothing on a 1 ms packet clock absorbs that reliably -- the
cushion was 60 ms and the radio's ring another 32 ms, so an 85 ms gap was only
just covered and any jitter punched through as a hole in the transmitted audio.
Requesting `latency="low"` gives one block every 20.00 ms with a worst case of
20.24 ms, and measures the device clock as -12 ppm instead of an apparent
+2884 ppm. Transmit capture therefore asks for low latency. Playback streams
still ask for high, where a deep buffer is what you want.

**Which process it runs in.** Capture now runs inside the sender process rather
than the GUI. The callback shares a GIL with everything else in its process, and
in the GUI that includes spectrum and waterfall repaints; the project's own notes
already identified that paint cost becomes transmit audio quality. There is also
no longer a queue between capture and pacing, so a block cannot be delayed or
dropped in transit. The GUI passes the device by name, not index, so the two
processes cannot disagree about a renumbered device list, and the transmit meter
reads the level the sender last saw.

With both changes, a twenty second transmission measures zero skips, zero
repeats, zero discarded blocks, no capture overflows, worst send gap 1.7 ms, and
a ring estimate sitting exactly on its 32 ms target.

### Transmit Underruns

When the sender has no audio for a scheduled slot it has two ways to fail, and
which one is right depends on how deep the radio's ring is at that moment.

Skipping the slot is inaudible while the ring has slack, because the ring holds
about 32 ms for exactly this purpose and covers the gap. But skipping also spends
that slack, and once the ring is empty every later hiccup becomes a hole on the
air. Substituting a packet of digital silence, which is what this used to do, is
worse still: it guarantees a hole immediately.

So the sender keeps a running estimate of the ring depth, skips only while that
estimate says the ring can afford it, and repeats the previous packet once it
cannot. A repeat is a millisecond of duplicated audio -- a small click -- but it
holds the ring at depth and keeps the schedule exact, so a shortfall cannot
cascade into a run of holes. The estimate is exact in slot units: one scheduled
slot is one packet's worth of consumption by construction, because the slot
period is `1/radio_rate`.

Modelling the firmware ring from real send timestamps: a 12 ms feeder stall
produces no holes at all, and a pathological 120 ms stall repeated every 3.5 s
puts 4 ms of holes on the air against a 1.5 s shortfall. Sending silence for the
same shortfall would have put all of it on the air.

The audio status line reports `skip` (slots the ring covered), `rep` (slots
covered by repeating), `drop` (whole microphone blocks discarded by a full feeder
queue) and `ring` (the depth estimate in ms). A healthy transmission sits in the
16 to 48 ms window with few skips and no repeats.

### Datagram Geometry

`Q900_TX_FRAMES` sets how many stereo frames go in a datagram, clamped to
48..640, which is 1 to 13.3 ms at 48 kHz.

The default is **640 frames, 2560 bytes, 75 packets a second**. It was chosen on
the air: recording a second radio while transmitting a tone, going from 48 frames
to 192 narrowed the skirt around the carrier by 8 to 9 dB and halved the
frequency wander, and 640 was better again. 640 is also exactly what the
firmware's receive callback will stage -- one byte more and it truncates the
datagram and discards the remainder.

Two mechanisms both predict that improvement and neither has been separated from
the other. The radio takes an Ethernet interrupt and runs lwIP for every
datagram, on the same Cortex-M7 that must meet a 666 us DSP block deadline; and
the ring's rate corrector engages at most once per datagram. Both scale with the
packet rate.

The trade is real and worth knowing:

| frames | bytes | packets/s | Ethernet frames/s | fragmented |
| --- | --- | --- | --- | --- |
| 48 | 192 | 1000 | 1000 | no |
| 320 | 1280 | 150 | 150 | no |
| 368 | 1472 | 130 | 130 | no, largest that is not |
| 640 | 2560 | 75 | 150 | yes, two fragments |

2560 bytes does not fit an Ethernet frame, so every datagram is sent as two IP
fragments. That makes 640 no better than 320 for interrupt load, adds a
reassembly in the radio for each packet, and loses the whole 13.3 ms datagram if
either fragment is dropped. It is the default because it is what has been
measured to work and because it gives the corrector 42 per cent fewer
opportunities than the MTU-safe size. **If the mechanism turns out to be
interrupt load rather than the corrector, 368 frames is the better choice.** The
audio status line says so when the configured size will fragment.

Large datagrams also make the ring coarse. At 640 frames one datagram is 13.3 ms
of a 32 ms corrector window, so priming settles at 30 ms with only about one
datagram of margin before the duplication threshold. The ring-aware underrun
logic protects that boundary, but a datagram lost in the network is invisible to
this side and costs the whole 13.3 ms.

Everything derived from the geometry is derived rather than written down: the slot
period, the priming burst, where priming leaves the ring, the debt bound and the
burst spacing floor. Verified on the wire at 48, 96, 192, 320, 368 and 640 frames:
THD -83 dB, all energy within +-2 Hz of the tone, skirt below -90 dB, no skips,
no repeats, and a long-run send rate within a few tens of ppm of the radio's
clock.

### Correcting The Measured Clock

`Q900_TX_PPM` shifts the send rate against the measured radio clock, bounded at
+-2000 ppm. Positive sends faster. It applies to the conversion ratio as well as
the period, because the period governs the radio's ring and the ratio governs the
host's buffer; correcting one without the other fixes one and breaks the other.

The use for it is that the corrector's window is only 32 ms wide. Priming puts
the ring in the middle, after which it drifts at whatever the error in the
measured clock is, and 32 ms divided by that error is how long a transmission
stays clean. So if transmit audio is clean for a while and then turns rough, the
time it took is a measurement: the error is roughly `16000/seconds` ppm, and this
cancels it.

### Transmitting A Known Tone

`Q900_TX_TONE` synthesises a sine inside the sender instead of reading the
microphone. Everything downstream is identical -- the same DC blocker, quantiser,
resampler, pacing and socket -- so anything a recording shows that is not present
in an exact sine belongs to this application or the radio.

```bash
Q900_TX_TONE=1500 python3 q900_control.py
```

This exists because a tone driven in through a virtual audio device cannot serve
as a reference. The source application, the virtual device and CoreAudio may each
resample it, and all of that sits upstream of anything here; a skirt measured on
the air could belong to any of them. Measuring a transmit path needs a source
known to be clean.

Measured on the wire with a 1500 Hz synthesised tone, 21 s, ceiling 3637:

| | measured | an exact int16 sine at the same ceiling |
| --- | --- | --- |
| THD | -83.1 dB | -76.6 dB |
| SNR | +74.2 dB | +87.4 dB |
| energy within +-2 Hz | 100.000 % | 100.000 % |
| skirt 3-10 Hz | -101.0 dB | -106.2 dB |
| worst spur within 500 Hz | -112.5 dB | -117.1 dB |

Distortion is *lower* than the naive ideal because rate conversion decorrelates
the quantiser's error, spreading it as noise rather than leaving it in harmonics;
that is why the SNR column is worse while the THD column is better. The total
error power is much the same and noise is the more benign of the two.

The practical use is attribution. If a tone sounds raspy from the microphone path
but clean with `Q900_TX_TONE`, the defect is in the source chain and nothing in
this application will change it.

### Diagnosing Transmit Audio

`--analyze-tx` reads a `Q900_TX_RECORD` capture, which is byte-for-byte what left
the socket, and so separates a host defect from a radio or network one. Drive it
with a steady tone -- WSJT-X `Tune` will do -- because a single sine makes every
defect measurable in a way speech cannot:

```bash
Q900_TX_RECORD=/tmp/q900 python3 q900_control.py
python3 q900_control.py --analyze-tx /tmp/q900
```

Each defect this path can produce has its own signature, so the numbers identify
which one rather than merely reporting that something is wrong:

| defect | THD | SNR | envelope ripple |
| --- | --- | --- | --- |
| clean | -76 dB | +85 dB | 0.04 % |
| a frame dropped per datagram (ring above 4608 words) | -34 dB | +25 dB | 16 % |
| a frame duplicated per datagram (ring below 1536 words) | -34 dB | +25 dB | 16 % |
| audio being transmitted as I/Q | -87 dB | 0 dB | 157 % |
| hard clipping, 6 dB into a clipper | -15 dB | +42 dB | 30 % |
| host underrun silence | -76 dB | +65 dB | 0.4 % |

Rate-conversion residue does not show up in any of those columns, because it is
neither harmonic nor amplitude modulation. It appears as a spur close to the
carrier, so the report names the worst one within 500 Hz and converts its offset
to ppm: a comb at that spacing is the conversion ratio made audible.

Measure only the steady part of a tone. A Tune transmission ramps up and down on
purpose, and a ramp inside the analysis window is a real amplitude modulation
that will report a tone flat to half a per cent as a hundred per cent modulated.

The clipping row is a hard clipper, which is what int16 saturation in the host
looks like and the reason a margin is kept below full scale. It is *not* what the
radio's ALC does: that is a gain-controlled limiter with a 21 ms time constant, so
against a constant-envelope signal it settles to a fixed scale factor and leaves
no fingerprint at all. Do not read this row as a reason to keep the drive down.


### Transmit Ring

The radio's transmit ring holds 6144 words, 64 ms at 48 kHz stereo. Its rate
corrector leaves 1536..4608 words (16..48 ms) alone; below that it duplicates a
frame on every datagram and above it drops one, a thousand times a second in
either direction. That is what roughness on this path sounds like.

Two properties of the firmware make this harder than it looks. The ring is
consumed **only while PTT is asserted** -- `0x0803432C` tests `state[0xAF]` and
runs either the receive path or the transmit path, never both -- and **nothing
ever resets its indices**. Whatever depth was left at the previous unkey is still
sitting there at the next key-up. Priming on top of it therefore accumulates:
within a few transmissions the depth passes 4608 words and the firmware starts
dropping a frame from every datagram, and past 6143 it overflows. An overflow
advances the read index by a single word, which permanently breaks its 64-word
alignment, and once misaligned `peek()` straddles the end of the ring -- it has
no wrap handling -- and reads out of bounds into the receive media ring.

There is no way to flush it from the host. Datagrams sent while unkeyed are
discarded by the PTT gate, no CAT command reports or clears the ring, and the
firmware's own depth and underflow counters are written but never read by
anything. The only mechanism is the consumer itself, so the sender keys the
transmitter and then deliberately sends nothing for slightly longer than a full
ring takes to drain, and only then primes. The starting depth is then
deterministic regardless of how the previous transmission ended.

Priming aims for the centre of the corrector's window. The consumer keeps running
while the burst is paced out, so a primed packet nets less than its whole 96
words, and the packet count is derived from the target depth rather than written
down. Bursts -- priming, catch-up after a late wake, and debt repayment -- are all
floored to a minimum spacing, because bursting at line rate asks the radio's
Ethernet and lwIP receive path to absorb a thousand times its steady-state packet
rate, and a datagram lost there is a millisecond of audio missing with nothing to
resend it. Schedule debt after a late wake is repaid rather than discarded, since
discarding it lowers the long-run send rate below the radio's consume rate and
walks the ring down into the duplication region with nothing able to observe it.


