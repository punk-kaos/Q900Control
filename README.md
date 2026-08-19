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

Virtual audio is deliberately inactive until at least one local rigctl client
is connected. When a client connects, it takes ownership of receive audio:

- Q900 receive audio is routed to the output endpoint named
  `Virtual Desktop Mic`.
- Rigctl PTT reads the input endpoint named `Virtual Desktop Speakers` and
  routes it to Q900 transmit audio.

These names follow the virtual-device endpoint convention: audio presented to
other applications as a microphone is written to the device's output side;
audio supplied by other applications through their speaker output is captured
from the device's input side.

When the last rigctl client disconnects, rigctl PTT is released, virtual audio
stops, and normal receive audio returns to the locally selected speaker.
GUI PTT and rigctl PTT are mutually exclusive.

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
firmware. That chain is calibrated for the codec's microphone input, which peaks
well below full scale. Sending int16 full scale instead drove the default
setting (`9`, a gain of 8.00x) to 262136 against a 30000 knee -- 18.8 dB into
the limiter -- and the ALC then held roughly 19 dB of gain reduction and
modulated it at audio rate. That was audible as rough transmit audio with a
pumping background, from the first moment of transmission rather than
progressively.

The application therefore derives its peak from the COMPRESSOR setting so that
`peak * pregain` lands just under the knee, and reports it in the audio status
line as `peak N/CMP M`. The radio never reports this setting back, so the value
used is the application's own record of it: if the radio's compressor has been
changed from the front panel, set it from the host as well or the level will be
wrong by the ratio of the two gains.

### Transmit Ring

The radio's transmit ring holds 6144 words, 64 ms at 48 kHz stereo. Its rate
corrector leaves 1536..4608 words (16..48 ms) alone; below that it duplicates a
frame on every datagram and above it drops one, a thousand times a second in
either direction. Transmit priming therefore aims for the centre of that window,
and schedule debt after a late wake is repaid rather than discarded, because
nothing on this path can observe the ring depth: no CAT command reports it, and
the firmware's own depth and underflow counters are written but never read.

