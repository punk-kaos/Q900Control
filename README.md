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

The sender buffers 40 ms before the transmitter is keyed and then holds a 20 ms
cushion inside its own process, topping it up from the capture feeder without
blocking. A late scheduler wake is made up with a bounded catch-up burst rather
than by discarding schedule, because discarding makes the long-run send rate
lower than the capture rate. Buffered capture is capped at 200 ms by trimming
the oldest whole packets, so transmit latency cannot grow without bound.

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
`0x68` I/Q), followed by a 192-byte payload. PC-to-radio payloads are raw
stereo PCM16LE without an application header, but use the same 192-byte
payload size and 1 ms cadence. Sending a larger datagram at a
proportionally slower rate delivers the correct aggregate byte rate and is
still wrong: the firmware consumes only the first media frame of each
datagram and discards the remainder.
