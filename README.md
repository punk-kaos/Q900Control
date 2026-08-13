# Q900 Control

Standalone PyQt6 control console for the Q900 radio. It supports the radio's
inbound TCP control connection on port 8081 and direct USB CAT serial control
at 115200 baud.

When connected by USB, select the Q900 USB receive-audio device and a local
speaker destination in the `USB RX Audio` bar, then select `Start Audio`.

Install the single UI dependency:

```bash
python3 -m pip install -r requirements.txt
```

Run the application directly:

```bash
python3 q900_control.py
```
