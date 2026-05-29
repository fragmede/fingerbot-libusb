---
name: fingerbot-control
description: Control a GL.iNet GL-FGB-01 Fingerbot USB dongle from macOS using the raw libusb HCI driver in this repo. Use when Codex needs to probe the FGB01-Dongle, read Fingerbot state, press or hold a target laptop power button, tune hard/light press timing, or recover the dongle after HCI timeouts.
---

# Fingerbot Control

Use the repo driver, not BlueZ/CoreBluetooth:

```bash
uv run fingerbot-libusb ...
```

The working dongle is USB `2fe3:000b` (`FGB01-Dongle`). The paired bot address
is read from the dongle at runtime; do not hard-code or publish a user's real
Bluetooth address. The vendor BlueZ path `service0010/char0011` maps to ATT
value handle `0x0012`.

## Environment

Install libusb and uv if needed:

```bash
brew install libusb uv
uv sync
```

If uv has a bad global config, use:

```bash
uv --no-config sync
```

The driver checks `LIBUSB_DYLIB` first, then the Homebrew Apple Silicon path
`/opt/homebrew/opt/libusb/lib/libusb-1.0.dylib`, then PyUSB's default libusb
discovery.

## Safety

Treat `click`, `push`, and `pull` as physical actions. Only send them when the
user explicitly requests a press/actuation. Send one press unless the user asks
for repeated attempts. Keep `--yes-press`; the driver refuses actuator commands
without it.

For read-only checks, use `probe`, `scan`, `discover`, or `read`.

## Workflow

1. Confirm the link is healthy:

   ```bash
   uv run fingerbot-libusb read --reset-controller
   ```

   Expected signs: paired address printed, advertisement found, connected,
   selected characteristic `value=0x0012`, and state fields printed.

2. If the user requested a normal click:

   ```bash
   uv run fingerbot-libusb click --yes-press --reset-controller
   ```

3. If the user requested a hard click:

   ```bash
   uv run fingerbot-libusb click --yes-press --reset-controller --strength hard
   ```

4. If the user requested a held press, set the hold duration in milliseconds:

   ```bash
   uv run fingerbot-libusb click --yes-press --reset-controller --strength hard --hold-ms 2000
   ```

   `--push-ms` and `--pull-ms` tune travel timing. Use them only when the
   default light/hard timings are insufficient.

5. Verify completion from command output. `click command sent` proves the GATT
   write succeeded. User visual confirmation, camera evidence, or target-device
   state is stronger proof that the physical button moved or booted.

## Recovery

The dongle can get into transient HCI states after failed attempts. If the
vendor paired-address read, LE remote-feature exchange, or disconnect times
out, run:

```bash
uv run fingerbot-libusb usb-reset
```

Then retry the read or click command. If software reset repeatedly fails and
the user is away but has allowed attention alerts, use macOS `say` to ask for a
physical USB replug.

## Protocol Notes

The raw driver must run LE Read Remote Used Features (`0x2016`) after the LE
connection before ATT traffic is reliable. ATT MTU exchange is skipped by
default because this Fingerbot ignores it and the control payloads fit in the
default 23-byte MTU.

Known GLKVM payload for click setup:

```text
03 00 <push_ms le16> <hold_ms le16> <pull_ms le16>
```
