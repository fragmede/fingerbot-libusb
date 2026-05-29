# fingerbot-libusb

Raw USB/libusb control for the GL.iNet GL-FGB-01 Fingerbot dongle from macOS.

The GL-FGB-01 USB receiver enumerates as a Bluetooth HCI controller. GL.iNet's
KVM firmware normally drives it through BlueZ and `/usr/sbin/fingerbot`; this
repo talks to the receiver directly with PyUSB/libusb and sends the minimal HCI,
L2CAP, and ATT traffic needed to control the paired Fingerbot.

## Hardware

- GL.iNet GL-FGB-01 Fingerbot USB dongle
- USB VID:PID `2fe3:000b`
- Tested on macOS with Homebrew libusb
- Tested with a paired GL-FGB-01 Fingerbot

The paired address is read from the dongle with vendor HCI command `0xfc33`, so
you usually do not need to pass an address manually.

## Install

Install Homebrew libusb and uv:

```bash
brew install libusb uv
```

Create the local environment:

```bash
uv sync
```

If your uv config is broken or you want to ignore global uv settings:

```bash
uv --no-config sync
```

## Read-Only Checks

Print USB descriptors and the dongle-paired Fingerbot address:

```bash
uv run fingerbot-libusb probe
```

Read the current Fingerbot state:

```bash
uv run fingerbot-libusb read --reset-controller
```

Expected output includes the paired address, an advertisement from the bot, a
connection line, selected characteristic `value=0x0012`, and state fields such
as `push_ms`, `hold_ms`, and `pull_ms`.

## Press The Button

Actuation commands require `--yes-press`. The tool refuses to move the Fingerbot
without it.

Normal click:

```bash
uv run fingerbot-libusb click --yes-press --reset-controller
```

Hard click:

```bash
uv run fingerbot-libusb click --yes-press --reset-controller --strength hard
```

Hard click held for 2 seconds:

```bash
uv run fingerbot-libusb click --yes-press --reset-controller --strength hard --hold-ms 2000
```

The timing controls are:

- `--push-ms`: arm extension travel time
- `--hold-ms`: time held down after extension
- `--pull-ms`: arm retraction travel time

Use one press at a time when the bot is attached to a computer power button.

## Recovery

The dongle can occasionally get into a transient HCI state after failed
experiments. Software reset usually recovers it:

```bash
uv run fingerbot-libusb usb-reset
```

Then retry the read or click command. If software reset repeatedly fails,
physically unplug and replug the USB dongle.

## Protocol Notes

The useful vendor details from GL.iNet firmware are:

- Read paired address: `hcitool cmd 0x3f 0x0033`
- Pair address: `hcitool cmd 0x3f 0x0034 <mac bytes>`
- BlueZ path: `/org/bluez/$DEVICE/dev_${MAC_PATH}/service0010/char0011`
- ATT value handle discovered from that path: `0x0012`
- Set-action payload:

```text
03 00 <push_ms little-endian u16> <hold_ms little-endian u16> <pull_ms little-endian u16>
```

The driver performs LE Read Remote Used Features (`0x2016`) after connecting.
Without that exchange, ATT traffic is unreliable with this receiver/peripheral
pair. ATT MTU exchange is skipped by default because this Fingerbot ignores it
and the command payloads fit in the default 23-byte MTU.

## Skill

This repo includes a project-local Codex skill at:

```text
.codex/skills/fingerbot-control/SKILL.md
```

Use `$fingerbot-control` in a Codex session inside this repo to load the
operational workflow and safety notes.
