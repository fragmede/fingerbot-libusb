#!/usr/bin/env python3
"""Raw libusb driver for the GL.iNet GL-FGB-01 Fingerbot dongle.

The FGB01-Dongle enumerates as a USB Bluetooth HCI controller.  GL.iNet's
GLKVM firmware uses BlueZ for the BLE work, but the protocol is small enough
to drive directly:

* HCI vendor command 0xfc33 returns the Fingerbot address paired to the dongle.
* BLE ATT traffic uses fixed L2CAP CID 0x0004 after an LE connection.
* The Fingerbot control characteristic receives GLKVM's set-action payload:
  03 00 <push_ms le16> <hold_ms le16> <pull_ms le16>
* The vendor BlueZ path is service0010/char0011; on the wire the
  characteristic value handle discovered from that declaration is 0x0012.

Actuating commands require --yes-press so read-only probing cannot
accidentally press the attached laptop power button.
"""

from __future__ import annotations

import argparse
import contextlib
import dataclasses
import os
import struct
import sys
import time
from collections.abc import Iterable

import usb.core
import usb.util
from usb.backend import libusb1


VENDOR_ID = 0x2FE3
PRODUCT_ID = 0x000B
INTERRUPT_IN_EP = 0x81
BULK_OUT_EP = 0x01
BULK_IN_EP = 0x82
DEFAULT_LIBUSB_DYLIB = "/opt/homebrew/opt/libusb/lib/libusb-1.0.dylib"
LIBUSB_DYLIB = os.environ.get("LIBUSB_DYLIB", DEFAULT_LIBUSB_DYLIB)

HCI_VENDOR_READ_PAIRED_ADDR = 0xFC33  # hcitool cmd 0x3f 0x0033
HCI_SET_EVENT_MASK = 0x0C01
HCI_RESET = 0x0C03
HCI_DISCONNECT = 0x0406
HCI_LE_SET_EVENT_MASK = 0x2001
HCI_LE_SET_SCAN_PARAMETERS = 0x200B
HCI_LE_SET_SCAN_ENABLE = 0x200C
HCI_LE_CREATE_CONNECTION = 0x200D
HCI_LE_CREATE_CONNECTION_CANCEL = 0x200E
HCI_LE_READ_REMOTE_USED_FEATURES = 0x2016

EV_DISCONNECTION_COMPLETE = 0x05
EV_COMMAND_COMPLETE = 0x0E
EV_COMMAND_STATUS = 0x0F
EV_LE_META = 0x3E

SUBEV_LE_CONNECTION_COMPLETE = 0x01
SUBEV_LE_ADVERTISING_REPORT = 0x02
SUBEV_LE_READ_REMOTE_USED_FEATURES_COMPLETE = 0x04
SUBEV_LE_ENHANCED_CONNECTION_COMPLETE = 0x0A

L2CAP_CID_ATT = 0x0004
L2CAP_CID_LE_SIGNALING = 0x0005
ATT_ERROR_RESPONSE = 0x01
ATT_EXCHANGE_MTU_REQ = 0x02
ATT_EXCHANGE_MTU_RSP = 0x03
ATT_READ_BY_TYPE_REQ = 0x08
ATT_READ_BY_TYPE_RSP = 0x09
ATT_READ_REQ = 0x0A
ATT_READ_RSP = 0x0B
ATT_READ_BY_GROUP_TYPE_REQ = 0x10
ATT_READ_BY_GROUP_TYPE_RSP = 0x11
ATT_WRITE_REQ = 0x12
ATT_WRITE_RSP = 0x13
ATT_WRITE_CMD = 0x52

ATT_ERR_ATTR_NOT_FOUND = 0x0A

UUID_PRIMARY_SERVICE = 0x2800
UUID_CHARACTERISTIC = 0x2803

CHAR_PROP_READ = 0x02
CHAR_PROP_WRITE_NO_RSP = 0x04
CHAR_PROP_WRITE = 0x08

LIGHT_PUSH_MS = 800
LIGHT_PULL_MS = 803
HARD_PUSH_MS = 1000
HARD_PULL_MS = 1003


class HciError(RuntimeError):
    def __init__(self, message: str, status: int | None = None) -> None:
        super().__init__(message)
        self.status = status


class AttError(RuntimeError):
    def __init__(self, request_opcode: int, handle: int, code: int) -> None:
        super().__init__(
            f"ATT error 0x{code:02x} for request 0x{request_opcode:02x} "
            f"at handle 0x{handle:04x}"
        )
        self.request_opcode = request_opcode
        self.handle = handle
        self.code = code


@dataclasses.dataclass(frozen=True)
class Advertisement:
    address: str
    address_type: int
    event_type: int
    rssi: int
    data: bytes


@dataclasses.dataclass(frozen=True)
class Service:
    start_handle: int
    end_handle: int
    uuid: str


@dataclasses.dataclass(frozen=True)
class Characteristic:
    declaration_handle: int
    value_handle: int
    properties: int
    uuid: str
    service: Service


def make_backend() -> object:
    if LIBUSB_DYLIB and os.path.exists(LIBUSB_DYLIB):
        return libusb1.get_backend(find_library=lambda _: LIBUSB_DYLIB)
    return libusb1.get_backend()


def find_dongle() -> usb.core.Device:
    dev = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID, backend=make_backend())
    if dev is None:
        raise RuntimeError("FGB01-Dongle not found at USB VID:PID 2fe3:000b")
    return dev


def le16(value: int) -> bytes:
    if not 0 <= value <= 0xFFFF:
        raise ValueError(f"value out of uint16 range: {value}")
    return struct.pack("<H", value)


def parse_addr(address: str) -> bytes:
    parts = address.split(":")
    if len(parts) != 6:
        raise ValueError(f"invalid Bluetooth address: {address!r}")
    try:
        return bytes(int(part, 16) for part in parts)
    except ValueError as exc:
        raise ValueError(f"invalid Bluetooth address: {address!r}") from exc


def format_addr_human_order(addr: bytes) -> str:
    return ":".join(f"{byte:02X}" for byte in addr)


def format_addr_hci_order(addr: bytes) -> str:
    return ":".join(f"{byte:02X}" for byte in reversed(addr))


def uuid_from_bytes(raw: bytes) -> str:
    if len(raw) == 2:
        return f"0000{int.from_bytes(raw, 'little'):04x}-0000-1000-8000-00805f9b34fb"
    if len(raw) != 16:
        return raw.hex()
    # ATT carries 128-bit UUIDs little-endian.
    be = bytes(reversed(raw))
    return (
        f"{be[0:4].hex()}-{be[4:6].hex()}-{be[6:8].hex()}-"
        f"{be[8:10].hex()}-{be[10:16].hex()}"
    )


def build_set_action_payload(push_ms: int, hold_ms: int, pull_ms: int) -> bytes:
    return b"\x03\x00" + le16(push_ms) + le16(hold_ms) + le16(pull_ms)


def parse_state(value: bytes) -> dict[str, int | bytes]:
    fields = ["battery", "push_ms", "hold_ms", "pull_ms", "repeat", "repeat_interval_ms"]
    result: dict[str, int | bytes] = {"raw": value}
    for idx, name in enumerate(fields):
        start = idx * 2
        if start + 2 <= len(value):
            result[name] = int.from_bytes(value[start : start + 2], "little")
    return result


class UsbHci:
    def __init__(self, verbose: bool = False, acl_pb_flag: int = 0) -> None:
        self.dev = find_dongle()
        self.verbose = verbose
        self.acl_pb_flag = acl_pb_flag
        self._claimed = False

    def __enter__(self) -> "UsbHci":
        try:
            self.dev.set_configuration()
        except usb.core.USBError as exc:
            print(f"warning: set_configuration failed: {exc}", file=sys.stderr)

        try:
            usb.util.claim_interface(self.dev, 0)
            self._claimed = True
        except usb.core.USBError as exc:
            print(f"warning: claim_interface failed: {exc}", file=sys.stderr)
        return self

    def __exit__(self, *_: object) -> None:
        if self._claimed:
            with contextlib.suppress(usb.core.USBError):
                usb.util.release_interface(self.dev, 0)

    def log(self, message: str) -> None:
        if self.verbose:
            print(message, file=sys.stderr)

    def print_descriptors(self) -> None:
        print(f"USB VID:PID: {self.dev.idVendor:04x}:{self.dev.idProduct:04x}")
        print(f"manufacturer: {usb.util.get_string(self.dev, self.dev.iManufacturer)}")
        print(f"product: {usb.util.get_string(self.dev, self.dev.iProduct)}")
        print(f"serial: {usb.util.get_string(self.dev, self.dev.iSerialNumber)}")
        for cfg in self.dev:
            print(f"configuration {cfg.bConfigurationValue}: interfaces={cfg.bNumInterfaces}")
            for intf in cfg:
                print(
                    "  interface "
                    f"{intf.bInterfaceNumber}: class/sub/proto="
                    f"0x{intf.bInterfaceClass:02x}/0x{intf.bInterfaceSubClass:02x}/"
                    f"0x{intf.bInterfaceProtocol:02x}"
                )
                for ep in intf:
                    print(
                        f"    endpoint 0x{ep.bEndpointAddress:02x}: "
                        f"attrs=0x{ep.bmAttributes:02x} max_packet={ep.wMaxPacketSize}"
                    )

    def usb_reset(self) -> None:
        self.log("resetting USB device")
        self.dev.reset()
        time.sleep(0.5)

    def read_event(self, timeout_ms: int) -> bytes:
        event = bytes(self.dev.read(INTERRUPT_IN_EP, 260, timeout=timeout_ms))
        self.log(f"< event {event.hex(' ')}")
        return event

    def read_acl(self, timeout_ms: int) -> tuple[int, int, int, bytes]:
        data = bytes(self.dev.read(BULK_IN_EP, 1024, timeout=timeout_ms))
        self.log(f"< acl {data.hex(' ')}")
        if len(data) < 8:
            raise RuntimeError(f"short ACL packet: {data.hex(' ')}")

        handle_flags, acl_len = struct.unpack_from("<HH", data, 0)
        handle = handle_flags & 0x0FFF
        pb_flag = (handle_flags >> 12) & 0x03
        l2cap = data[4 : 4 + acl_len]
        if len(l2cap) < 4:
            raise RuntimeError(f"short L2CAP packet: {data.hex(' ')}")

        l2_len, cid = struct.unpack_from("<HH", l2cap, 0)
        payload = l2cap[4 : 4 + l2_len]
        return handle, pb_flag, cid, payload

    def poll_event(self) -> bytes | None:
        try:
            return self.read_event(1)
        except usb.core.USBTimeoutError:
            return None

    def hci_command(
        self,
        opcode: int,
        params: bytes = b"",
        timeout_ms: int = 2000,
        allow_status: bool = False,
        check_status: bool = True,
    ) -> bytes:
        packet = opcode.to_bytes(2, "little") + bytes([len(params)]) + params
        self.log(f"> cmd 0x{opcode:04x} {packet.hex(' ')}")
        self.dev.ctrl_transfer(0x20, 0x00, 0, 0, packet, timeout=timeout_ms)

        deadline = time.monotonic() + (timeout_ms / 1000)
        while time.monotonic() < deadline:
            remaining_ms = max(1, int((deadline - time.monotonic()) * 1000))
            try:
                event = self.read_event(min(remaining_ms, 500))
            except usb.core.USBTimeoutError:
                continue

            if len(event) >= 6 and event[0] == EV_COMMAND_COMPLETE:
                completed_opcode = int.from_bytes(event[3:5], "little")
                if completed_opcode != opcode:
                    continue
                payload = event[5:]
                if check_status and payload and payload[0] != 0:
                    raise HciError(
                        f"HCI command 0x{opcode:04x} failed with status 0x{payload[0]:02x}",
                        payload[0],
                    )
                return payload

            if len(event) >= 6 and event[0] == EV_COMMAND_STATUS:
                status = event[2]
                completed_opcode = int.from_bytes(event[4:6], "little")
                if completed_opcode != opcode:
                    continue
                if status != 0 and check_status:
                    raise HciError(
                        f"HCI command 0x{opcode:04x} status 0x{status:02x}",
                        status,
                    )
                if allow_status:
                    return bytes([status])

        raise TimeoutError(f"no HCI completion for opcode 0x{opcode:04x}")

    def read_paired_address(self) -> tuple[str, bytes]:
        payload = self.hci_command(HCI_VENDOR_READ_PAIRED_ADDR, timeout_ms=1500)
        if len(payload) < 7:
            raise RuntimeError(f"unexpected paired-address response: {payload.hex(' ')}")
        status = payload[0]
        if status != 0:
            raise HciError(f"paired-address command failed with status 0x{status:02x}", status)
        raw_human_order = payload[-6:]
        return format_addr_human_order(raw_human_order), payload

    def init_controller(self, reset: bool = False) -> None:
        if reset:
            self.hci_command(HCI_RESET, timeout_ms=2000)
            time.sleep(0.25)

        # Keep this close to what normal host stacks do, but do not fail the
        # whole run if a clone controller rejects a mask bit.
        for opcode, params in (
            (HCI_SET_EVENT_MASK, bytes.fromhex("ff ff ff ff ff ff ff 3f")),
            (HCI_LE_SET_EVENT_MASK, bytes.fromhex("3f 04 00 00 00 00 00 00")),
        ):
            try:
                self.hci_command(opcode, params, timeout_ms=1000)
            except (HciError, TimeoutError, usb.core.USBError) as exc:
                print(f"warning: event-mask command 0x{opcode:04x} failed: {exc}", file=sys.stderr)

    def set_scan_enable(self, enabled: bool) -> None:
        self.hci_command(HCI_LE_SET_SCAN_ENABLE, bytes([1 if enabled else 0, 0]), timeout_ms=1000)

    def scan(self, target: str | None, seconds: float) -> list[Advertisement]:
        self.hci_command(
            HCI_LE_SET_SCAN_PARAMETERS,
            struct.pack("<BHHBB", 0x01, 0x0060, 0x0030, 0x00, 0x00),
            timeout_ms=1000,
        )
        advertisements: list[Advertisement] = []
        target_upper = target.upper() if target else None

        self.set_scan_enable(True)
        try:
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                timeout_ms = max(1, int((deadline - time.monotonic()) * 1000))
                try:
                    event = self.read_event(min(timeout_ms, 500))
                except usb.core.USBTimeoutError:
                    continue

                for adv in self._parse_advertising_event(event):
                    advertisements.append(adv)
                    if target_upper is None or adv.address == target_upper:
                        print(
                            f"advertisement {adv.address} type={adv.address_type} "
                            f"event={adv.event_type} rssi={adv.rssi} data={adv.data.hex(' ')}"
                        )
                    if target_upper is not None and adv.address == target_upper:
                        return advertisements
        finally:
            with contextlib.suppress(Exception):
                self.set_scan_enable(False)

        return advertisements

    def _parse_advertising_event(self, event: bytes) -> Iterable[Advertisement]:
        if len(event) < 4 or event[0] != EV_LE_META or event[2] != SUBEV_LE_ADVERTISING_REPORT:
            return []

        reports: list[Advertisement] = []
        count = event[3]
        pos = 4
        for _ in range(count):
            if pos + 10 > len(event):
                break
            event_type = event[pos]
            address_type = event[pos + 1]
            address = format_addr_hci_order(event[pos + 2 : pos + 8])
            data_len = event[pos + 8]
            data_start = pos + 9
            data_end = data_start + data_len
            if data_end >= len(event):
                break
            data = event[data_start:data_end]
            rssi = struct.unpack("b", event[data_end : data_end + 1])[0]
            reports.append(Advertisement(address, address_type, event_type, rssi, data))
            pos = data_end + 1
        return reports

    def find_address_type(self, address: str, seconds: float) -> int:
        for adv in self.scan(address, seconds):
            if adv.address == address.upper():
                return adv.address_type
        print(f"warning: did not see {address} advertising; trying public address type", file=sys.stderr)
        return 0x00

    def connect(self, address: str, address_type: int, timeout_s: float = 12) -> int:
        hci_addr = bytes(reversed(parse_addr(address)))
        params = struct.pack(
            "<HHBB6sBHHHHHH",
            0x0060,  # scan interval
            0x0030,  # scan window
            0x00,  # initiator filter policy
            address_type,
            hci_addr,
            0x00,  # own public address
            0x0018,  # connection interval min, 30 ms
            0x0028,  # connection interval max, 50 ms
            0x0000,  # latency
            0x01F4,  # supervision timeout, 5 s
            0x0000,  # min CE length
            0x0000,  # max CE length
        )
        self.hci_command(HCI_LE_CREATE_CONNECTION, params, timeout_ms=1500, allow_status=True)

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            timeout_ms = max(1, int((deadline - time.monotonic()) * 1000))
            try:
                event = self.read_event(min(timeout_ms, 1000))
            except usb.core.USBTimeoutError:
                continue

            if len(event) >= 19 and event[0] == EV_LE_META and event[2] == SUBEV_LE_CONNECTION_COMPLETE:
                status = event[3]
                if status != 0:
                    raise HciError(f"LE connection failed with status 0x{status:02x}", status)
                handle = int.from_bytes(event[4:6], "little") & 0x0FFF
                peer = format_addr_hci_order(event[8:14])
                print(f"connected handle=0x{handle:04x} peer={peer}")
                return handle

            if len(event) >= 31 and event[0] == EV_LE_META and event[2] == SUBEV_LE_ENHANCED_CONNECTION_COMPLETE:
                status = event[3]
                if status != 0:
                    raise HciError(f"LE enhanced connection failed with status 0x{status:02x}", status)
                handle = int.from_bytes(event[4:6], "little") & 0x0FFF
                peer = format_addr_hci_order(event[8:14])
                print(f"connected handle=0x{handle:04x} peer={peer}")
                return handle

        with contextlib.suppress(Exception):
            self.hci_command(HCI_LE_CREATE_CONNECTION_CANCEL, timeout_ms=1000)
        raise TimeoutError(f"timed out connecting to {address}")

    def disconnect(self, handle: int) -> None:
        try:
            self.hci_command(HCI_DISCONNECT, struct.pack("<HB", handle, 0x13), timeout_ms=1000, allow_status=True)
        except Exception as exc:
            print(f"warning: disconnect command failed: {exc}", file=sys.stderr)
            return

        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            try:
                event = self.read_event(300)
            except usb.core.USBTimeoutError:
                continue
            if len(event) >= 7 and event[0] == EV_DISCONNECTION_COMPLETE:
                disc_handle = int.from_bytes(event[4:6], "little") & 0x0FFF
                if disc_handle == handle:
                    return

    def read_remote_features(self, handle: int, timeout_s: float = 5) -> bytes:
        self.hci_command(
            HCI_LE_READ_REMOTE_USED_FEATURES,
            struct.pack("<H", handle),
            timeout_ms=1000,
            allow_status=True,
        )
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            timeout_ms = max(1, int((deadline - time.monotonic()) * 1000))
            try:
                event = self.read_event(min(timeout_ms, 1000))
            except usb.core.USBTimeoutError:
                continue

            if (
                len(event) >= 14
                and event[0] == EV_LE_META
                and event[2] == SUBEV_LE_READ_REMOTE_USED_FEATURES_COMPLETE
            ):
                status = event[3]
                event_handle = int.from_bytes(event[4:6], "little") & 0x0FFF
                if event_handle != handle:
                    continue
                if status != 0:
                    raise HciError(f"LE remote features failed with status 0x{status:02x}", status)
                features = event[6:14]
                self.log(f"remote LE features {features.hex(' ')}")
                return features

        raise TimeoutError("timed out waiting for LE remote features")

    def att_request(
        self,
        handle: int,
        payload: bytes,
        expected_opcodes: set[int],
        timeout_ms: int = 3000,
    ) -> bytes:
        self._send_att(handle, payload)

        deadline = time.monotonic() + (timeout_ms / 1000)
        while time.monotonic() < deadline:
            timeout = max(1, int((deadline - time.monotonic()) * 1000))
            try:
                acl_handle, _, cid, att = self.read_acl(min(timeout, 800))
            except usb.core.USBTimeoutError:
                self.poll_event()
                continue
            self.poll_event()
            if acl_handle != handle:
                continue
            if cid == L2CAP_CID_LE_SIGNALING:
                self.handle_l2cap_signal(handle, att)
                continue
            if cid != L2CAP_CID_ATT or not att:
                continue
            if att[0] == ATT_ERROR_RESPONSE and len(att) >= 5:
                raise AttError(att[1], int.from_bytes(att[2:4], "little"), att[4])
            if att[0] in expected_opcodes:
                return att
            self.log(f"ignoring ATT opcode 0x{att[0]:02x}")

        raise TimeoutError(f"timed out waiting for ATT response to 0x{payload[0]:02x}")

    def handle_l2cap_signal(self, handle: int, payload: bytes) -> None:
        if len(payload) < 4:
            return
        code, identifier, length = struct.unpack_from("<BBH", payload, 0)
        params = payload[4 : 4 + length]
        self.log(f"L2CAP signal code=0x{code:02x} id={identifier} params={params.hex(' ')}")

    def att_write_command(self, handle: int, att_handle: int, value: bytes) -> None:
        self._send_att(handle, bytes([ATT_WRITE_CMD]) + le16(att_handle) + value)

    def _send_att(self, handle: int, payload: bytes) -> None:
        l2cap = struct.pack("<HH", len(payload), L2CAP_CID_ATT) + payload
        handle_flags = handle | (self.acl_pb_flag << 12)
        acl = struct.pack("<HH", handle_flags, len(l2cap)) + l2cap
        self.log(f"> acl {acl.hex(' ')}")
        self.dev.write(BULK_OUT_EP, acl, timeout=1000)


class FingerbotRaw:
    def __init__(
        self,
        hci: UsbHci,
        address: str,
        address_type: int | None,
        scan_seconds: float,
        reset: bool,
        skip_mtu: bool,
        skip_features: bool,
        post_connect_delay: float,
    ) -> None:
        self.hci = hci
        self.address = address
        self.address_type = address_type
        self.scan_seconds = scan_seconds
        self.reset = reset
        self.skip_mtu = skip_mtu
        self.skip_features = skip_features
        self.post_connect_delay = post_connect_delay
        self.conn_handle: int | None = None

    def __enter__(self) -> "FingerbotRaw":
        try:
            self.hci.init_controller(reset=self.reset)
            address_type = self.address_type
            if address_type is None:
                address_type = self.hci.find_address_type(self.address, self.scan_seconds)
            self.conn_handle = self.hci.connect(self.address, address_type)
            if not self.skip_features:
                self.hci.read_remote_features(self.conn_handle)
            if self.post_connect_delay > 0:
                time.sleep(self.post_connect_delay)
            if not self.skip_mtu:
                self.exchange_mtu()
        except Exception:
            if self.conn_handle is not None:
                self.hci.disconnect(self.conn_handle)
            raise
        return self

    def __exit__(self, *_: object) -> None:
        if self.conn_handle is not None:
            self.hci.disconnect(self.conn_handle)

    @property
    def handle(self) -> int:
        if self.conn_handle is None:
            raise RuntimeError("not connected")
        return self.conn_handle

    def exchange_mtu(self, mtu: int = 247) -> int:
        try:
            rsp = self.hci.att_request(
                self.handle,
                bytes([ATT_EXCHANGE_MTU_REQ]) + le16(mtu),
                {ATT_EXCHANGE_MTU_RSP},
                timeout_ms=1500,
            )
        except (AttError, TimeoutError) as exc:
            print(f"warning: MTU exchange failed: {exc}", file=sys.stderr)
            return 23
        server_mtu = int.from_bytes(rsp[1:3], "little") if len(rsp) >= 3 else 23
        negotiated = min(mtu, server_mtu)
        print(f"ATT MTU: {negotiated}")
        return negotiated

    def discover_services(self) -> list[Service]:
        services: list[Service] = []
        start = 0x0001
        while start <= 0xFFFF:
            req = bytes([ATT_READ_BY_GROUP_TYPE_REQ]) + struct.pack("<HHH", start, 0xFFFF, UUID_PRIMARY_SERVICE)
            try:
                rsp = self.hci.att_request(self.handle, req, {ATT_READ_BY_GROUP_TYPE_RSP})
            except AttError as exc:
                if exc.code == ATT_ERR_ATTR_NOT_FOUND:
                    break
                raise
            if len(rsp) < 2:
                break
            entry_len = rsp[1]
            if entry_len < 6:
                break
            entries = rsp[2:]
            last_end = start
            for pos in range(0, len(entries) - entry_len + 1, entry_len):
                item = entries[pos : pos + entry_len]
                svc_start, svc_end = struct.unpack_from("<HH", item, 0)
                uuid = uuid_from_bytes(item[4:])
                services.append(Service(svc_start, svc_end, uuid))
                last_end = svc_end
            if last_end >= 0xFFFF:
                break
            start = last_end + 1
        return services

    def discover_characteristics(self) -> list[Characteristic]:
        chars: list[Characteristic] = []
        for service in self.discover_services():
            start = service.start_handle
            while start <= service.end_handle:
                req = bytes([ATT_READ_BY_TYPE_REQ]) + struct.pack("<HHH", start, service.end_handle, UUID_CHARACTERISTIC)
                try:
                    rsp = self.hci.att_request(self.handle, req, {ATT_READ_BY_TYPE_RSP})
                except AttError as exc:
                    if exc.code == ATT_ERR_ATTR_NOT_FOUND:
                        break
                    raise
                if len(rsp) < 2:
                    break
                entry_len = rsp[1]
                if entry_len < 7:
                    break
                entries = rsp[2:]
                last_decl = start
                for pos in range(0, len(entries) - entry_len + 1, entry_len):
                    item = entries[pos : pos + entry_len]
                    decl_handle = int.from_bytes(item[0:2], "little")
                    props = item[2]
                    value_handle = int.from_bytes(item[3:5], "little")
                    uuid = uuid_from_bytes(item[5:])
                    chars.append(Characteristic(decl_handle, value_handle, props, uuid, service))
                    last_decl = decl_handle
                if last_decl >= service.end_handle:
                    break
                start = last_decl + 1
        return chars

    def select_control_characteristic(self, requested_handle: int | None) -> Characteristic:
        if requested_handle is not None:
            dummy_service = Service(0, 0xFFFF, "manual")
            return Characteristic(
                requested_handle,
                requested_handle,
                CHAR_PROP_WRITE | CHAR_PROP_READ,
                "manual",
                dummy_service,
            )

        chars = self.discover_characteristics()
        write_chars = [char for char in chars if char.properties & (CHAR_PROP_WRITE | CHAR_PROP_WRITE_NO_RSP)]
        for char in write_chars:
            if char.service.start_handle == 0x0010 and char.declaration_handle == 0x0011:
                return char
        for char in write_chars:
            if char.declaration_handle == 0x0011 or char.value_handle == 0x0012:
                return char
        if write_chars:
            return write_chars[0]
        raise RuntimeError("no writable Fingerbot characteristic found")

    def read_handle(self, att_handle: int) -> bytes:
        rsp = self.hci.att_request(self.handle, bytes([ATT_READ_REQ]) + le16(att_handle), {ATT_READ_RSP})
        return rsp[1:]

    def write_handle(self, att_handle: int, value: bytes, write_command: bool = False) -> None:
        if write_command:
            self.hci.att_write_command(self.handle, att_handle, value)
            return
        self.hci.att_request(
            self.handle,
            bytes([ATT_WRITE_REQ]) + le16(att_handle) + value,
            {ATT_WRITE_RSP},
            timeout_ms=3000,
        )


def print_characteristics(chars: list[Characteristic]) -> None:
    if not chars:
        print("no characteristics discovered")
        return
    for char in chars:
        print(
            f"service=0x{char.service.start_handle:04x}-0x{char.service.end_handle:04x} "
            f"decl=0x{char.declaration_handle:04x} value=0x{char.value_handle:04x} "
            f"props=0x{char.properties:02x} uuid={char.uuid}"
        )


def print_state(value: bytes) -> None:
    state = parse_state(value)
    print(f"raw: {value.hex(' ')}")
    for key in ("battery", "push_ms", "hold_ms", "pull_ms", "repeat", "repeat_interval_ms"):
        if key in state:
            suffix = " %" if key == "battery" else " ms" if key.endswith("_ms") else ""
            print(f"{key}: {state[key]}{suffix}")


def resolve_address(hci: UsbHci, requested: str | None) -> str:
    if requested:
        return requested.upper()
    paired, payload = hci.read_paired_address()
    print(f"vendor response: {payload.hex(' ')}")
    print(f"paired Fingerbot address: {paired}")
    return paired


def require_press_ack(args: argparse.Namespace) -> None:
    if not args.yes_press:
        raise RuntimeError(
            "refusing to actuate without --yes-press; the Fingerbot is attached "
            "to a laptop power button"
        )


def add_common_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--address", help="Fingerbot BLE address; defaults to dongle-paired address")
    parser.add_argument("--address-type", type=lambda x: int(x, 0), choices=[0, 1], help="0=public, 1=random")
    parser.add_argument("--scan-seconds", type=float, default=4.0)
    parser.add_argument("--handle", type=lambda x: int(x, 0), help="override ATT characteristic value handle")
    parser.add_argument("--write-command", action="store_true", help="use ATT Write Command instead of Write Request")
    parser.add_argument("--reset-controller", action="store_true", help="send HCI Reset before BLE work")
    parser.set_defaults(skip_mtu=True)
    parser.add_argument("--skip-mtu", dest="skip_mtu", action="store_true", help="skip ATT MTU exchange")
    parser.add_argument("--exchange-mtu", dest="skip_mtu", action="store_false", help="attempt ATT MTU exchange")
    parser.add_argument("--skip-features", action="store_true", help="skip LE Read Remote Used Features")
    parser.add_argument("--post-connect-delay", type=float, default=0.25)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("-v", "--verbose", action="store_true")
    parser.add_argument("--acl-pb-flag", type=lambda x: int(x, 0), choices=[0, 2], default=0)
    subparsers = parser.add_subparsers(dest="command")

    subparsers.add_parser("probe", help="read USB descriptors and paired address")
    subparsers.add_parser("usb-reset", help="reset the USB dongle and exit")

    scan_parser = subparsers.add_parser("scan", help="scan for the paired Fingerbot")
    scan_parser.add_argument("--address", help="Fingerbot BLE address; defaults to dongle-paired address")
    scan_parser.add_argument("--seconds", type=float, default=6.0)
    scan_parser.add_argument("--reset-controller", action="store_true")

    discover_parser = subparsers.add_parser("discover", help="connect and print GATT characteristics")
    add_common_args(discover_parser)

    read_parser = subparsers.add_parser("read", help="connect and read Fingerbot state")
    add_common_args(read_parser)

    click_parser = subparsers.add_parser("click", help="press and release the Fingerbot")
    add_common_args(click_parser)
    click_parser.add_argument("--yes-press", action="store_true")
    click_parser.add_argument("--push-ms", type=int)
    click_parser.add_argument("--hold-ms", type=int, default=500)
    click_parser.add_argument("--pull-ms", type=int)
    click_parser.add_argument("--strength", choices=["light", "hard"], default="light")

    push_parser = subparsers.add_parser("push", help="extend the Fingerbot arm")
    add_common_args(push_parser)
    push_parser.add_argument("--yes-press", action="store_true")
    push_parser.add_argument("--push-ms", type=int)
    push_parser.add_argument("--strength", choices=["light", "hard"], default="light")

    pull_parser = subparsers.add_parser("pull", help="retract the Fingerbot arm")
    add_common_args(pull_parser)
    pull_parser.add_argument("--yes-press", action="store_true")
    pull_parser.add_argument("--pull-ms", type=int)
    pull_parser.add_argument("--strength", choices=["light", "hard"], default="light")

    args = parser.parse_args()
    command = args.command or "probe"

    if command in {"click", "push", "pull"}:
        require_press_ack(args)

    with UsbHci(verbose=args.verbose, acl_pb_flag=args.acl_pb_flag) as hci:
        if command == "usb-reset":
            hci.usb_reset()
            print("USB device reset")
            return 0

        if command == "probe":
            hci.print_descriptors()
            paired, payload = hci.read_paired_address()
            print(f"vendor response: {payload.hex(' ')}")
            print(f"paired Fingerbot address: {paired}")
            payload = build_set_action_payload(LIGHT_PUSH_MS, 500, LIGHT_PULL_MS)
            print("light click GATT payload, not sent:", payload.hex(" "))
            return 0

        if command == "scan":
            hci.init_controller(reset=args.reset_controller)
            address = resolve_address(hci, args.address)
            hci.scan(address, args.seconds)
            return 0

        address = resolve_address(hci, args.address)
        with FingerbotRaw(
            hci,
            address=address,
            address_type=args.address_type,
            scan_seconds=args.scan_seconds,
            reset=args.reset_controller,
            skip_mtu=args.skip_mtu,
            skip_features=args.skip_features,
            post_connect_delay=args.post_connect_delay,
        ) as fingerbot:
            if command == "discover":
                print_characteristics(fingerbot.discover_characteristics())
                return 0

            char = fingerbot.select_control_characteristic(args.handle)
            print(
                f"selected characteristic decl=0x{char.declaration_handle:04x} "
                f"value=0x{char.value_handle:04x} props=0x{char.properties:02x} "
                f"uuid={char.uuid}"
            )

            if command == "read":
                print_state(fingerbot.read_handle(char.value_handle))
                return 0

            if command == "click":
                default_push = HARD_PUSH_MS if args.strength == "hard" else LIGHT_PUSH_MS
                default_pull = HARD_PULL_MS if args.strength == "hard" else LIGHT_PULL_MS
                payload = build_set_action_payload(
                    args.push_ms if args.push_ms is not None else default_push,
                    args.hold_ms,
                    args.pull_ms if args.pull_ms is not None else default_pull,
                )
                fingerbot.write_handle(char.value_handle, payload, write_command=args.write_command)
                print("click command sent")
                return 0

            if command == "push":
                default_push = HARD_PUSH_MS if args.strength == "hard" else LIGHT_PUSH_MS
                payload = build_set_action_payload(args.push_ms if args.push_ms is not None else default_push, 0, 0)
                fingerbot.write_handle(char.value_handle, payload, write_command=args.write_command)
                print("push command sent")
                return 0

            if command == "pull":
                default_pull = HARD_PULL_MS if args.strength == "hard" else LIGHT_PULL_MS
                payload = build_set_action_payload(0, 0, args.pull_ms if args.pull_ms is not None else default_pull)
                fingerbot.write_handle(char.value_handle, payload, write_command=args.write_command)
                print("pull command sent")
                return 0

    parser.error(f"unhandled command: {command}")
    return 2


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1)
