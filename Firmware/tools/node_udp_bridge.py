#!/usr/bin/env python3
"""Transparent UDP-to-NodeID bridge for SiK multipoint radios (HOSTMUX/COBS mode).

Routing key: UDP port number ONLY.  The bridge is protocol-agnostic — it does
not inspect, parse, or modify the payload in any way.

Port assignment
---------------
  --port-base B  (default 14550)
  Control / AT   -> B        (UDP)
  Drone NodeID 1 -> B + 1    (e.g. :14551)
  Drone NodeID 2 -> B + 2    (e.g. :14552)
  Drone NodeID N -> B + N    (e.g. :14550+N)

Uplink  (GCS -> Radio) : packet on :B+N  => COBS frame  TO NodeID N
Downlink (Radio -> GCS): COBS frame FROM NodeID N => packet on :B+N

Works on Linux (/dev/ttyUSBx) and Windows (COMx) — serial reading is done
in a background thread via run_in_executor, never blocking the event loop.

Usage
-----
  # Two drones
  python3 node_udp_bridge.py --port /dev/ttyUSB0 --node 1 --node 2

  # Windows, three drones, custom base port
  python3 node_udp_bridge.py --port COM7 --node 1 --node 2 --node 3 --port-base 14560

  # Send startup AT commands
  python3 node_udp_bridge.py --port /dev/ttyUSB0 --node 1 --init-at ATS18=4
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import time

logger = logging.getLogger("node_bridge")
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(message)s",
    datefmt="%H:%M:%S",
)

# ── Protocol constants ────────────────────────────────────────────────────────
MAX_PAYLOAD      = 244        # max bytes per hostmux frame
FRAME_TO_NODE    = 1
FRAME_FROM_NODE  = 2
FRAME_AT_CMD     = 3
FRAME_AT_REPLY   = 4
BROADCAST_NODE   = 0xFFFF

DEFAULT_NODES    = [1, 2, 3, 4]

# ── CRC-16 ────────────────────────────────────────────────────────────────────
_T1 = [
    0x00,0x00,0x10,0x21,0x20,0x42,0x30,0x63,0x40,0x84,0x50,0xA5,0x60,0xC6,0x70,0xE7,
    0x81,0x08,0x91,0x29,0xA1,0x4A,0xB1,0x6B,0xC1,0x8C,0xD1,0xAD,0xE1,0xCE,0xF1,0xEF,
    0x12,0x31,0x02,0x10,0x32,0x73,0x22,0x52,0x52,0xB5,0x42,0x94,0x72,0xF7,0x62,0xD6,
    0x93,0x39,0x83,0x18,0xB3,0x7B,0xA3,0x5A,0xD3,0xBD,0xC3,0x9C,0xF3,0xFF,0xE3,0xDE,
    0x24,0x62,0x34,0x43,0x04,0x20,0x14,0x01,0x64,0xE6,0x74,0xC7,0x44,0xA4,0x54,0x85,
    0xA5,0x6A,0xB5,0x4B,0x85,0x28,0x95,0x09,0xE5,0xEE,0xF5,0xCF,0xC5,0xAC,0xD5,0x8D,
    0x36,0x53,0x26,0x72,0x16,0x11,0x06,0x30,0x76,0xD7,0x66,0xF6,0x56,0x95,0x46,0xB4,
    0xB7,0x5B,0xA7,0x7A,0x97,0x19,0x87,0x38,0xF7,0xDF,0xE7,0xFE,0xD7,0x9D,0xC7,0xBC,
    0x48,0xC4,0x58,0xE5,0x68,0x86,0x78,0xA7,0x08,0x40,0x18,0x61,0x28,0x02,0x38,0x23,
    0xC9,0xCC,0xD9,0xED,0xE9,0x8E,0xF9,0xAF,0x89,0x48,0x99,0x69,0xA9,0x0A,0xB9,0x2B,
    0x5A,0xF5,0x4A,0xD4,0x7A,0xB7,0x6A,0x96,0x1A,0x71,0x0A,0x50,0x3A,0x33,0x2A,0x12,
    0xDB,0xFD,0xCB,0xDC,0xFB,0xBF,0xEB,0x9E,0x9B,0x79,0x8B,0x58,0xBB,0x3B,0xAB,0x1A,
    0x6C,0xA6,0x7C,0x87,0x4C,0xE4,0x5C,0xC5,0x2C,0x22,0x3C,0x03,0x0C,0x60,0x1C,0x41,
    0xED,0xAE,0xFD,0x8F,0xCD,0xEC,0xDD,0xCD,0xAD,0x2A,0xBD,0x0B,0x8D,0x68,0x9D,0x49,
    0x7E,0x97,0x6E,0xB6,0x5E,0xD5,0x4E,0xF4,0x3E,0x13,0x2E,0x32,0x1E,0x51,0x0E,0x70,
    0xFF,0x9F,0xEF,0xBE,0xDF,0xDD,0xCF,0xFC,0xBF,0x1B,0xAF,0x3A,0x9F,0x59,0x8F,0x78,
]
_T2 = [
    0x91,0x88,0x81,0xA9,0xB1,0xCA,0xA1,0xEB,0xD1,0x0C,0xC1,0x2D,0xF1,0x4E,0xE1,0x6F,
    0x10,0x80,0x00,0xA1,0x30,0xC2,0x20,0xE3,0x50,0x04,0x40,0x25,0x70,0x46,0x60,0x67,
    0x83,0xB9,0x93,0x98,0xA3,0xFB,0xB3,0xDA,0xC3,0x3D,0xD3,0x1C,0xE3,0x7F,0xF3,0x5E,
    0x02,0xB1,0x12,0x90,0x22,0xF3,0x32,0xD2,0x42,0x35,0x52,0x14,0x62,0x77,0x72,0x56,
    0xB5,0xEA,0xA5,0xCB,0x95,0xA8,0x85,0x89,0xF5,0x6E,0xE5,0x4F,0xD5,0x2C,0xC5,0x0D,
    0x34,0xE2,0x24,0xC3,0x14,0xA0,0x04,0x81,0x74,0x66,0x64,0x47,0x54,0x24,0x44,0x05,
    0xA7,0xDB,0xB7,0xFA,0x87,0x99,0x97,0xB8,0xE7,0x5F,0xF7,0x7E,0xC7,0x1D,0xD7,0x3C,
    0x26,0xD3,0x36,0xF2,0x06,0x91,0x16,0xB0,0x66,0x57,0x76,0x76,0x46,0x15,0x56,0x34,
    0xD9,0x4C,0xC9,0x6D,0xF9,0x0E,0xE9,0x2F,0x99,0xC8,0x89,0xE9,0xB9,0x8A,0xA9,0xAB,
    0x58,0x44,0x48,0x65,0x78,0x06,0x68,0x27,0x18,0xC0,0x08,0xE1,0x38,0x82,0x28,0xA3,
    0xCB,0x7D,0xDB,0x5C,0xEB,0x3F,0xFB,0x1E,0x8B,0xF9,0x9B,0xD8,0xAB,0xBB,0xBB,0x9A,
    0x4A,0x75,0x5A,0x54,0x6A,0x37,0x7A,0x16,0x0A,0xF1,0x1A,0xD0,0x2A,0xB3,0x3A,0x92,
    0xFD,0x2E,0xED,0x0F,0xDD,0x6C,0xCD,0x4D,0xBD,0xAA,0xAD,0x8B,0x9D,0xE8,0x8D,0xC9,
    0x7C,0x26,0x6C,0x07,0x5C,0x64,0x4C,0x45,0x3C,0xA2,0x2C,0x83,0x1C,0xE0,0x0C,0xC1,
    0xEF,0x1F,0xFF,0x3E,0xCF,0x5D,0xDF,0x7C,0xAF,0x9B,0xBF,0xBA,0x8F,0xD9,0x9F,0xF8,
    0x6E,0x17,0x7E,0x36,0x4E,0x55,0x5E,0x74,0x2E,0x93,0x3E,0xB2,0x0E,0xD1,0x1E,0xF0,
]

def _crc16(data: bytes) -> int:
    h = l = 0
    for b in data:
        k = (h << 1) & 0xFF
        if h & 0x80:
            h = l ^ _T2[k]; l = b ^ _T2[(k + 1) & 0xFF]
        else:
            h = l ^ _T1[k]; l = b ^ _T1[(k + 1) & 0xFF]
    return (h << 8) | l

# ── COBS ─────────────────────────────────────────────────────────────────────
def _cobs_enc(data: bytes) -> bytes:
    out = bytearray(); ci = 0; out.append(0); c = 1
    for b in data:
        if b == 0:
            out[ci] = c; ci = len(out); out.append(0); c = 1
        else:
            out.append(b); c += 1
            if c == 0xFF:
                out[ci] = c; ci = len(out); out.append(0); c = 1
    out[ci] = c
    return bytes(out)

def _cobs_dec(data: bytes) -> bytes:
    out = bytearray(); i = 0
    while i < len(data):
        c = data[i]; i += 1
        if c == 0 or i + c - 1 > len(data) + 1:
            raise ValueError("bad COBS")
        for _ in range(1, c):
            if i >= len(data): raise ValueError("truncated")
            out.append(data[i]); i += 1
        if c != 0xFF and i < len(data):
            out.append(0)
    return bytes(out)

# ── Hostmux frame encode / decode ─────────────────────────────────────────────
def _frame_encode(ftype: int, node: int, payload: bytes) -> bytes:
    """Build a complete hostmux COBS frame (including trailing 0x00 sentinel)."""
    raw = bytearray([ftype, node & 0xFF, (node >> 8) & 0xFF, len(payload)])
    raw += payload
    crc = _crc16(raw)
    raw += bytes([(crc >> 8) & 0xFF, crc & 0xFF])
    return _cobs_enc(bytes(raw)) + b"\x00"

def _frame_decode(frame: bytes) -> tuple[int, int, bytes]:
    """Decode a raw (no trailing sentinel) COBS frame.
    Returns (frame_type, node_id, payload) or raises ValueError."""
    raw = _cobs_dec(frame)
    if len(raw) < 6:
        raise ValueError("frame too short")
    plen = raw[3]
    if len(raw) != plen + 6:
        raise ValueError("length mismatch")
    if ((raw[-2] << 8) | raw[-1]) != _crc16(raw[:-2]):
        raise ValueError("CRC mismatch")
    return raw[0], raw[1] | (raw[2] << 8), bytes(raw[4:-2])

# ── Per-node UDP protocol ─────────────────────────────────────────────────────
class _NodeUDP(asyncio.DatagramProtocol):
    """One UDP socket per remote node.

    Downlink (RF -> GCS) is pushed immediately to the statically configured
    (gcs_ip, gcs_out_port) — no registration packet from GCS required.
    GCS simply binds to gcs_out_port to receive data.

    Uplink (GCS -> RF) still works: any packet received on this socket's
    listen port is forwarded over serial to the node.
    """

    def __init__(self, node_id: int, udp_port: int, write_serial,
                 gcs_ip: str, gcs_out_port: int):
        self.node_id       = node_id
        self.udp_port      = udp_port
        self._write_serial = write_serial
        self._gcs_addr     = (gcs_ip, gcs_out_port)   # static push target
        self.transport: asyncio.DatagramTransport | None = None

    def connection_made(self, transport):
        self.transport = transport

    def datagram_received(self, data: bytes, addr: tuple[str, int]):
        """Uplink: GCS sent data to our listen port — forward to node over RF."""
        for off in range(0, len(data), MAX_PAYLOAD):
            chunk = data[off : off + MAX_PAYLOAD]
            self._write_serial(_frame_encode(FRAME_TO_NODE, self.node_id, chunk))
        logger.info(
            f"[UDP :{self.udp_port}] Uplink: {len(data)} B from {addr} -> Node {self.node_id}"
        )

    def forward(self, payload: bytes):
        """Downlink: push RF payload directly to the static GCS address."""
        if self.transport:
            self.transport.sendto(payload, self._gcs_addr)
            logger.info(
                f"[Node {self.node_id}] Downlink: {len(payload)} B -> {self._gcs_addr}"
            )

# ── AT control protocol ───────────────────────────────────────────────────────
class _ControlUDP(asyncio.DatagramProtocol):
    """Simple plain-text AT command interface on the base port.
    Send 'ATI5\\n', receive the radio's reply as plain text."""

    def __init__(self, write_serial):
        self._write_serial = write_serial
        self.transport: asyncio.DatagramTransport | None = None
        self.gcs_addr:  tuple[str, int] | None = None

    def connection_made(self, transport):
        self.transport = transport

    def datagram_received(self, data: bytes, addr):
        self.gcs_addr = addr
        for line in data.split(b"\n"):
            line = line.strip()
            if line:
                self._write_serial(_frame_encode(FRAME_AT_CMD, 0, line))

    def send_reply(self, text: bytes):
        if self.transport and self.gcs_addr:
            self.transport.sendto(text, self.gcs_addr)

# ── Serial reader (cross-platform thread executor) ────────────────────────────
class _SerialReader:
    """Reads the physical serial port in a background thread so the asyncio
    event loop is never blocked.  Compatible with Windows COM ports."""

    def __init__(self, ser, loop: asyncio.AbstractEventLoop,
                 nodes: dict[int, _NodeUDP], ctrl: _ControlUDP):
        self._ser     = ser
        self._loop    = loop
        self._nodes   = nodes
        self._ctrl    = ctrl
        self._buf     = bytearray()
        self._running = True
        self._task    = loop.create_task(self._run())

    async def _run(self):
        while self._running:
            try:
                chunk = await self._loop.run_in_executor(None, self._read)
            except asyncio.CancelledError:
                break
            except Exception as exc:
                logger.error(f"[Serial] Read error: {exc}")
                await asyncio.sleep(1)
                continue
            if chunk:
                self._buf.extend(chunk)
                self._drain()

    def _read(self) -> bytes:
        try:
            w = self._ser.in_waiting
            if w > 0:
                return self._ser.read(w)
            # Nothing available — yield for 5 ms to avoid busy-spinning
            # and prevent blocking the executor thread for up to 0.5 s,
            # which would delay time-critical MAVLink heartbeat packets.
            time.sleep(0.005)
            return b""
        except Exception:
            return b""

    def _drain(self):
        """Parse all complete COBS frames from the internal accumulation buffer."""
        while True:
            try:
                end = self._buf.index(0)
            except ValueError:
                return                    # no complete frame yet
            raw = bytes(self._buf[:end])
            del self._buf[:end + 1]
            if not raw:
                continue
            try:
                ftype, node_id, payload = _frame_decode(raw)
            except ValueError as exc:
                logger.debug(f"[Serial] Discarded frame: {exc}")
                continue

            if ftype == FRAME_FROM_NODE:
                if node_id == 0:
                    # NodeID 0 = Base Radio status (RADIO_STATUS / RSSI).
                    # Broadcast to ALL active GCS ports so every drone's
                    # dashboard can display the radio link health.
                    for proto in self._nodes.values():
                        proto.forward(payload)
                    logger.debug(f"[Node 0] RADIO_STATUS broadcast ({len(payload)} B) -> {len(self._nodes)} GCS port(s)")
                else:
                    proto = self._nodes.get(node_id)
                    if proto:
                        proto.forward(payload)
                    else:
                        logger.debug(f"[Serial] Frame from unregistered NodeID {node_id}")

            elif ftype == FRAME_AT_REPLY:
                self._ctrl.send_reply(payload)

            else:
                logger.debug(f"[Serial] Unexpected frame type {ftype}")

    def stop(self):
        self._running = False
        self._task.cancel()

# ── Entry point ───────────────────────────────────────────────────────────────
async def _main():
    parser = argparse.ArgumentParser(
        description="Transparent UDP-to-NodeID bridge for SiK HOSTMUX radios.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--port", required=True,
                        help="Base radio serial port (/dev/ttyUSB0 or COM3)")
    parser.add_argument("--baud", type=int, default=57600,
                        help="Serial baud rate (default: 57600)")
    parser.add_argument("--node", type=int, action="append", metavar="N",
                        help="NodeID to expose (repeat for each drone, default: 1-4)")
    parser.add_argument("--port-base", type=int, default=14550,
                        help="UDP listen base port; NodeID N uses base+N (default: 14550)")
    parser.add_argument("--bind", default="0.0.0.0",
                        help="UDP bind address (default: 0.0.0.0)")
    parser.add_argument("--gcs-ip", default="127.0.0.1",
                        help="IP address to push downlink data to (default: 127.0.0.1)")
    parser.add_argument("--gcs-out-base", type=int, default=14650,
                        help="GCS receive base port; NodeID N uses base+N "
                             "(default: 14650, so Node 1 -> :14651)")
    parser.add_argument("--init-at", action="append", default=[], metavar="CMD",
                        help="AT command to send at startup, e.g. --init-at ATS18=4")
    args = parser.parse_args()

    node_ids = sorted(set(args.node)) if args.node else DEFAULT_NODES
    loop = asyncio.get_running_loop()

    import serial as _serial
    ser = _serial.Serial(
        port=args.port,
        baudrate=args.baud,
        timeout=0,          # non-blocking: eliminates 0-500 ms bridge-induced jitter
        dsrdtr=False,
    )

    def _write(data: bytes):
        try:
            ser.write(data)
        except Exception as exc:
            logger.error(f"[Serial] Write error: {exc}")

    # ── Bind control port (AT commands) ──────────────────────────────────────
    ctrl_tr, ctrl = await loop.create_datagram_endpoint(
        lambda: _ControlUDP(_write),
        local_addr=(args.bind, args.port_base),
    )
    logger.info(f"AT control  -> udp://{args.bind}:{args.port_base}")

    # ── Bind one UDP socket per node ──────────────────────────────────────────
    nodes: dict[int, _NodeUDP] = {}
    for nid in node_ids:
        udp_port     = args.port_base + nid
        gcs_out_port = args.gcs_out_base + nid
        _, proto = await loop.create_datagram_endpoint(
            lambda nid=nid, p=udp_port, gp=gcs_out_port: _NodeUDP(
                nid, p, _write, args.gcs_ip, gp
            ),
            local_addr=(args.bind, udp_port),
        )
        nodes[nid] = proto
        logger.info(
            f"Node {nid:<4}  listen :{ udp_port}  "
            f"push -> {args.gcs_ip}:{gcs_out_port}"
        )

    # ── Start serial reader ───────────────────────────────────────────────────
    reader = _SerialReader(ser, loop, nodes, ctrl)

    # ── Optional startup AT commands ──────────────────────────────────────────
    for cmd in args.init_at:
        _write(_frame_encode(FRAME_AT_CMD, 0, cmd.strip().encode()))
        await asyncio.sleep(0.05)
    if args.init_at:
        _write(_frame_encode(FRAME_AT_CMD, 0, b"AT&W"))
        await asyncio.sleep(0.1)

    logger.info("Bridge running. Ctrl-C to stop.")
    try:
        await asyncio.sleep(float("inf"))
    except asyncio.CancelledError:
        pass
    finally:
        reader.stop()
        ctrl_tr.close()
        ser.close()


if __name__ == "__main__":
    try:
        asyncio.run(_main())
    except KeyboardInterrupt:
        logger.info("Stopped.")
