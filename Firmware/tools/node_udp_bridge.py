#!/usr/bin/env python3
"""MAVLink sysid UDP bridge for SiK strict-TDM tests (no COBS).

Port mapping (defaults):
  sysid 1: UDP listen :14551 -> serial uplink, serial downlink -> :14651
  sysid 2: UDP listen :14552 -> serial uplink, serial downlink -> :14652
  ...

Uplink (GCS -> telemetry module):
  Any bytes received on UDP :14550+N are written to serial unchanged.

Downlink (telemetry module -> GCS):
  Parse MAVLink packets from serial stream, extract system id, and forward
  each complete packet to UDP :14650+sysid.

This intentionally does not implement COBS/HOSTMUX framing.
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import time

from pymavlink import mavutil

logger = logging.getLogger("node_bridge")
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(message)s",
    datefmt="%H:%M:%S",
)

DEFAULT_NODES = [1, 2, 3, 4]


class _NodeUDP(asyncio.DatagramProtocol):
    def __init__(self, node_id: int, udp_port: int, write_serial, gcs_ip: str, gcs_out_port: int):
        self.node_id = node_id
        self.udp_port = udp_port
        self._write_serial = write_serial
        self._gcs_addr = (gcs_ip, gcs_out_port)
        self.transport: asyncio.DatagramTransport | None = None

    def connection_made(self, transport):
        self.transport = transport

    def datagram_received(self, data: bytes, addr: tuple[str, int]):
        # Uplink path: forward unchanged bytes to telemetry serial.
        self._write_serial(data)
        logger.info(
            f"[UDP :{self.udp_port}] Uplink: {len(data)} B from {addr} -> serial"
        )

    def forward(self, packet: bytes):
        # Downlink path for this sysid.
        if self.transport:
            self.transport.sendto(packet, self._gcs_addr)

class _SerialReader:
    """Non-blocking serial reader via executor thread using pymavlink."""

    def __init__(self, ser, loop: asyncio.AbstractEventLoop, nodes: dict[int, _NodeUDP]):
        self._ser = ser
        self._loop = loop
        self._nodes = nodes
        # Initialize robust pymavlink parser (dialect agnostic headers)
        self._mav = mavutil.mavlink.MAVLink(None)
        self._mav.robust_parsing = True
        
        self._running = True
        self._task = loop.create_task(self._run())

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

            if not chunk:
                continue

            messages = self._mav.parse_buffer(chunk)
            if messages:
                for msg in messages:
                    if msg.get_type() == 'BAD_DATA':
                        continue

                    sysid = msg.get_srcSystem()
                    proto = self._nodes.get(sysid)
                    if proto:
                        proto.forward(msg.get_msgbuf())
                    else:
                        logger.debug(f"[Serial] Unmapped sysid {sysid}; packet dropped")

    def _read(self) -> bytes:
        try:
            waiting = self._ser.in_waiting
            if waiting > 0:
                return self._ser.read(waiting)

            time.sleep(0.005)
            return b""
        except Exception:
            return b""

    def stop(self):
        self._running = False
        self._task.cancel()


async def _main():
    parser = argparse.ArgumentParser(description="MAVLink sysid UDP bridge (no COBS)")
    parser.add_argument("--port", required=True,
                        help="Telemetry serial port (/dev/ttyUSB0 or COM3)")
    parser.add_argument("--baud", type=int, default=57600,
                        help="Serial baud rate (default: 57600)")
    parser.add_argument("--node", type=int, action="append", metavar="N",
                        help="SysID/node to expose (repeat; default: 1-4)")
    parser.add_argument("--port-base", type=int, default=14550,
                        help="UDP listen base; sysid N listens on base+N (default: 14550)")
    parser.add_argument("--bind", default="0.0.0.0",
                        help="UDP bind address (default: 0.0.0.0)")
    parser.add_argument("--gcs-ip", default="127.0.0.1",
                        help="Downlink push IP (default: 127.0.0.1)")
    parser.add_argument("--gcs-out-base", type=int, default=14650,
                        help="Downlink push base; sysid N pushes to base+N (default: 14650)")
    args = parser.parse_args()

    node_ids = sorted(set(args.node)) if args.node else DEFAULT_NODES
    loop = asyncio.get_running_loop()

    import serial as _serial
    ser = _serial.Serial(
        port=args.port,
        baudrate=args.baud,
        timeout=0,
        dsrdtr=False,
    )

    def _write_serial(data: bytes):
        try:
            ser.write(data)
        except Exception as exc:
            logger.error(f"[Serial] Write error: {exc}")

    nodes: dict[int, _NodeUDP] = {}
    for nid in node_ids:
        udp_port = args.port_base + nid
        gcs_out_port = args.gcs_out_base + nid

        _, proto = await loop.create_datagram_endpoint(
            lambda nid=nid, p=udp_port, gp=gcs_out_port: _NodeUDP(
                nid, p, _write_serial, args.gcs_ip, gp
            ),
            local_addr=(args.bind, udp_port),
        )
        nodes[nid] = proto

        logger.info(
            f"SysID {nid:<4} listen :{udp_port}  push -> {args.gcs_ip}:{gcs_out_port}"
        )

    reader = _SerialReader(ser, loop, nodes)

    logger.info("Bridge running. Ctrl-C to stop.")
    try:
        await asyncio.sleep(float("inf"))
    except asyncio.CancelledError:
        pass
    finally:
        reader.stop()
        ser.close()


if __name__ == "__main__":
    try:
        asyncio.run(_main())
    except KeyboardInterrupt:
        logger.info("Stopped.")
