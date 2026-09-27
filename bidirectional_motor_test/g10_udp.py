"""Decoder and UDP receiver for the DET G10 10 kHz data stream.

The layout was established from captures made with G10X.322.0.0.492 and a
DET G10-10KGF-5 (SN DET50316-62-50307-1).  The device sends 986-byte UDP
payloads from 192.168.127.56:5000 to the host on port 4800.  Each payload
contains 40 consecutive samples of eight signed, big-endian ADC channels.
At roughly 250 packets/s this is the bench's nominal 10 ksample/s mode.

Calibration is deliberately not done here: raw ADC counts are the only
quantity present on the wire.  Zero and scale are test-bench specific.
"""

from __future__ import annotations

from dataclasses import dataclass
import queue
import socket
import struct
import threading
import time


PAYLOAD_SIZE = 986
DATA_PREFIX = bytes.fromhex("fffefd")
DATA_SUFFIX = bytes.fromhex("0102030d0a")
HEADER_SIZE = 10
SAMPLE_COUNT = 40
ADC_CHANNEL_COUNT = 8
RECORD_SIZE = 22
PACKET_SEQUENCE_OFFSET = 972
DEFAULT_SAMPLE_PERIOD_NS = 100_000

_RECORD = struct.Struct(">8h3H")
_EXPECTED_MARKERS = (
    0x0CC7, 0x0CC7, 0, 0, 0, 0, 0x6666, 0x7777, 0x8888, 0x9999,
) * 4


class G10ProtocolError(ValueError):
    """Raised when a datagram is not a supported G10 data packet."""


@dataclass(frozen=True)
class G10Packet:
    sequence: int
    samples: tuple[tuple[int, ...], ...]

    def sample_timestamps(self, received_ns, sample_period_ns=DEFAULT_SAMPLE_PERIOD_NS,
                          arrival_bias_ns=0):
        """Return oldest-to-newest sample times on the receiver monotonic clock.

        ``arrival_bias_ns`` can remove a separately measured fixed device/network
        delay.  It should remain zero until such a measurement exists.
        """
        if sample_period_ns <= 0 or arrival_bias_ns < 0:
            raise ValueError("sample period must be positive; arrival bias >= 0")
        newest = int(received_ns) - int(arrival_bias_ns)
        oldest = newest - (SAMPLE_COUNT - 1) * int(sample_period_ns)
        return tuple(oldest + i * int(sample_period_ns)
                     for i in range(SAMPLE_COUNT))


def decode_g10_payload(payload):
    """Decode one 986-byte G10 UDP payload into forty 8-channel samples."""
    if len(payload) != PAYLOAD_SIZE:
        raise G10ProtocolError(
            "expected %d payload bytes, got %d" % (PAYLOAD_SIZE, len(payload)))
    if not payload.startswith(DATA_PREFIX):
        raise G10ProtocolError("unexpected G10 data prefix")
    if not payload.endswith(DATA_SUFFIX):
        raise G10ProtocolError("unexpected G10 data suffix")

    samples = []
    markers = []
    for index in range(SAMPLE_COUNT):
        record = _RECORD.unpack_from(payload, HEADER_SIZE + index * RECORD_SIZE)
        samples.append(tuple(record[:ADC_CHANNEL_COUNT]))
        markers.append(record[-1])
    if tuple(markers) != _EXPECTED_MARKERS:
        raise G10ProtocolError("unexpected record marker sequence")

    sequence = struct.unpack_from(">H", payload, PACKET_SEQUENCE_OFFSET)[0]
    return G10Packet(sequence=sequence, samples=tuple(samples))


class G10UDPReceiver:
    """Small stoppable receiver thread; decoded packets are read from ``packets``."""

    def __init__(self, bind_host="0.0.0.0", port=4800,
                 expected_device_ip="192.168.127.56", queue_size=1024):
        self.expected_device_ip = str(expected_device_ip)
        self.packets = queue.Queue(maxsize=int(queue_size))
        self.received_packets = 0
        self.invalid_packets = 0
        self.dropped_packets = 0
        self.error = None
        self._stop = threading.Event()
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        self._socket.settimeout(0.2)
        self._socket.bind((str(bind_host), int(port)))
        self._thread = threading.Thread(
            target=self._run, name="g10_udp_receiver", daemon=True)

    def start(self):
        self._thread.start()

    def _run(self):
        try:
            while not self._stop.is_set():
                try:
                    payload, address = self._socket.recvfrom(2048)
                except socket.timeout:
                    continue
                except OSError:
                    if self._stop.is_set():
                        break
                    raise
                received_ns = time.monotonic_ns()
                if self.expected_device_ip and address[0] != self.expected_device_ip:
                    continue
                try:
                    packet = decode_g10_payload(payload)
                except G10ProtocolError:
                    self.invalid_packets += 1
                    continue
                self.received_packets += 1
                try:
                    self.packets.put_nowait((received_ns, packet))
                except queue.Full:
                    self.dropped_packets += 1
        except Exception as exc:  # surfaced by the ROS timer, not hidden in a thread
            self.error = exc

    def stop(self):
        self._stop.set()
        self._socket.close()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
