"""Read-only G10 UDP probe. Does not send any commands to the bench.

Run with the G10 connected to a *separate* Ubuntu Ethernet adapter:
python3 -m bidirectional_motor_test.g10_probe --seconds 12
Never run concurrently with the ROS node: both bind UDP 4800.
"""

import argparse
import socket
import time

from .g10_udp import G10UDPReceiver


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="192.168.127.55")
    parser.add_argument("--port", type=int, default=4800)
    parser.add_argument("--device-ip", default="192.168.127.56")
    parser.add_argument("--seconds", type=float, default=12.0)
    args = parser.parse_args(argv)
    if args.seconds <= 0:
        parser.error("--seconds must be positive")

    try:
        receiver = G10UDPReceiver(bind_host=args.bind, port=args.port,
                                  expected_device_ip=args.device_ip)
    except OSError as exc:
        parser.error("cannot bind UDP %s:%s: %s" % (args.bind, args.port, exc))
    receiver.start()
    last_sequence = None
    gaps = 0
    count = 0
    minimum = [float("inf")] * 8
    maximum = [float("-inf")] * 8
    first_ns = None
    last_ns = None
    print("Listening ONLY; no initialization / control datagrams are sent.")
    try:
        stop_at = time.monotonic() + args.seconds
        while time.monotonic() < stop_at:
            try:
                recv_ns, packet = receiver.packets.get(timeout=0.2)
            except __import__("queue").Empty:
                continue
            count += 1
            if first_ns is None:
                first_ns = recv_ns
            last_ns = recv_ns
            if last_sequence is not None:
                expected = (last_sequence + 1) & 65535
                if packet.sequence != expected:
                    gaps += (packet.sequence - expected) & 65535
            last_sequence = packet.sequence
            for sample in packet.samples:
                for ch, val in enumerate(sample):
                    minimum[ch] = min(minimum[ch], val)
                    maximum[ch] = max(maximum[ch], val)
    except KeyboardInterrupt:
        pass
    finally:
        receiver.stop()
    print("decoded_packets=%d invalid=%d queue_dropped=%d sequence_gaps=%d" %
          (count, receiver.invalid_packets, receiver.dropped_packets, gaps))
    print("ADC min/max per channel:",
          list(zip(minimum, maximum)) if count else "(no samples)")
    if count < 1:
        print("NO DATA: Check the dedicated NIC/IP and firewall. The receiver "
              "does not implement an unknown vendor initialization handshake; "
              "a Windows startup pcap may be required.")
        return 2
    if count > 1 and last_ns > first_ns:
        print("Observed packet rate: %.1f packet/s" %
              ((count - 1) * 1e9 / (last_ns - first_ns)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
