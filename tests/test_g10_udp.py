import struct
import unittest

from bidirectional_motor_test.g10_udp import (
    DATA_PREFIX, DATA_SUFFIX, G10ProtocolError, HEADER_SIZE, PAYLOAD_SIZE,
    RECORD_SIZE, SAMPLE_COUNT, decode_g10_payload,
)


MARKERS = (0x0CC7, 0x0CC7, 0, 0, 0, 0,
           0x6666, 0x7777, 0x8888, 0x9999) * 4


def make_packet(sequence=0x4C00):
    payload = bytearray(PAYLOAD_SIZE)
    payload[:len(DATA_PREFIX)] = DATA_PREFIX
    payload[-len(DATA_SUFFIX):] = DATA_SUFFIX
    for sample in range(SAMPLE_COUNT):
        channels = tuple(-1000 + sample * 10 + channel for channel in range(8))
        struct.pack_into(">8h3H", payload, HEADER_SIZE + sample * RECORD_SIZE,
                         *channels, 1, 1, MARKERS[sample])
    struct.pack_into(">H", payload, 972, sequence)
    return bytes(payload)


class DecoderTests(unittest.TestCase):
    def test_decodes_samples_sequence_and_times(self):
        packet = decode_g10_payload(make_packet(0x4C2A))
        self.assertEqual(packet.sequence, 0x4C2A)
        self.assertEqual(len(packet.samples), 40)
        self.assertEqual(packet.samples[0][0], -1000)
        self.assertEqual(packet.samples[39][6], -604)
        times = packet.sample_timestamps(10_000_000, 100_000)
        self.assertEqual(times[0], 6_100_000)
        self.assertEqual(times[-1], 10_000_000)
        self.assertTrue(all(b - a == 100_000 for a, b in zip(times, times[1:])))

    def test_rejects_wrong_length_prefix_suffix_and_markers(self):
        valid = make_packet()
        cases = [
            valid[:-1],
            b"bad" + valid[3:],
            valid[:-5] + b"wrong",
            valid[:HEADER_SIZE + 20] + b"\x12\x34" + valid[HEADER_SIZE + 22:],
        ]
        for payload in cases:
            with self.subTest(length=len(payload)):
                with self.assertRaises(G10ProtocolError):
                    decode_g10_payload(payload)

    def test_validates_timestamp_arguments(self):
        packet = decode_g10_payload(make_packet())
        with self.assertRaises(ValueError):
            packet.sample_timestamps(1, 0)
        with self.assertRaises(ValueError):
            packet.sample_timestamps(1, 100_000, -1)


if __name__ == "__main__":
    unittest.main()
