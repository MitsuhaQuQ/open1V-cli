"""Reference codec for the Open1V MCU bridge protocol.

This contains no USB access. It is used by offline tests and will later be
shared as a behavioral reference with the PC-side implementation.
"""

from __future__ import annotations

from dataclasses import dataclass
import struct

MAGIC = b"O1"
VERSION = 1
MAX_PAYLOAD = 512


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


@dataclass(frozen=True)
class Frame:
    message_type: int
    sequence: int
    payload: bytes = b""

    def encode(self) -> bytes:
        if not 0 <= self.message_type <= 0xFF:
            raise ValueError("message_type is outside uint8")
        if not 0 <= self.sequence <= 0xFFFF:
            raise ValueError("sequence is outside uint16")
        if len(self.payload) > MAX_PAYLOAD:
            raise ValueError("payload exceeds bridge limit")
        body = struct.pack("<2sBBHH", MAGIC, VERSION, self.message_type,
                           self.sequence, len(self.payload)) + self.payload
        return body + struct.pack("<H", crc16_ccitt_false(body))

    @classmethod
    def decode(cls, raw: bytes) -> "Frame":
        if len(raw) < 10:
            raise ValueError("truncated frame")
        magic, version, message_type, sequence, length = struct.unpack("<2sBBHH", raw[:8])
        if magic != MAGIC:
            raise ValueError("invalid magic")
        if version != VERSION:
            raise ValueError("unsupported protocol version")
        if length > MAX_PAYLOAD or len(raw) != 10 + length:
            raise ValueError("invalid payload length")
        expected_crc = struct.unpack("<H", raw[-2:])[0]
        if crc16_ccitt_false(raw[:-2]) != expected_crc:
            raise ValueError("CRC mismatch")
        return cls(message_type, sequence, raw[8:-2])


def exchange(sequence: int, transmit: bytes, expected: int,
             first_timeout_ms: int = 1000,
             inter_byte_timeout_ms: int = 200) -> Frame:
    if len(transmit) > 64:
        raise ValueError("camera transmit payload exceeds 64 bytes")
    if not 0 <= expected <= MAX_PAYLOAD:
        raise ValueError("expected reply exceeds 512 bytes")
    for timeout in (first_timeout_ms, inter_byte_timeout_ms):
        if not 1 <= timeout <= 5000:
            raise ValueError("timeout must be 1..5000 ms")
    payload = struct.pack("<HHHH", expected, first_timeout_ms,
                          inter_byte_timeout_ms, len(transmit)) + transmit
    return Frame(0x10, sequence, payload)

