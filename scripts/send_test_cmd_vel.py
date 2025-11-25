#!/usr/bin/env python3
"""
Send a fixed (v, w) command to the Pico in TEST_STAGE=3 via PacketSerial and print encoder feedback.

Usage:
  python3 scripts/send_test_cmd_vel.py --port /dev/ttyACM0 --v 0.1 --w 0.0 --hz 10
"""

import argparse
import struct
import sys
import time

import serial


HEADER_SIZE = 8
BODY_SIZE = 64
PACKET_SIZE = HEADER_SIZE + BODY_SIZE
LOCAL_PORT = 8888


def checksum(data: bytes) -> int:
    """16-bit one's complement checksum over body."""
    s = 0
    length = len(data)
    for i in range(0, length, 2):
        word = data[i] << 8
        if i + 1 < length:
            word |= data[i + 1]
        s += word
    s = (s & 0xFFFF) + (s >> 16)
    return (~s) & 0xFFFF


def build_packet(v: float, w: float) -> bytes:
    body = bytearray(BODY_SIZE)
    struct.pack_into("<f", body, 0, v)
    struct.pack_into("<f", body, 4, w)
    csum = checksum(body)
    header = struct.pack("<HHHH", LOCAL_PORT, LOCAL_PORT, PACKET_SIZE, csum)
    return header + body


def parse_args():
    parser = argparse.ArgumentParser(description="Send test cmd_vel to Pico (TEST_STAGE=3)")
    parser.add_argument("--port", required=True, help="Serial port, e.g., /dev/ttyACM0")
    parser.add_argument("--v", type=float, default=0.0, help="Linear velocity [m/s]")
    parser.add_argument("--w", type=float, default=0.0, help="Angular velocity [rad/s]")
    parser.add_argument("--hz", type=float, default=10.0, help="Send frequency [Hz]")
    parser.add_argument("--count", type=int, default=0, help="Number of packets to send (0=unlimited)")
    return parser.parse_args()


def main():
    args = parse_args()
    interval = 1.0 / args.hz if args.hz > 0 else 0.1
    pkt = build_packet(args.v, args.w)

    try:
        ser = serial.Serial(args.port, 115200, timeout=0.05)
    except serial.SerialException as e:
        sys.stderr.write(f"Failed to open port {args.port}: {e}\n")
        sys.exit(1)

    print(f"Sending v={args.v:.3f} [m/s], w={args.w:.3f} [rad/s] to {args.port} at {args.hz} Hz")
    sent = 0
    try:
        while args.count == 0 or sent < args.count:
            sent += 1
            ser.write(pkt)
            time.sleep(0.005)
            resp = ser.read(PACKET_SIZE)
            if len(resp) == PACKET_SIZE:
                # Body 0-3: enc L, 4-7: enc R (int32 little-endian)
                enc_l = struct.unpack_from("<l", resp, HEADER_SIZE + 0)[0]
                enc_r = struct.unpack_from("<l", resp, HEADER_SIZE + 4)[0]
                print(f"resp enc_l={enc_l}, enc_r={enc_r}")
            if interval > 0:
                time.sleep(interval)
    except KeyboardInterrupt:
        print("Interrupted, stopping.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
