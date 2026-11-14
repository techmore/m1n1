#!/usr/bin/env python3
"""Decode the durable Aurora m1n1 milestone record from its 16 KiB page."""

import argparse
import struct
from pathlib import Path

OFFSET = 0x1000
FORMAT = "<QIIQQQQ"
MAGIC = 0x41554F5252414D31
STATES = {0x01: "proxy-prepared", 0x10: "assembly-entry", 0x20: "c-entry",
          0x30: "kernel-prepared", 0x40: "handoff"}


def decode(data: bytes) -> dict:
    size = struct.calcsize(FORMAT)
    if len(data) < OFFSET + size:
        raise ValueError(f"capture is {len(data)} bytes; need at least {OFFSET + size}")
    magic, version, state, page, detail, sequence, complement = struct.unpack_from(FORMAT, data, OFFSET)
    if magic != MAGIC:
        raise ValueError(f"bad magic 0x{magic:016x}")
    if version != 1:
        raise ValueError(f"unsupported version {version}")
    if complement != ((~state) & 0xffffffffffffffff):
        raise ValueError("torn record: state complement mismatch")
    return {"version": version, "state": state, "state_name": STATES.get(state, "unknown"),
            "page_phys": page, "detail": detail, "sequence": sequence}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("capture", type=Path)
    args = parser.parse_args()
    for key, value in decode(args.capture.read_bytes()).items():
        print(f"{key}={value:#x}" if isinstance(value, int) else f"{key}={value}")


if __name__ == "__main__":
    main()
