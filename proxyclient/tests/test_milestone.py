import struct
import pytest

from proxyclient.tools.decode_milestone import FORMAT, MAGIC, OFFSET, decode


def record(state=0x30, complement=None):
    data = bytearray(0x4000)
    if complement is None:
        complement = (~state) & 0xffffffffffffffff
    struct.pack_into(FORMAT, data, OFFSET, MAGIC, 1, state, 0x800000000, 0x1234, 4, complement)
    return bytes(data)


def test_decode_valid_record():
    assert decode(record())["state_name"] == "kernel-prepared"


def test_decode_rejects_torn_record():
    with pytest.raises(ValueError, match="complement"):
        decode(record(complement=0))
