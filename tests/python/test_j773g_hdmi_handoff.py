# SPDX-License-Identifier: MIT
"""Mac mini M4 (J773g) HDMI handoff: the dcpext0 display pipe.

Builds tests/j773g_hdmi_host.c with the tree's src/kboot_display_j773g.c, the host stand-in
headers of tests/t8122_display_host/ (tests/j773g_hdmi_host/ over them) and m1n1's libfdt (ASan,
UBSan), and runs every scenario: a J773g boot whose device tree describes dcpext0 publishes the
contract the kernel's mini/hdmi branch expects and locks both display DARTs; another chip or board,
or a device tree without dcpext0, gets nothing at all; every failed check publishes nothing but the
refusal in /chosen and writes no register. The ADT, the MMIO and the device tree are synthetic
(addresses and segments are the J773g ADT's); no hardware is exercised.
"""
from pathlib import Path
import re
import shutil
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[2]
HOST = ROOT / "tests/j773g_hdmi_host.c"
LIBFDT = ["fdt.c", "fdt_ro.c", "fdt_rw.c", "fdt_wip.c", "fdt_sw.c", "fdt_empty_tree.c",
          "fdt_strerror.c", "fdt_addresses.c", "fdt_check.c"]


@pytest.fixture(scope="module")
def host(tmp_path_factory):
    cc = shutil.which("cc")
    if not cc:
        pytest.skip("no host C compiler")
    tmp = tmp_path_factory.mktemp("j773g")
    src = tmp / "src"
    src.mkdir()
    # The code under test next to the stand-ins, so that its #include "utils.h" finds them.
    shutil.copy(ROOT / "src/kboot_display_j773g.c", src)
    shutil.copy(HOST, src)
    for header in (ROOT / "tests/t8122_display_host").glob("*.h"):
        shutil.copy(header, src)
    for header in (ROOT / "tests/j773g_hdmi_host").glob("*.h"):
        shutil.copy(header, src)
    binary = tmp / "j773g-hdmi-host"
    subprocess.run(
        [cc, "-std=gnu11", "-D_GNU_SOURCE", "-O1", "-g", "-Wall", "-Wno-unused-function",
         "-Wno-format", "-Wno-format-truncation", "-fsanitize=address,undefined",
         "-fno-omit-frame-pointer", "-fno-sanitize-recover=undefined",
         "-I", str(src), "-I", str(ROOT / "src"),
         str(src / "j773g_hdmi_host.c"), str(src / "kboot_display_j773g.c"),
         *(str(ROOT / "src/libfdt" / f) for f in LIBFDT), "-o", str(binary)],
        check=True,
    )
    return binary


def run(binary, scenario):
    env = {"ASAN_OPTIONS": "detect_leaks=0", "PATH": "/usr/bin:/bin"}
    return subprocess.run([str(binary), scenario], capture_output=True, text=True, env=env)


SCENARIOS = re.findall(r'^    \{"([a-z0-9-]+)", (?:PUBLISH|NOTHING|REFUSE|REFUSE_NO_MMIO)\b',
                       HOST.read_text(), re.M)

PUBLISHED = "dcpext0 firmware 81BAF476-679D-3CC0-8C6A-E6B6FA659D5E handed over"
NOT_DESCRIBED = "the device tree does not describe dcpext0: boot framebuffer only"
# The log line of the check each scenario is meant to reach, so that none passes for another reason
REASONS = {
    "publish": PUBLISHED,
    "publish-prelocked": PUBLISHED,
    "publish-disp-heap": PUBLISHED,
    "publish-dcp-vram": PUBLISHED,
    "publish-piodma-empty": PUBLISHED,
    "publish-oslog-in-memreserve": PUBLISHED,
    "repeat": "handoff already published",
    "wrong-chip": None,
    "wrong-board": None,
    "old-dt": NOT_DESCRIBED,
    "wrong-dcp-compat": NOT_DESCRIBED,
    "dcp-enabled": 'dcpext0 is not status "disabled"',
    "dart-enabled": 'dart-dispext0 is not status "disabled"',
    "wrong-sid": "dcpext0 iommus is not <&dart 23>",
    "wrong-dart-reg": "dart-dispext0 is not the apple,t8110-dart at 0x286300000",
    "no-mbox": "dcpext0 mailbox missing",
    "no-piodma": "dcpext0 has no piodma child",
    "no-disp-alias": "alias disp0 is not an apple,display-subsystem node",
    "has-memory-region": "dcpext0 already has a memory-region",
    "wrong-os": "OS firmware 14.7 is not 26.6.2",
    "bad-uuid": "dcpext0 runs another firmware image",
    "adt-not-running": "iBoot did not leave the dcpext0 firmware loaded and running",
    "no-nub": "no /arm-io/dcpext0/iop-dcpext0-nub",
    "no-adt-dart": "ADT /arm-io/dart-dispext0 is not the dart,t8110",
    "seg-flags": "dcpext0 segment 1 is not data",
    "text-in-ram": "text 0x100002a8000+0x808000 lies in the RAM handed to the OS",
    "fb-outside-vram": "is not in /vram",
    "carveout": "data 0x103e4c6c000+0x328000 overlaps memory carveout",
    "power-off": "dispext0 is not powered",
    "cpu-stopped": "dcpext0 is not running",
    "disp-ttbr-invalid": "display SID 0: TCR 0x1 TTBR 0x0 is not a live two-level translation",
    "disp-no-fb": "display SID 0 does not scan out the boot framebuffer",
    "stray-map": "outside the firmware segments and /vram",
    "oslog-mapped": "the __OS_LOG buffer",
    "text-wrong-dva": "dcpext0 SID 23 maps text at DVA 0x1c48000, the ADT says 0x1c4c000",
    "text-partial": "dcpext0 SID 23 maps 0x100002a8000+0x804000 of text",
    "pt-outside": "L2 table 0x103f0000000 outside pt-region",
    "resv-conflict": "framebuffer 0x103e5300000+0x854000 overlaps /reserved-memory/other@0",
    "oslog-memreserve-straddle": "os_log 0x10003f30000+0x24000 overlaps /memreserve/",
    "dt-full": "device tree update failed (full?); nothing published",
    "lock-fails": "locking dart-dcpext0 failed",
}


def test_scenario_list_matches(host):
    out = subprocess.run([str(host), "list"], check=True, capture_output=True, text=True)
    assert out.stdout.split() == SCENARIOS
    assert sorted(REASONS) == sorted(SCENARIOS)


@pytest.mark.parametrize("scenario", SCENARIOS)
def test_j773g_scenario(host, scenario):
    result = run(host, scenario)
    assert result.returncode == 0, result.stdout + result.stderr
    if REASONS[scenario]:
        assert REASONS[scenario] in result.stdout, result.stdout
    if REASONS[scenario] != PUBLISHED and scenario != "repeat":
        assert PUBLISHED not in result.stdout, result.stdout
    assert result.stdout.rstrip().endswith(f"{scenario}: pass"), result.stdout
    assert "FAIL" not in result.stdout, result.stdout


@pytest.mark.parametrize("scenario", ["wrong-chip", "wrong-board", "old-dt", "wrong-dcp-compat"])
def test_other_boards_and_old_trees_get_nothing(host, scenario):
    out = run(host, scenario).stdout
    assert "nothing done: the device tree is byte for byte unchanged" in out
    assert "nothing done: no MMIO access at all" in out


def test_publish_contract(host):
    out = run(host, "publish").stdout
    for line in ("dart-dcpext0 locked", "dart-dispext0 locked",
                 "dcpext0 apple,firmware-uuid = 81BAF476-679D-3CC0-8C6A-E6B6FA659D5E",
                 "dcpext0 memory-region = <text data heap>",
                 "display memory-region = <framebuffer>", "piodma memory-region = <heap>",
                 "/soc/display-subsystem status okay", "/soc/mailbox@286e08000 status okay"):
        assert "ok   " + line in out, out


def test_kboot_reaches_j773g_only_on_j773g():
    """dt_set_display() sends only an apple,j773g root to the J773g handoff; J713 is unchanged."""
    source = (ROOT / "src/kboot.c").read_text()
    start = source.index("static int dt_set_display(void)")
    body = source[start:source.index("\n}\n", start)]
    t8132 = body[body.index("if (chip_id == T8132) {"):]
    t8132 = t8132[:t8132.index("return 0;\n    }") + len("return 0;\n    }")]
    assert re.search(r'if \(!fdt_node_check_compatible\(dt, 0, "apple,j773g"\)\)\s*'
                     r'return dt_set_j773g_hdmi_handoff\(dt\);', t8132)
    assert t8132.index("dt_set_j773g_hdmi_handoff") < t8132.index("dt_set_j713_dcp_handoff()")
    assert "kboot_display_j773g.o" in (ROOT / "Makefile").read_text()


def test_handoff_is_gated_on_chip_board_and_tree():
    source = (ROOT / "src/kboot_display_j773g.c").read_text()
    entry = source[source.index("int dt_set_j773g_hdmi_handoff(void *dt)\n{"):]
    gate = entry[:entry.index("handoff(dt, dcp)")]
    assert 'chip_id != T8132 || fdt_node_check_compatible(dt, 0, "apple,j773g")' in gate
    assert 'fdt_get_alias(dt, "dcpext0")' in gate
    assert '"apple,t8132-dcpext"' in gate
    # no register access and no device tree edit before those gates
    assert "read32" not in gate and "write32" not in gate and "fdt_setprop" not in gate
