# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Do not access DAPF MMIO without both a handoff request and known power."""
from pathlib import Path
import subprocess


def test_j713_dapf_power_admission(tmp_path):
    repo = Path(__file__).resolve().parents[2]
    text = (repo / "src/dapf.c").read_text()
    start = text.index("static int dapf_init_t8132(")
    production = text[start:text.index("int dapf_init_fdt(", start)]
    harness = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
static void *adt;
static int requested, power, enable_error, requests_power, dapf_calls, enables;
static int fdt_path_offset(void *dt, const char *p) { return 1; }
static void *fdt_getprop(void *dt, int n, const char *p, void *len)
{ return requested ? (void *)1 : NULL; }
static int fdt_stringlist_search(void *dt, int n, const char *p, const char *v)
{
    if (strcmp(v, "/arm-io/mtp")) return -1;
    return !strcmp(p, "linux-enablement-mac,dapf-handoff") || requests_power ? 0 : -1;
}
static int adt_path_offset(void *dt, const char *p)
{ return !strcmp(p, "/arm-io/dart-mtp") ? 1 : -1; }
static int pmgr_adt_power_enable(const char *p) { enables++; return enable_error; }
static int pmgr_adt_is_powered(const char *p) { return power; }
static int dapf_init(const char *p, int i)
{ assert(power == 1 && requested); dapf_calls++; return 0; }
''' + production + r'''
int main(void)
{
    for (requested = 0; requested <= 1; requested++)
    for (requests_power = 0; requests_power <= 1; requests_power++)
    for (power = -1; power <= 1; power++)
    for (enable_error = -1; enable_error <= 0; enable_error++) {
        dapf_calls = enables = 0;
        int ret = dapf_init_t8132((void *)1);
        bool failed = requested && requests_power && enable_error;
        assert(dapf_calls == (requested && !failed && power == 1));
        assert(enables == (requested && requests_power));
        assert((ret < 0) == failed);
    }
    return 0;
}
'''
    source = tmp_path / "dapf-power.c"
    source.write_text(harness)
    binary = tmp_path / "dapf-power"
    subprocess.run(["cc", "-std=gnu11", "-Wall", "-Wextra", "-Werror",
                    "-Wno-unused-parameter", str(source), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
