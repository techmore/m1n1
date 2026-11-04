# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Inline J773g payloads retain only a qualified, live ANS session."""
from pathlib import Path
import subprocess


def test_j773g_inline_ans_admission(tmp_path):
    repo = Path(__file__).resolve().parents[2]
    text = (repo / "src/payload.c").read_text()
    start = text.index("static int payload_prepare_t8132_storage(")
    opening = text.index("{", start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (text[end] == "{") - (text[end] == "}")
        end += 1
    production = text[start:end]
    harness = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#define T8132 0x8132
static unsigned chip_id = T8132;
static const char *board = "apple,j713";
static bool qualified_j773g, init_ok = true, live = true;
static unsigned inits;
static int fdt_node_check_compatible(const void *fdt, int node, const char *compatible)
{ return fdt && node == 0 && !strcmp(board, compatible) ? 0 : 1; }
static bool kboot_j773g_has_ans(const void *fdt) { return fdt && qualified_j773g; }
static bool nvme_init(void) { inits++; return init_ok; }
static bool nvme_has_live_post_m4_session(void) { return live; }
''' + production + r'''
int main(void)
{
    bool retain;
    assert(!payload_prepare_t8132_storage((void *)1, &retain));
    assert(inits == 1 && !retain);

    board = "apple,j715"; inits = 0;
    assert(!payload_prepare_t8132_storage((void *)1, &retain));
    assert(!inits && !retain);

    board = "apple,j773g"; qualified_j773g = true; inits = 0;
    assert(!payload_prepare_t8132_storage((void *)1, &retain));
    assert(inits == 1 && retain);

    live = false; inits = 0;
    assert(payload_prepare_t8132_storage((void *)1, &retain) < 0);
    assert(inits == 1 && retain);

    live = true; init_ok = false; inits = 0;
    assert(payload_prepare_t8132_storage((void *)1, &retain) < 0);
    assert(inits == 1 && retain);
    return 0;
}
'''
    source = tmp_path / "j773g-payload-ans.c"
    source.write_text(harness)
    binary = tmp_path / "j773g-payload-ans"
    subprocess.run(["cc", "-std=gnu11", "-Wall", "-Wextra", "-Werror",
                    "-Wno-unused-parameter", str(source), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
