# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""The T8132 DAPF path admits J773g only through the live-board gate."""
from pathlib import Path
import subprocess


def test_j773g_dapf_board_gate(tmp_path):
    repo = Path(__file__).resolve().parents[2]
    text = (repo / "src/dapf.c").read_text()
    start = text.index("int dapf_init_fdt(")
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
#define T8140 0x8140
struct entry { const char *path; int index; const char *alias; };
static struct entry dapf_entries[] = {{NULL, -1, NULL}};
static unsigned chip_id = T8132;
static const char *board = "apple,j713";
static bool live_j773g;
static unsigned calls;
static void *adt;
static int fdt_node_check_compatible(const void *fdt, int node, const char *compatible)
{ return fdt && node == 0 && !strcmp(board, compatible) ? 0 : 1; }
static bool kboot_is_j773g(const void *fdt) { return fdt && live_j773g; }
static int dapf_init_t8132(void *fdt) { assert(fdt); calls++; return 7; }
static int fdt_path_offset(const void *fdt, const char *path) { return -1; }
static const void *fdt_getprop(const void *fdt, int node, const char *name, int *len)
{ return NULL; }
static int adt_path_offset(const void *tree, const char *path) { return -1; }
static int dapf_init(const char *path, int index) { return -1; }
''' + production + r'''
int main(void)
{
    assert(dapf_init_fdt((void *)1) == 7 && calls == 1);
    board = "apple,j773g"; live_j773g = true;
    assert(dapf_init_fdt((void *)1) == 7 && calls == 2);
    board = "apple,j715"; live_j773g = false;
    assert(!dapf_init_fdt((void *)1) && calls == 2);
    assert(!dapf_init_fdt(NULL) && calls == 2);
    return 0;
}
'''
    source = tmp_path / "j773g-dapf-gate.c"
    source.write_text(harness)
    binary = tmp_path / "j773g-dapf-gate"
    subprocess.run(["cc", "-std=gnu11", "-Wall", "-Wextra", "-Werror",
                    "-Wno-unused-parameter", str(source), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
