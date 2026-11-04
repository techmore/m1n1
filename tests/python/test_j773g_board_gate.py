# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Require the live ADT and the OS FDT to identify the same J773g board."""
from pathlib import Path
import subprocess


def test_j773g_board_gate(tmp_path):
    repo = Path(__file__).resolve().parents[2]
    text = (repo / "src/kboot.c").read_text()
    start = text.index("bool kboot_is_j773g(")
    opening = text.index("{", start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (text[end] == "{") - (text[end] == "}")
        end += 1
    storage = text.index("bool kboot_j773g_has_ans(")
    opening = text.index("{", storage)
    depth = 1
    storage_end = opening + 1
    while depth:
        depth += (text[storage_end] == "{") - (text[storage_end] == "}")
        storage_end += 1
    production = text[start:end] + "\n" + text[storage:storage_end]
    harness = r'''
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#define T8132 0x8132
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
static unsigned chip_id = T8132;
static unsigned board_id = 42;
static void *adt = (void *)1;
static bool live_j773g = true;
static const char *fdt_board = "apple,j773g";
static bool fdt_t8132 = true;
static unsigned missing_alias;
static unsigned wrong_compatible;
static bool adt_is_compatible(const void *tree, int node, const char *compatible)
{
    return tree == adt && node == 0 && live_j773g && !strcmp(compatible, "J773gAP");
}
static int fdt_node_check_compatible(const void *fdt, int node, const char *compatible)
{
    static const char *const node_compatible[] = {
        NULL, "apple,t8132-nvme-ans2", "apple,t8132-asc-mailbox", "apple,t8132-sart"
    };
    if (!fdt || node < 0 || node > 3)
        return 1;
    if (node)
        return wrong_compatible == (unsigned)node || strcmp(compatible, node_compatible[node]);
    if (!strcmp(compatible, "apple,t8132"))
        return fdt_t8132 ? 0 : 1;
    return !strcmp(compatible, fdt_board) ? 0 : 1;
}
static const char *fdt_get_alias(const void *fdt, const char *alias)
{
    static const char *const names[] = {NULL, "ans", "ans-mbox", "sart-ans"};
    static const char *const paths[] = {NULL, "/ans", "/ans-mbox", "/sart-ans"};
    for (unsigned i = 1; i < 4; i++)
        if (!strcmp(alias, names[i]))
            return missing_alias == i ? NULL : paths[i];
    return NULL;
}
static int fdt_path_offset(const void *fdt, const char *path)
{
    static const char *const paths[] = {NULL, "/ans", "/ans-mbox", "/sart-ans"};
    for (unsigned i = 1; i < 4; i++)
        if (!strcmp(path, paths[i]))
            return i;
    return -1;
}
''' + production + r'''
int main(void)
{
    assert(kboot_is_j773g((void *)1));
    assert(kboot_j773g_has_ans((void *)1));
    for (missing_alias = 1; missing_alias < 4; missing_alias++)
        assert(!kboot_j773g_has_ans((void *)1));
    missing_alias = 0;
    for (wrong_compatible = 1; wrong_compatible < 4; wrong_compatible++)
        assert(!kboot_j773g_has_ans((void *)1));
    wrong_compatible = 0;
    chip_id = 0x8130; assert(!kboot_is_j773g((void *)1)); chip_id = T8132;
    board_id = 41; assert(!kboot_is_j773g((void *)1)); board_id = 42;
    live_j773g = false; assert(!kboot_is_j773g((void *)1)); live_j773g = true;
    fdt_board = "apple,j713"; assert(!kboot_is_j773g((void *)1));
    fdt_board = "apple,j773g";
    fdt_t8132 = false; assert(!kboot_is_j773g((void *)1)); fdt_t8132 = true;
    assert(!kboot_is_j773g(NULL));
    return 0;
}
'''
    source = tmp_path / "j773g-board.c"
    source.write_text(harness)
    binary = tmp_path / "j773g-board"
    subprocess.run(["cc", "-std=gnu11", "-Wall", "-Wextra", "-Werror",
                    "-Wno-unused-parameter", str(source), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
