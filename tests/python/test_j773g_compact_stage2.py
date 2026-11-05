import subprocess
from pathlib import Path


REPO = Path(__file__).resolve().parents[2]


def test_compact_profile_requires_the_esp_stage2_role():
    result = subprocess.run(
        ["make", "-n", "J773G_COMPACT_STAGE2=1", "build-cfg"],
        cwd=REPO,
        text=True,
        capture_output=True,
    )
    assert result.returncode != 0
    assert "J773G_COMPACT_STAGE2 requires ESP_STAGE2=1" in result.stderr


def test_compact_profile_keeps_dynamic_stacks_reserved():
    makefile = (REPO / "Makefile").read_text()
    linker = (REPO / "m1n1-raw.ld").read_text()
    smp = (REPO / "src/smp.c").read_text()
    kboot = (REPO / "src/kboot.c").read_text()
    start = (REPO / "src/start.S").read_text()

    assert "CFG += J773G_COMPACT_STAGE2" in makefile
    assert "--defsym=compact_stage2=1" in makefile
    assert "_section_align = DEFINED(compact_stage2) ? 0x4000 : 0x10000" in linker
    assert "_file_limit = 0x120000" in linker
    assert "#ifdef J773G_COMPACT_STAGE2\nu8 *secondary_stacks[MAX_CPUS]" in smp
    assert "secondary_stacks[index] = memalign(SZ_16K, SECONDARY_STACK_SIZE)" in smp
    assert "Match the proven Gravity lifetime" in smp
    assert "if (smp_alloc_stacks(index))" in smp
    assert "fdt_add_mem_rsv(dt, (u64)stack, SECONDARY_STACK_SIZE)" in kboot
    assert "#ifdef J773G_COMPACT_STAGE2\n    ldr x1, [x1, x3, lsl #3]" in start
