# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Host-only check of J773g PMP metadata inspection admission."""
from pathlib import Path
import subprocess
import tempfile


def test_j773g_pmp_inspection(tmp_path):
    repo = Path(__file__).resolve().parents[2]
    source = (repo / 'src/kboot.c').read_text()
    start = source.index('static const char *excluded_pmp_props[]')
    end = source.index('static int dt_set_sep(void)', start)
    production = source[start:end]
    harness = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "libfdt.h"
typedef uint32_t u32;
#define T8132 0x8132
static unsigned chip_id = T8132;
static void *dt, *adt;
struct adt_prop { const char *name; const void *value; unsigned size; };
static const unsigned raw_table[] = {0x1234, 0x5678};
static struct adt_prop props[] = {{"soc-device",raw_table,sizeof(raw_table)}, {NULL,NULL,0}};
#define ADT_FOREACH_PROPERTY(a,n,p) for (struct adt_prop *p=props; p->name; p++)
#define bail(...) return -1
static int adt_path_offset(void *a, const char *p) { return strstr(p,"chosen") ? 1 : 2; }
static int adt_word(void *a, int n, const char *p, u32 *v) { *v=0x1234; return sizeof(*v); }
#define ADT_GETPROP(a,n,p,v) adt_word(a,n,p,v)
''' + production + r'''
int main(void) {
 for (int mode=0;mode<6;mode++) {
  char buf[16384]; assert(!fdt_create_empty_tree(buf,sizeof(buf)));dt=buf;
  assert(!fdt_setprop_string(dt,0,"compatible",mode==0?"apple,j713":"apple,j773g"));
  int n=fdt_add_subnode(dt,0,"pmp");assert(n>=0);
  assert(!fdt_setprop_string(dt,n,"status",mode==4?"okay":"disabled"));
  n=fdt_path_offset(dt,"/pmp");
  if(mode==2||mode==4)assert(!fdt_setprop_empty(dt,n,"apple,j773g-pmp-inspect-only"));
  if(mode==3)assert(!fdt_setprop_u32(dt,n,"apple,j773g-pmp-inspect-only",1));
  int aliases=fdt_add_subnode(dt,0,"aliases");assert(aliases>=0);
  if(mode!=5)assert(!fdt_setprop_string(dt,aliases,"pmp","/pmp"));
  int result=dt_set_pmp(); n=fdt_path_offset(dt,"/pmp");int len;
  const char *status=fdt_getprop(dt,n,"status",&len);
  const void *table=fdt_getprop(dt,n,"apple,tunable-soc-device",NULL);
  if(mode==0){assert(!result&&status&&!strcmp(status,"okay")&&table);}
  if(mode==1||mode==5){assert(!result&&status&&!strcmp(status,"disabled")&&!table);}
  if(mode==2){assert(!result&&status&&!strcmp(status,"disabled")&&table);}
  if(mode==3){assert(result<0&&status&&!strcmp(status,"disabled")&&!table);}
  if(mode==4){assert(result<0&&status&&!strcmp(status,"okay")&&!table);}
 }
 puts("J773g PMP inspection remains disabled; J713 unchanged; malformed inputs refused");
 return 0;
}
'''
    file = tmp_path / 'pmp.c'
    file.write_text(harness)
    libfdt = repo / 'src/libfdt'
    names = ['fdt', 'fdt_ro', 'fdt_rw', 'fdt_sw', 'fdt_wip', 'fdt_empty_tree',
             'fdt_check', 'fdt_strerror', 'fdt_addresses']
    binary = tmp_path / 'pmp'
    subprocess.run(['cc', '-D_GNU_SOURCE', '-std=gnu11', '-Wall', '-Wextra', '-Werror',
                    '-Wno-unused-parameter', '-I', str(libfdt), str(file),
                    *[str(libfdt / (n + '.c')) for n in names], '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    with tempfile.TemporaryDirectory(prefix='j773g-pmp-inspect-') as tmp:
        test_j773g_pmp_inspection(Path(tmp))
