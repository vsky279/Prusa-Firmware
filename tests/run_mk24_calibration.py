"""Compile the actual AVR fitting source on the host; no Python solver copy.

Usage: python tests/run_mk24_calibration.py [path/to/c++compiler]
The local bootstrap compiler fallback is .dependencies/host/ziglang/zig.exe.
"""
from pathlib import Path
import re, subprocess, sys
r = Path(__file__).resolve().parents[1]
s = (r/'Firmware/mesh_bed_calibration.cpp').read_text()
def function(name, source=None):
    source = s if source is None else source
    start = source.index(name)
    start = source.rfind('\n',0,start)+1
    end = source.index('\n}',start)+2
    return source[start:end]
prefix = '''#include <cmath>
#include <cstdint>
#include <initializer_list>
using namespace std;
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define MK24_MISSING_XYZ_POINT 2
#define HEATBED_V2
#define Y_MIN_POS -2.2f
#define pgm_read_float(p) (*(p))
#define DBG(...) ((void)0)
#define SERIAL_ECHOLNPGM(...) ((void)0)
#define SERIAL_ECHO(...) ((void)0)
#define SERIAL_ECHOPGM(...) ((void)0)
struct SerialStub { template<class... T> void print(T...) {} template<class... T> void println(T...) {} } MYSERIAL;
#define EEPROM_XYZ_CAL_SKEW 0
static int writes=0;
void eeprom_update_float_notify(float*,float) { ++writes; }
void delay_keep_alive(int) {}
enum BedSkewOffsetDetectionResultType { BED_SKEW_OFFSET_DETECTION_POINT_NOT_FOUND=-1,
BED_SKEW_OFFSET_DETECTION_POINT_SCAN_FAILED=-3, BED_SKEW_OFFSET_DETECTION_FITTING_FAILED=-2,
BED_SKEW_OFFSET_DETECTION_PERFECT=0, BED_SKEW_OFFSET_DETECTION_SKEW_MILD=1,
BED_SKEW_OFFSET_DETECTION_SKEW_EXTREME=2 };
'''
constants = '\n'.join(line for line in s.splitlines() if line.startswith(('#define MACHINE_AXIS_SCALE_', '#define BED_CALIBRATION_POINT_OFFSET_', '#define WEIGHT_FIRST_ROW_', '#define Y_MIN_POS_CALIBRATION_','const float bed_skew_angle_')))
parts = [prefix,constants]
for name in ['static inline bool xyz_reference_missing', 'static inline float sqr', 'static inline bool point_on_1st_row', 'static inline float point_weight_x','static inline float point_weight_y', 'BedSkewOffsetDetectionResultType calculate_machine_skew_and_offset_LS(']:
    # sqr is a one-line function, so keep only that line.
    parts.append(s[s.index(name):].splitlines()[0] if name.endswith('sqr') else function(name))
parts.append((r/'tests/mk24_scan_stubs.h').read_text())
parts.append(function('BedSkewOffsetDetectionResultType find_bed_offset_and_skew('))
parts.append((r/'tests/fixtures/mk24_original_fit.inc').read_text())
parts.append((r/'tests/mk24_calibration_test.cpp').read_text())
out = r/'build/mk24-tests'; out.mkdir(parents=True,exist_ok=True)
src = out/'fitter.cpp'; src.write_text('\n'.join(parts))
compiler = [sys.argv[1]] if len(sys.argv)>1 else [str(r/'.dependencies/host/ziglang/zig.exe'),'c++']
for diagnostic in [False, True]:
    exe = out/('fitter-diagnostic.exe' if diagnostic else 'fitter.exe')
    flags = ['-DSUPPORT_VERBOSITY'] if diagnostic else []
    subprocess.run(compiler+['-std=c++17','-O0']+flags+[str(src),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
# Ensure guards precede all four-reference movement loops, including diagnostics.
for match in re.finditer(r'for \((?:int k|int8_t mesh_point) = 0; (?:k|mesh_point) < 4;[^\n]*\) \{',s):
    assert 'xyz_reference_missing' in s[match.end():match.end()+100]
assert 'if (retries == 1)\n            return BED_SKEW_OFFSET_DETECTION_POINT_SCAN_FAILED;' in s
print('XYZ reference motion guards and retry exhaustion checks passed')
