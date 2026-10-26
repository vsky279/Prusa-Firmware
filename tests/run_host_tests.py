"""Configure and run upstream CMake/CTest tests with a local portable compiler."""
from pathlib import Path
import subprocess, sys
r = Path(__file__).resolve().parents[1]
cmake = str(r/'.dependencies/cmake-3.22.5/bin/cmake.exe')
zig = (r/'.dependencies/host/ziglang/zig.exe').as_posix()
out = r/'build/upstream-tests'
wrappers = r/'build/host-tools'
wrappers.mkdir(parents=True,exist_ok=True)
for name,mode in [('cc','cc'),('cxx','c++'),('ar','ar'),('ranlib','ranlib')]:
    (wrappers/(name+'.cmd')).write_text('@"'+zig+'" '+mode+' %*\n')
subprocess.run([cmake,'-S',str(r),'-B',str(out),'-G','Ninja',
    '-DCMAKE_MAKE_PROGRAM='+(r/'.dependencies/ninja-1.12.1/ninja.exe').as_posix(),
    '-DCMAKE_C_COMPILER='+(wrappers/'cc.cmd').as_posix(),
    '-DCMAKE_CXX_COMPILER='+(wrappers/'cxx.cmd').as_posix(),
    '-DCMAKE_AR='+(wrappers/'ar.cmd').as_posix(),
    '-DCMAKE_RANLIB='+(wrappers/'ranlib.cmd').as_posix(),
    '-DPython3_EXECUTABLE='+sys.executable],check=True)
subprocess.run([cmake,'--build',str(out),'--target','tests','-j','4'],check=True)
subprocess.run([str(r/'.dependencies/cmake-3.22.5/bin/ctest.exe'),'--test-dir',str(out),'--output-on-failure'],check=True)
