from pathlib import Path
import subprocess, sys
r = Path(__file__).resolve().parent
baseline = '--baseline' in sys.argv
variant = 'MK25-RAMBo13a' if '--upstream' in sys.argv else 'MK24-RAMBo13a'
src = r / 'baseline-source' if baseline else r
out = r / ('build-baseline-isolated' if baseline else ('build/upstream-firmware' if '--upstream' in sys.argv else 'build'))
cmake = r / '.dependencies/cmake-3.22.5/bin/cmake.exe'
args = [str(cmake), '-S', str(src), '-B', str(out), '-G', 'Ninja',
        '-DCMAKE_MAKE_PROGRAM='+str(r / '.dependencies/ninja-1.12.1/ninja.exe'),
        '-DCMAKE_TOOLCHAIN_FILE='+str(src / 'cmake/AvrGcc.cmake'),
        '-DCMAKE_BUILD_TYPE=Release', '-DFW_VARIANTS='+variant,
        '-DPython3_EXECUTABLE='+sys.executable, '-DMAIN_LANGUAGES=cs']
subprocess.run(args, check=True)
subprocess.run([str(cmake),'--build',str(out),'--target',
                variant+('_en-cs' if '--cs' in sys.argv else '_ENGLISH')],check=True)
