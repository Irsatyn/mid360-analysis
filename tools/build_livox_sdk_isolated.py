#!/usr/bin/env python3
"""Build a local Livox-SDK2 shared library exporting only its public C API.

Keeps bundled spdlog symbols out of the ROS process's global symbol lookup.
Does not edit SDK sources or install anything into system directories.
"""
import argparse
from pathlib import Path
import re
import shutil
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--prefix', type=Path, required=True)
    args = parser.parse_args()
    source, prefix = args.source.resolve(), args.prefix.resolve()
    if source == prefix or source in prefix.parents or prefix in source.parents:
        parser.error('source and prefix must be separate directories')
    header = source / 'include/livox_lidar_api.h'
    if not header.is_file() or not (source / 'CMakeLists.txt').is_file():
        parser.error('source must be a Livox-SDK2 checkout')
    text = re.sub(r'/\*.*?\*/|//[^\n]*', '', header.read_text(), flags=re.S)
    names = sorted(set(re.findall(
        r'^\s*(?:[A-Za-z_]\w*[ \t*]+)+([A-Za-z_]\w*)[ \t]*\(', text, re.M)))
    required = {'GetLivoxLidarSdkVer', 'LivoxLidarSdkInit', 'LivoxLidarSdkUninit'}
    if not required.issubset(names):
        parser.error('cannot identify the SDK public C API')
    build = prefix / 'build'
    build.mkdir(parents=True, exist_ok=True)
    exports = build / 'livox_sdk_exports.map'
    exports.write_text('{\n  global:\n' + ''.join(f'    {name};\n' for name in names)
                       + '  local: *;\n};\n')
    subprocess.run([
        'cmake', '-S', str(source), '-B', str(build), '-DCMAKE_BUILD_TYPE=Release',
        '-DCMAKE_SHARED_LINKER_FLAGS=-Wl,--version-script=' + str(exports),
    ], check=True)
    subprocess.run(['cmake', '--build', str(build), '--target', 'livox_lidar_sdk_shared',
                    '--parallel', '2'], check=True)
    library = build / 'sdk_core/liblivox_lidar_sdk_shared.so'
    symbols = subprocess.check_output(['nm', '-D', '--defined-only', str(library)], text=True)
    exported = {line.split()[-1] for line in symbols.splitlines() if line.split()}
    if exported != set(names):
        raise RuntimeError(f'Unexpected SDK exports: missing={set(names) - exported}, '
                           f'extra={exported - set(names)}')
    (prefix / 'lib').mkdir(exist_ok=True)
    (prefix / 'include').mkdir(exist_ok=True)
    shutil.copy2(library, prefix / 'lib' / library.name)
    for name in ('livox_lidar_api.h', 'livox_lidar_def.h', 'livox_lidar_cfg.h'):
        shutil.copy2(source / 'include' / name, prefix / 'include' / name)
    print(f'Isolated SDK: {prefix / "lib" / library.name} ({len(names)} public C APIs)')


if __name__ == '__main__':
    main()
