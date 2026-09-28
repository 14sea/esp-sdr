#!/usr/bin/env python3
"""Export one ESP-IDF build as a versioned artifact for esp-web-sdr (stdlib only)."""
import argparse
import hashlib
import json
from pathlib import Path
import re


def export(build, output, board, label, version, allow_larger_flash=False):
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]*', board):
        raise ValueError('Board ID must contain only letters, numbers, dots, underscores or hyphens')
    cache = (build / 'CMakeCache.txt').read_text()
    for line in cache.splitlines():
        if re.match(r'(SAMPLE_RATE_PROBE|FILTER_REGISTER_PROBE|S3_RF_PROBE|C5_TUNE_PROBE|C5_REPLAY_PROBE):', line):
            if line.rsplit('=', 1)[-1].upper() not in ('OFF', 'FALSE', '0', 'NO', ''):
                raise ValueError('Refusing to export diagnostic probe firmware')
    config = json.loads((build / 'config/sdkconfig.json').read_text())
    args = json.loads((build / 'flasher_args.json').read_text())
    target = config['IDF_TARGET']
    if args['extra_esptool_args']['chip'] != target:
        raise ValueError('Build target differs from flashing target')
    if not re.fullmatch(r'esp32[a-z0-9]*', target):
        raise ValueError('Expected an ESP32 target')
    settings = args['flash_settings']
    size = re.fullmatch(r'(\d+)MB', settings['flash_size'])
    if not size:
        raise ValueError('Build must specify a concrete flash size in MB')
    limit = int(size[1]) * 1024 * 1024
    parts, payloads = [], []
    end = 0
    for i, (address, name) in enumerate(sorted(args['flash_files'].items(), key=lambda p: int(p[0], 0))):
        source = (build / name).resolve()
        if not source.is_relative_to(build.resolve()):
            raise ValueError('Flash image must be inside the build directory')
        data = source.read_bytes()
        offset = int(address, 0)
        if not data or offset < end or offset + len(data) > limit:
            raise ValueError('Empty, overlapping or out-of-flash image')
        end = offset + len(data)
        filename = f'{i}-{source.name}'
        parts.append(dict(name=filename, offset=offset, size=len(data),
                          sha256=hashlib.sha256(data).hexdigest(), md5=hashlib.md5(data).hexdigest()))
        payloads.append((filename, data))
    if not parts:
        raise ValueError('No firmware images in flasher_args.json')
    chip = 'ESP32' + ('-' + target[5:].upper() if target[5:] else '')
    manifest = dict(schema_version=1, version=version, variants={board: dict(
        revision=board, label=label, target=target, chip=chip, version=version,
        flash_size=settings['flash_size'], flash_size_policy='minimum' if allow_larger_flash else 'exact',
        flash_settings=settings, parts=parts)})
    # Each matrix job writes a separate artifact directory; never merge in place.
    output.mkdir(parents=True, exist_ok=False)
    images = output / board
    images.mkdir()
    for name, data in payloads:
        (images / name).write_bytes(data)
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    return output / 'manifest.json'


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--board', required=True, help='Unique board/revision ID, e.g. esp32c5')
    parser.add_argument('--label', required=True, help='Human-readable board name')
    parser.add_argument('--version', required=True, help='Firmware source revision or release tag')
    parser.add_argument('--allow-larger-flash', action='store_true',
                        help='Allow this fixed flash layout on chips with larger flash')
    a = parser.parse_args()
    print(export(a.build_dir, a.output, a.board, a.label, a.version, a.allow_larger_flash))
