#!/usr/bin/env python3
"""Package and verify the model/map files distributed outside Git."""
import argparse
import hashlib
import io
from pathlib import Path, PurePosixPath
import shutil
import stat
import tarfile

ASSET_ROOTS = (
    'core/perception/jetson_perception/models',
    'utilities/smart_car_launch/maps',
    'core/localization/sac_localization/maps',
    'core/localization/autoware/llh_converter/data',
)


def digest(stream):
    value = hashlib.sha256()
    for block in iter(lambda: stream.read(1024 * 1024), b''):
        value.update(block)
    return value.hexdigest()


def pack(repo, archive):
    for required in ASSET_ROOTS[:2]:
        if not any(p.is_file() for p in (repo / required).rglob('*')):
            raise ValueError(f'Asset directory missing or empty: {required}')
    paths = sorted(p for root in ASSET_ROOTS for p in (repo / root).rglob('*')
                   if p.is_file() and '.git' not in p.parts)
    checksums = []
    for path in paths:
        with path.open('rb') as stream:
            checksums.append(f'{digest(stream)}  {path.relative_to(repo).as_posix()}\n')
    metadata = {
        'ASSETS.files': ''.join(p.relative_to(repo).as_posix() + '\n' for p in paths),
        'SHA256SUMS': ''.join(checksums),
    }
    archive.parent.mkdir(parents=True, exist_ok=True)
    temporary = archive.with_suffix(archive.suffix + '.tmp')
    try:
        with tarfile.open(temporary, 'w') as output:
            for path in paths:
                member = tarfile.TarInfo(path.relative_to(repo).as_posix())
                member.size = path.stat().st_size
                member.mode = 0o644
                with path.open('rb') as stream:
                    output.addfile(member, stream)
            for name, content in metadata.items():
                content = content.encode()
                member = tarfile.TarInfo(name)
                member.size = len(content)
                member.mode = 0o644
                output.addfile(member, io.BytesIO(content))
        verify(temporary)
        temporary.replace(archive)
    finally:
        temporary.unlink(missing_ok=True)
    with archive.open('rb') as stream:
        checksum = digest(stream)
    archive.with_suffix(archive.suffix + '.sha256').write_text(
        f'{checksum}  {archive.name}\n')
    print(f'Archive ready: {archive} ({len(paths)} files)')


def safe_name(name):
    path = PurePosixPath(name)
    return (not path.is_absolute() and '..' not in path.parts
            and name.startswith(tuple(root + '/' for root in ASSET_ROOTS)))


def verify(archive):
    with tarfile.open(archive) as source:
        members = source.getmembers()
        names = [m.name for m in members]
        if len(names) != len(set(names)) or any(not m.isfile() for m in members):
            raise ValueError('Archive contains duplicate names or non-file entries')
        sums = source.extractfile('SHA256SUMS').read().decode().splitlines()
        expected = {}
        for line in sums:
            checksum, name = line.split('  ', 1)
            if not safe_name(name) or name in expected:
                raise ValueError(f'Invalid asset path: {name}')
            expected[name] = checksum
        listed = source.extractfile('ASSETS.files').read().decode().splitlines()
        if set(listed) != set(expected) or set(names) != set(expected) | {'ASSETS.files', 'SHA256SUMS'}:
            raise ValueError('Archive file list and checksums disagree')
        for name, checksum in expected.items():
            if digest(source.extractfile(name)) != checksum:
                raise ValueError(f'SHA-256 mismatch: {name}')
    print(f'SHA-256 verified: {len(expected)} files')
    return expected


def extract(archive, repo):
    expected = verify(archive)
    repo = repo.resolve()
    with tarfile.open(archive) as source:
        for name in expected:
            target = repo / name
            if not target.resolve().is_relative_to(repo):
                raise ValueError(f'Destination points outside repository: {name}')
            target.parent.mkdir(parents=True, exist_ok=True)
            temporary = target.with_name(target.name + '.download')
            try:
                with source.extractfile(name) as stream, temporary.open('wb') as output:
                    shutil.copyfileobj(stream, output)
                if target.exists():
                    temporary.chmod(stat.S_IMODE(target.stat().st_mode))
                temporary.replace(target)
            finally:
                temporary.unlink(missing_ok=True)
    print(f'Assets installed: {repo}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['pack', 'verify', 'extract'])
    parser.add_argument('archive', type=Path)
    parser.add_argument('--repo', type=Path, default=Path(__file__).resolve().parents[2])
    args = parser.parse_args()
    try:
        if args.action == 'pack':
            pack(args.repo, args.archive)
        elif args.action == 'verify':
            verify(args.archive)
        else:
            extract(args.archive, args.repo)
    except (ValueError, KeyError, OSError, tarfile.TarError) as error:
        parser.exit(1, f'ERROR: {error}\n')
