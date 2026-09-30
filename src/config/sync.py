"""将中央配置原样覆盖到本工作空间对应文件。"""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import tempfile
import uuid



def workspace_root():
    for candidate in (Path(__file__).resolve(), Path.cwd()):
        for parent in (candidate, *candidate.parents):
            if (parent / 'src/config/manifest.json').is_file() and (parent / 'tool/package.xml').is_file():
                return parent.resolve()
    raise ValueError('找不到 chairman_navigation 源码工作空间；请在工程内执行并 source install/setup.bash')


def confined(root, relative):
    path = (root / relative).resolve()
    if not path.is_relative_to(root.resolve()):
        raise ValueError(f'路径越过本工程边界: {relative}')
    return path


def plan_sync(root, selections):
    manifest = json.loads((root / 'src/config/manifest.json').read_text())
    writes = {}
    for key in dict.fromkeys(selections):
        if key not in manifest:
            raise ValueError(f'未注册配置: {key}')
        source = confined(root, Path('src/config') / key)
        data = source.read_bytes()
        package, relative = key.split('/', 1)
        candidates = [confined(root, manifest[key])]
        # Support isolated and merged installs; never touch a different overlay.
        for prefix in (root / 'install' / package, root / 'install'):
            target = prefix / 'share' / package / 'config' / relative
            if target.exists() or target.is_symlink():
                candidates.append(confined(root, target.relative_to(root)))
        for target in candidates:
            if not target.is_file():
                raise ValueError(f'目标配置不存在: {target}')
            writes[target] = data
    return writes


def atomic_write(path, data):
    mode = path.stat().st_mode & 0o777
    fd, temporary = tempfile.mkstemp(prefix='.' + path.name, dir=path.parent)
    try:
        with os.fdopen(fd, 'wb') as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(temporary, mode)
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def apply_sync(root, writes):
    changed = {p: data for p, data in writes.items() if p.read_bytes() != data}
    if not changed:
        return None
    backup = root / '.configuration_backups' / (datetime.now().strftime('%Y%m%d-%H%M%S') + '-' + uuid.uuid4().hex[:8])
    originals = {path: path.read_bytes() for path in changed}
    for path, data in originals.items():
        saved = backup / path.relative_to(root)
        saved.parent.mkdir(parents=True, exist_ok=True)
        saved.write_bytes(data)
    committed = []
    try:
        for path, data in changed.items():
            atomic_write(path, data)
            committed.append(path)
    except Exception:
        for path in reversed(committed):
            atomic_write(path, originals[path])
        raise
    return backup


def main(args=None):
    parser = argparse.ArgumentParser(
        description='将 src/config 中的配置原样覆盖到对应包，自动备份原文件。',
        epilog='--files 后填写相对 src/config 的文件路径，多个文件用空格分隔。')
    selection = parser.add_mutually_exclusive_group(required=True)
    selection.add_argument('--all', action='store_true', help='覆盖 manifest.json 登记的全部配置')
    selection.add_argument('--files', nargs='+', help='覆盖指定文件，如 fast_lio/mid360.yaml odometry/regions.yaml')
    options = parser.parse_args(args)
    try:
        root = workspace_root()
        manifest = json.loads((root / 'src/config/manifest.json').read_text())
        selected = sorted(manifest) if options.all else options.files
        writes = plan_sync(root, selected)
        backup = apply_sync(root, writes)
        for path in writes:
            print(f'已同步: {path.relative_to(root)}')
        if backup:
            print(f'原文件备份: {backup.relative_to(root)}')
        print('配置覆盖完成。重启对应节点后生效。')
    except (OSError, ValueError) as exc:
        parser.exit(2, f'配置覆盖失败: {exc}\n')
