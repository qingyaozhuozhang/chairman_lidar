import importlib.util
import json
from pathlib import Path

import pytest

PATH = Path(__file__).parents[2] / 'src/config/sync.py'
spec = importlib.util.spec_from_file_location('sync', PATH)
sync = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sync)


def workspace(tmp_path):
    (tmp_path / 'src/config/fast_lio').mkdir(parents=True)
    (tmp_path / 'src/fishbot/fast_lio/config').mkdir(parents=True)
    manifest = {}
    for name in ('one.yaml', 'two.yaml'):
        (tmp_path / 'src/config/fast_lio' / name).write_text('speed: 2.0\n')
        target = 'src/fishbot/fast_lio/config/' + name
        (tmp_path / target).write_text('speed: 1.0\n')
        manifest['fast_lio/' + name] = target
    (tmp_path / 'src/config/manifest.json').write_text(json.dumps(manifest))
    return tmp_path


def test_selection_and_backup(tmp_path):
    root = workspace(tmp_path)
    backup = sync.apply_sync(root, sync.plan_sync(root, ['fast_lio/one.yaml']))
    target = Path('src/fishbot/fast_lio/config/one.yaml')
    assert (root / target).read_text() == 'speed: 2.0\n'
    assert (backup / target).read_text() == 'speed: 1.0\n'
    assert (root / 'src/fishbot/fast_lio/config/two.yaml').read_text() == 'speed: 1.0\n'


def test_copies_content_without_yaml_validation(tmp_path):
    root = workspace(tmp_path)
    (root / 'src/config/fast_lio/two.yaml').write_text('bad: [')
    sync.apply_sync(root, sync.plan_sync(root, ['fast_lio/two.yaml']))
    assert (root / 'src/fishbot/fast_lio/config/two.yaml').read_text() == 'bad: ['


def test_unknown_selection_does_not_write_partial_batch(tmp_path):
    root = workspace(tmp_path)
    with pytest.raises(ValueError):
        sync.plan_sync(root, ['fast_lio/one.yaml', 'missing.yaml'])
    assert (root / 'src/fishbot/fast_lio/config/one.yaml').read_text() == 'speed: 1.0\n'


@pytest.mark.parametrize('args,changed', [(['--all'], ['one.yaml', 'two.yaml']),
                                        (['--files', 'fast_lio/two.yaml'], ['two.yaml'])])
def test_only_two_sync_modes(tmp_path, monkeypatch, args, changed):
    root = workspace(tmp_path)
    monkeypatch.setattr(sync, 'workspace_root', lambda: root)
    sync.main(args)
    for name in ('one.yaml', 'two.yaml'):
        expected = '2.0' if name in changed else '1.0'
        assert (root / 'src/fishbot/fast_lio/config' / name).read_text() == f'speed: {expected}\n'


@pytest.mark.parametrize('args', [[], ['--all', '--files', 'fast_lio/one.yaml'], ['--dry-run'], ['--list'], ['--packages', 'fast_lio']])
def test_old_or_conflicting_arguments_are_rejected(args):
    with pytest.raises(SystemExit) as error:
        sync.main(args)
    assert error.value.code == 2


def test_symlink_cannot_escape_workspace(tmp_path):
    root = workspace(tmp_path / 'workspace')
    outside = tmp_path / 'outside.yaml'
    outside.write_text('speed: 9.0\n')
    target = root / 'src/fishbot/fast_lio/config/one.yaml'
    target.unlink()
    target.symlink_to(outside)
    with pytest.raises(ValueError):
        sync.plan_sync(root, ['fast_lio/one.yaml'])
    assert outside.read_text() == 'speed: 9.0\n'


def test_installed_config_and_source_both_synchronized(tmp_path):
    root = workspace(tmp_path)
    installed = root / 'install/fast_lio/share/fast_lio/config/one.yaml'
    installed.parent.mkdir(parents=True)
    installed.write_text('speed: 0.0\n')
    sync.apply_sync(root, sync.plan_sync(root, ['fast_lio/one.yaml']))
    assert installed.read_text() == 'speed: 2.0\n'


def test_failed_write_restores_previous_files(tmp_path, monkeypatch):
    root = workspace(tmp_path)
    original_write = sync.atomic_write
    count = 0

    def fail_second(path, data):
        nonlocal count
        count += 1
        if count == 2:
            raise OSError('disk error')
        original_write(path, data)

    monkeypatch.setattr(sync, 'atomic_write', fail_second)
    with pytest.raises(OSError):
        sync.apply_sync(root, sync.plan_sync(root, ['fast_lio/one.yaml', 'fast_lio/two.yaml']))
    assert (root / 'src/fishbot/fast_lio/config/one.yaml').read_text() == 'speed: 1.0\n'
