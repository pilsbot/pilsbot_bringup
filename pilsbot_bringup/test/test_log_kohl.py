"""Tests for the helpers of log_kohl.launch.py."""
from datetime import datetime
import importlib.util
from pathlib import Path

import yaml

PACKAGE = Path(__file__).resolve().parents[1]


def load_launch_file():
    spec = importlib.util.spec_from_file_location(
        'log_kohl', PACKAGE / 'launch' / 'log_kohl.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


log_kohl = load_launch_file()


def test_bag_names_differ_even_with_the_same_clock():
    now = datetime(1970, 1, 1)
    a, b = log_kohl.bag_name(now), log_kohl.bag_name(now)
    assert a != b
    assert a.startswith('kohl_1970_01_01-00_00_00_')


def test_all_groups_record_rosout_and_static_data():
    config = yaml.safe_load((PACKAGE / 'config' / 'recording.yaml').read_text())
    cmd = log_kohl.record_command(config, 'all', '/bags/x')
    for topic in ('/rosout', '/tf_static', '/robot_description'):
        assert topic in cmd


def test_dir_on_a_system_filesystem_is_a_problem(tmp_path):
    problems = log_kohl.output_dir_problems(
        tmp_path / 'not' / 'yet' / 'there', 0, system_paths=(str(tmp_path),))
    assert len(problems) == 1 and 'same filesystem' in problems[0]


def test_low_free_space_is_a_problem(tmp_path):
    problems = log_kohl.output_dir_problems(tmp_path, 1e9, system_paths=())
    assert len(problems) == 1 and 'GB free' in problems[0]


def test_dir_on_its_own_filesystem_with_space_is_fine(tmp_path):
    assert log_kohl.output_dir_problems(tmp_path, 0, system_paths=()) == []


class Exit:
    returncode = 1


class Context:
    def __init__(self, is_shutdown):
        self.is_shutdown = is_shutdown


def test_recorder_exit_is_reported_loudly():
    [log] = log_kohl.recorder_exited(Exit(), Context(is_shutdown=False))
    assert 'RECORDER EXITED (return code 1)' in log.msg[0].text


def test_recorder_exit_on_shutdown_is_quiet():
    [log] = log_kohl.recorder_exited(Exit(), Context(is_shutdown=True))
    assert '!!' not in log.msg[0].text
