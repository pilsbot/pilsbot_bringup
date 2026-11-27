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


def test_dir_without_marker_is_a_problem(tmp_path):
    problems = log_kohl.output_dir_problems(str(tmp_path), 0)
    assert problems == [f'no marker file {tmp_path}/.pilsbot_bags']


def test_missing_dir_is_a_problem(tmp_path):
    problems = log_kohl.output_dir_problems(str(tmp_path / 'not' / 'there'), 0)
    assert len(problems) == 1 and 'no marker file' in problems[0]


def test_marked_dir_with_space_is_fine(tmp_path):
    (tmp_path / '.pilsbot_bags').touch()
    assert log_kohl.output_dir_problems(str(tmp_path), 0) == []


def test_marker_name_is_configurable(tmp_path):
    (tmp_path / 'my_marker').touch()
    assert log_kohl.output_dir_problems(str(tmp_path), 0, 'my_marker') == []
    assert log_kohl.output_dir_problems(str(tmp_path), 0) != []


def test_low_free_space_is_a_problem_even_with_marker(tmp_path):
    (tmp_path / '.pilsbot_bags').touch()
    problems = log_kohl.output_dir_problems(str(tmp_path), 1e9)
    assert len(problems) == 1 and 'GB free' in problems[0]


def test_refusal_for_default_dir_says_to_touch_the_marker_on_the_host():
    lines = log_kohl.refusal('/workspaces/bags', '.pilsbot_bags',
                             ['no marker file /workspaces/bags/.pilsbot_bags'])
    assert any('touch /media/externSSD/bags/.pilsbot_bags' in line for line in lines)
    assert any('check_output_dir:=false' in line for line in lines)


def test_refusal_for_other_dir_names_that_dir():
    lines = log_kohl.refusal('/mnt/x/', 'm', ['no marker file /mnt/x/m'])
    assert any('touch /mnt/x/m' in line for line in lines)
    assert not any('externSSD' in line for line in lines)


def test_refusal_for_low_space_only_does_not_ask_for_the_marker():
    lines = log_kohl.refusal('/workspaces/bags', '.pilsbot_bags', ['only 1.0 GB free'])
    assert not any('touch' in line for line in lines)


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
