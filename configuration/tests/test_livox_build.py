"""Opt-in real CMake regression for building Livox without build.sh flags."""
import os
from pathlib import Path
import subprocess

import pytest


@pytest.mark.skipif(
    os.environ.get('CHAIRMAN_BUILD_INTEGRATION') != '1',
    reason='Requires sourced ROS 2 and installed Livox SDK2/PCL build dependencies')
def test_livox_configures_without_humble_build_script_flag(tmp_path):
    source = Path(__file__).parents[2] / 'src/fishbot/livox_ros_driver2'
    # A fresh cache must configure successfully without ROS_EDITION/HUMBLE_ROS.
    # Checking the real generate step catches missing transitive message includes.
    result = subprocess.run(
        ['cmake', '-S', str(source), '-B', str(tmp_path / 'build'),
         '-DBUILD_TESTING=OFF'],
        text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=120)
    assert result.returncode == 0, result.stdout
