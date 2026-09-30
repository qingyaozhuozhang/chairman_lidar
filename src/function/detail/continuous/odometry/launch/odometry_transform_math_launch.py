# Compatibility alias for the original launch command.
import runpy
from pathlib import Path

def generate_launch_description():
    return runpy.run_path(str(Path(__file__).with_name('odometry.launch.py')))['generate_launch_description']()
