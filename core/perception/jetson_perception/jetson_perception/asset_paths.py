"""Resolve model/config paths relative to the installed ROS package."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory


def asset_path(value):
    path = Path(str(value)).expanduser()
    if not path.is_absolute():
        path = Path(get_package_share_directory('jetson_perception')) / path
    return str(path)
