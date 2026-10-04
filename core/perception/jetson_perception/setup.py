from glob import glob
from pathlib import Path
import os

from setuptools import find_packages, setup


package_name = "jetson_perception"

asset_data_files = []
for directory in ("models", "third_party/YOLOPv2"):
    for path in sorted(Path(directory).rglob("*")):
        if path.is_file() and not any(part in {".git", "__pycache__"} for part in path.parts):
            asset_data_files.append((os.path.join("share", package_name, str(path.parent)), [str(path)]))

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "docs"), glob("docs/*.md")),
    ] + asset_data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Smart Car Team",
    maintainer_email="dev@smart-car.local",
    description="ZED camera YOLOPv2 road segmentation and traffic-sign detection.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "yolopv2_road_node = jetson_perception.yolopv2_road_node:main",
            "road_hazard_node = jetson_perception.road_hazard_node:main",
            "traffic_sign_node = jetson_perception.traffic_sign_node:main",
            "dynamic_object_tracker = jetson_perception.dynamic_object_tracker:main",
        ],
    },
)
