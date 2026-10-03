import os
from glob import glob

from setuptools import find_packages, setup

package_name = "sac_planning"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "routes"), glob("routes/*")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Yunus Ahmet AKDAL",
    maintainer_email="yunus.akdal@gmail.com",
    description="Planning for the SAC car",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "route_planner = sac_planning.route_planner:main",
        ],
    },
)
