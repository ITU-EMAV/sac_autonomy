from setuptools import find_packages, setup
import os
from glob import glob


def files_in(pattern):
    # Sadece dosyalari dondur: __pycache__ gibi dizinler setuptools'u kirar.
    return [p for p in glob(pattern) if os.path.isfile(p)]

package_name = "sac_trajectory_planning"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join('share', package_name, 'launch'), files_in('launch/*')),
        
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="nuke",
    maintainer_email="iamburakglr@gmail.com",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "local_planner = sac_trajectory_planning.local_planner:main",
            "global_planner = sac_trajectory_planning.global_planner:main",
            "pc2og = sac_trajectory_planning.pc2og:main" 
            
        ],
    },
)
