from setuptools import find_packages, setup

package_name = 'sac_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nuke',
    maintainer_email='iamburakglr@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rotation_utils = sac_utils.rotation_utils:main',
            'vehicle_twist_converter = sac_utils.vehicle_twist_converter:main'
        ],
    },
)
