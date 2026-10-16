import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'ugv_webots'


def share(subdir, pattern='*'):
    return (os.path.join('share', package_name, subdir), glob(os.path.join(subdir, pattern)))


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        share('launch', '*.py'),
        share('worlds', '*.wbt'),
        share('protos', '*.proto'),
        share('resource', '*.yaml'),
        share('resource', '*.urdf'),
        share('maps', '*.yaml'),
        share('maps', '*.pgm'),
        share('rviz', '*.rviz'),
        share('scripts', '*.py'),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='S-H-E-R-P-AI-Lab',
    maintainer_email='a.hamilton72006@gmail.com',
    description='Webots simulation of the Waveshare UGV Rover',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scan_normalizer = ugv_webots.scan_normalizer:main',
            'motion_check = ugv_webots.motion_check:main',
            'rtf_meter = ugv_webots.rtf_meter:main',
            'print_urdf = ugv_webots.description:main',
        ],
    },
)
