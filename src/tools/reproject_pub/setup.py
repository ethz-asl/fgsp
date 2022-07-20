#!/usr/bin/env python3
import os
from setuptools import setup
from glob import glob

package_name = 'reproject_viz'

setup(
    name=package_name,
    version='1.0.0',
    packages=['src/'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lukas Bernreiter',
    maintainer_email='berlukas@ethz.ch',
    description='Pose and Cloud Publisher',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reproject_viz = src.reproject_viz:main',
        ],
    },
)
