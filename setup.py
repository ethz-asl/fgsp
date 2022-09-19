#!/usr/bin/env python3
import os
from setuptools import setup
from glob import glob

package_name = 'fgsp'

setup(
    name=package_name,
    version='2.0.0',
    packages=[f'src/{package_name}', f'src/{package_name}.classifier', f'src/{package_name}.tools',
              f'src/{package_name}.graph', f'src/{package_name}.common', f'src/{package_name}.controller'],
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
    description='Factor Graph Signal Processing',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'graph_monitor = src.fgsp.graph_monitor:main',
            'graph_client = src.fgsp.graph_client:main',
            'simulation = src.fgsp.tools.simulation:main',
            'reproject_pub = src.fgsp.tools.reproject_pub:main',
            'cloud_saver = src.fgsp.tools.cloud_saver:main',
            'cloud_publisher = src.fgsp.tools.cloud_publisher:main',
            'lookup_aligned_pose = src.fgsp.tools.lookup_aligned_pose:main',
            'object_publisher = src.fgsp.tools.object_publisher:main',
        ],
    },
)
