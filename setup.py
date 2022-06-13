#!/usr/bin/env python3
from setuptools import setup

package_name = 'fgsp'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name, f'{package_name}.foo'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            # 'graph_monitor = fgsp.graph_monitor:main',
            'graph_client = fgsp.graph_client:main',
        ],
    },
)
