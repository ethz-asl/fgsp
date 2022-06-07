#!/usr/bin/env python3
from setuptools import setup

package_name = fgsp
setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
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
            'graph_client = src.fgsp.graph_client:main',
        ],
    },
)
