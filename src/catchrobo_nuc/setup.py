from setuptools import setup
from glob import glob
import os

package_name = 'catchrobo_nuc'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Altair',
    maintainer_email='altair@example.com',
    description='Catchrobo 2026 NUC側パッケージ',
    license='MIT',
    entry_points={
        'console_scripts': [
            'can_node = catchrobo_nuc.can_node:main',
            'serial_motor_node = catchrobo_nuc.serial_motor_node:main',
            'debug_node = catchrobo_nuc.debug_node:main',
        ],
    },
)
