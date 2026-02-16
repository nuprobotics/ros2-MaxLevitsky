from setuptools import setup
import os
from glob import glob

package_name = 'task02'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/task02.launch']),
        ('share/' + package_name + '/config', ['config/task02.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'publisher = task02.publisher:main',
        ],
    },
)
