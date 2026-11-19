from setuptools import setup
import os
from glob import glob

pkg = 'elev_bringup'
setup(
    name=pkg,
    version='0.2.0',
    packages=[pkg],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + pkg]),
        ('share/' + pkg, ['package.xml']),
        (os.path.join('share', pkg, 'launch'), glob('launch/*.py')),
        (os.path.join('share', pkg, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={'console_scripts': [
        'elevation_mapper = elev_bringup.mapper_node:main',
    ]},
)
