from setuptools import setup
import os
from glob import glob

package_name = 'py_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Laxman Poudel',
    maintainer_email='poudell@lafayette.edu',
    description='UWB serial reader, ROS 2 publisher, and CSV logger',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'serial_publisher = py_pubsub.serial_publisher:main',
        ],
    },
)
