from setuptools import setup

package_name = 'py_pubsub'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Laxman Poudel',
    maintainer_email='poudell@lafayette.edu',
    description='UWB distance publisher using wheelchair_sensor_msgs',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'serial_publisher = py_pubsub.serial_publisher:main',
        ],
    },
)
