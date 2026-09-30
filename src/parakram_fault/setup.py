from glob import glob

from setuptools import find_packages, setup

package_name = 'parakram_fault'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Suryansh Singh',
    maintainer_email='suryanshsinghvns@gmail.com',
    description='PARAKRAM: Heartbeat, watchdog and recovery trigger.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'heartbeat_node = parakram_fault.heartbeat_node:main',
            'watchdog_node = parakram_fault.watchdog_node:main',
            'recovery_coordinator = parakram_fault.recovery_coordinator:main',
            'recovery_logger = parakram_fault.recovery_logger:main',
        ],
    },
)
