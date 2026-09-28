from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'parakram_coord'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Suryansh Singh',
    maintainer_email='suryanshsinghvns@gmail.com',
    description='PARAKRAM: decentralized PIBT-style coordination gated by spatial leases.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'coordination_node = parakram_coord.coordination_node:main',
            'roster_helper = parakram_coord.roster:helper_main',
            'coord_acceptance = parakram_coord.acceptance:main',
        ],
    },
)
