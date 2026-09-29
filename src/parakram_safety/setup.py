from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'parakram_safety'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Suryansh Singh',
    maintainer_email='suryanshsinghvns@gmail.com',
    description='PARAKRAM: comms-free reactive safety layer (Collision Monitor + NH-ORCA filter).',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'orca_filter = parakram_safety.orca_filter:main',
            'safety_acceptance = parakram_safety.acceptance:main',
        ],
    },
)
