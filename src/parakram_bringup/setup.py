from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'parakram_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'behavior_trees'), glob('behavior_trees/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Suryansh Singh',
    maintainer_email='suryanshsinghvns@gmail.com',
    description='PARAKRAM bringup: per-robot Nav2, fleet sim launch, params, run manifest.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'send_test_goals = parakram_bringup.send_test_goals:main',
            'load_components = parakram_bringup.component_loader:main',
        ],
    },
)
