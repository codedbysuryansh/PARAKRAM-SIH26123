from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'parakram_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'models', 'parakram_burger'),
            glob('models/parakram_burger/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Suryansh Singh',
    maintainer_email='suryanshsinghvns@gmail.com',
    description='PARAKRAM simulation: warehouse world, namespaced TB3 spawning, grid utils.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'generate_warehouse = parakram_sim.warehouse:main',
            'ground_truth_publisher = parakram_sim.ground_truth:main',
            'sim_clock_bridge = parakram_sim.sim_clock:main',
        ],
    },
)
