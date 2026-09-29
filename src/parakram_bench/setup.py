from setuptools import find_packages, setup

package_name = 'parakram_bench'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/scripts', ['scripts/netem.sh', 'scripts/netem_verify.sh']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Suryansh Singh',
    maintainer_email='suryanshsinghvns@gmail.com',
    description='PARAKRAM: Benchmark harness, scenarios, loss injector, loggers, GT checker.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'run_loss_sweep = parakram_bench.run_loss_sweep:main',
            'netem_check = parakram_bench.netem_check:main',
        ],
    },
)
