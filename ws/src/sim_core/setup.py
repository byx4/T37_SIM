from setuptools import setup

package_name = 'sim_core'

setup(
    name=package_name,
    version='0.0.1',
    packages=['sim_core'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/sim_core']),
        ('share/sim_core', ['package.xml', 'README.md']),
        ('share/sim_core/launch', ['launch/sim_launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bryanx',
    maintainer_email='bryanx@cs.washington.edu',
    description='Minimal simulator core for driverless FSAE (ROS2 Jazzy).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mimic_perception = sim_core.mimic_perception:main',
            'vehicle_dynamics = sim_core.vehicle_dynamics:main'
        ],
    },
)
