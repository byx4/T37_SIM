from setuptools import setup

package_name = 'sim_viz'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/viz.launch.py']),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='Bryan',
    maintainer_email='bryanx@cs.washington.edu',
    description='Visualization helpers for T37_SIM',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'publish_cone_map = sim_viz.publish_cone_map:main',
            'pose_to_tf = sim_viz.pose_to_tf:main',
        ],
    },
)