from glob import glob

from setuptools import setup


package_name = 'ropes_bringup'

setup(
    name=package_name,
    version='0.5.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Project owner',
    maintainer_email='maintainer@example.com',
    description='Robot bringup, manual controls and RViz cable markers.',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'cable_visualizer = ropes_bringup.cable_visualizer:main',
            'manual_control = ropes_bringup.manual_control:main',
        ],
    },
)
