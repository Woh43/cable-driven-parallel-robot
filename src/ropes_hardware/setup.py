from setuptools import setup


package_name = 'ropes_hardware'

setup(
    name=package_name,
    version='0.5.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/config', [
            'config/hardware.example.yaml', 'config/hardware.bluetooth.example.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Project owner',
    maintainer_email='maintainer@example.com',
    description='USB serial hardware backend for three closed-loop stepper drives.',
    license='Proprietary',
    entry_points={
        'console_scripts': ['hardware_driver = ropes_hardware.hardware_driver:main'],
    },
)
