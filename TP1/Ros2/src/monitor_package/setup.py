from setuptools import find_packages, setup

package_name = 'monitor_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    # Solo setuptools: pyserial y matplotlib se declaran en package.xml
    # (python3-serial / python3-matplotlib) y los resuelve rosdep con apt.
    # Listarlos aca hace que setup.py intente bajarlos de PyPI durante el
    # colcon build, que es justo lo que no queremos en un paquete ament_python.
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='juanma',
    maintainer_email='painenaojuanmanuel@gmail.com',
    description='Monitor del encoder: puente serial y visualizacion/logging',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'monitor_subscriber = monitor_package.monitor_subscriber:main',
            'encoder_serial_bridge = monitor_package.encoder_serial_bridge:main',
        ],
    },
)
