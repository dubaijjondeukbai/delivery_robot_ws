from setuptools import find_packages, setup

package_name = 'delivery_robot_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeongjonghee',
    maintainer_email='jeongjonghee@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'stm32_serial_bridge = delivery_robot_bridge.stm32_serial_bridge:main',       
		    'esp32_serial_bridge = delivery_robot_bridge.esp32_serial_bridge:main',
        ],
    },
)
