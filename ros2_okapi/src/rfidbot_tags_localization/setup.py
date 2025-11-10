from setuptools import setup, find_packages

package_name = 'rfidbot_tags_localization'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[f"{package_name}", f"{package_name}.*"]),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rfid3',
    maintainer_email='rfid@todo.todo',
    description='RFID tag localization node for Okapi (ROS 2)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'rfh_tags_localization_main_ros2 = rfidbot_tags_localization.rfh_tags_localization_main_ros2:main',
        ],
    },
)


