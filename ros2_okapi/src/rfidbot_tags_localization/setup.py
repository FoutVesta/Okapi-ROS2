from setuptools import setup, find_packages
import os
import glob

package_name = 'rfidbot_tags_localization'

# Collect data files (e.g., antenna models) if the directory exists
data_files_extra = []
data_dir = 'data'
if os.path.isdir(data_dir):
    for f in glob.glob(os.path.join(data_dir, '*')):
        data_files_extra.append(
            (os.path.join('share', package_name, 'data'), [f])
        )

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[f"{package_name}", f"{package_name}.*"]),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/rfidbot_tags_localization.launch.xml',
             'launch/rfh_tags_localization.launch.xml']),
    ] + data_files_extra,
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
