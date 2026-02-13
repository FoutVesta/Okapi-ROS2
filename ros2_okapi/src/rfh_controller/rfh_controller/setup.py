from setuptools import setup, find_packages

package_name = 'rfh_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/rfh_realsensemapping_bringup.launch.xml',
            'launch/rfh_rfid_bringup.launch.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YOUR_NAME',
    maintainer_email='YOUR_EMAIL@example.com',
    description='RFH system controller (ROS 2, Python).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # name users will run  =  module.submodule:function
            'rfh_console = rfh_controller.rfh_controller_console:main',
            'rfh_gui = rfh_controller.rfh_gui_main:main',
            'rfh_sys_controller = rfh_controller.rfh_sys_controller:main',
            'rfh_system_monitor = rfh_controller.rfh_system_monitor:main',
        ],
    },
)


