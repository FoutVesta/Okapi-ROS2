from setuptools import setup

package_name = 'rfh_share_lib'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rfid3',
    maintainer_email='rfid@todo.todo',
    description='Shared library utilities for RFIDbot localization and control',
    license='MIT',
    entry_points={'console_scripts': []},
)
