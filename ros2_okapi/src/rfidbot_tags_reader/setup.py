from setuptools import setup, find_packages

package_name = 'rfidbot_tags_reader'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/rfidbot_tags_reading.launch.xml',
            'launch/rfidbot_tags_reading_py.launch.xml',
             'launch/rfidbot_tags_reading_RFD8500.launch.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rfid3',
    maintainer_email='you@example.com',
    description='Python version of RFIDBot Tags Reader',
    #entry_points={
    #  'console_scripts': [
    #       'rfidbot_tags_reader_main_py = rfidbot_tags_reader.rfidbot_tags_reader_main_py:main',
    #       'rfd8500_reader = rfidbot_tags_reader.rfd8500_reader:main',
    #    ],
    #},
)

