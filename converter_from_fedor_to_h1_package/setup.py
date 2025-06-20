from setuptools import setup

package_name = 'converter_from_fedor_to_h1_package'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='banana-killer',
    maintainer_email='sashagrachev2005@gmail.com',
    description="This ROS2 node converts joint position data from Fedor format to\
        Unitree H1 format by subscribing to the Fedor_bare_data topic,\
        transforming angles with respect to each joint's constraints, and\
        publishing the result to positions_to_unitree at 333.3 Hz,\
        including a smooth falloff on shutdown. The code includes dictionaries\
        for mapping joints to their ranges, as well as error handling and\
        logging via ROS2.",
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'converter_from_fedor_to_h1_node = converter_from_fedor_to_h1_package.converter_from_fedor_to_h1_node:main',
        ],
    },
)
