from setuptools import setup

package_name = 'repeater_package'

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
    description='This ROS2 node (UDPRepeaterNode) accepts JSON data via a UDP\
        socket and publishes it to the topic "Fedor_bare_data" at a frequency\
        of 333.3 Hz.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'repeater_node = repeater_package.repeater_node:main'
        ],
    },
)
