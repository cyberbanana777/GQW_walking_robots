from setuptools import setup

package_name = 'extractor_package'

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
    description='ROS2 node for monitoring and comparing joint angles between \
        Fedor device and Unitree H1 robot. Subscribes to Fedor data(JSON) \
        and H1 motor states, publishes selected joint values for PlotJuggler\
        visualization. Supports joint selection via H1_joint_num parameter.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'extractor_node = extractor_package.extractor_node:main'
        ],
    },
)
