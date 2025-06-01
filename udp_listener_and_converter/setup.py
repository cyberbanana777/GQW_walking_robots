from setuptools import setup

package_name = 'udp_listener_and_converter'

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
    description='he package includes a script that allows you to listen \
                to a udp socket, convert coordinates from the Fedor robot\
                to Unitree_H1',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'udp_listener_and_converter_with_hands = udp_listener_and_converter.udp_listener_and_converter_with_hands:main',
            'udp_listener_and_converter_without_hands = udp_listener_and_converter.udp_listener_and_converter_without_hands:main',
        ],
    },
)
