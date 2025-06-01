from setuptools import setup

package_name = 'completed_scripts'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/H1_control_by_buttons_with_hands.launch.py']),
        ('share/' + package_name, ['launch/H1_control_by_buttons_without_hands.launch.py']),
        ('share/' + package_name, ['launch/Fedor_teleop_with_hands.launch.py']),
        ('share/' + package_name, ['launch/Fedor_teleop_without_hands.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='banana-killer',
    maintainer_email='sashagrachev2005@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },

)
