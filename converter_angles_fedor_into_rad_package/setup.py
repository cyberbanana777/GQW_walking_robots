from setuptools import setup

package_name = 'converter_angles_fedor_into_rad_package'

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
    description='The node listens to bare_data, converts the copier joint \
        values ​​from the FEDOR robot to radians via the given mappings, and\
        publishes to Fedor_data_rad. Runs at 333.3 Hz for real time.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'converter_angles_fedor_into_rad_node = converter_angles_fedor_into_rad_package.converter_angles_fedor_into_rad_node:main'
        ],
    },
)
