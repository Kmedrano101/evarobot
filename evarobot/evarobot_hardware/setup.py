from setuptools import setup

package_name = 'evarobot_hardware'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kevin Medrano Ayala',
    maintainer_email='kevin.ejem18@gmail.com',
    description='Hardware interface nodes for EvaRobot sensors and actuators',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Motor control nodes moved to evarobot_firmware package
        ],
    },
)
