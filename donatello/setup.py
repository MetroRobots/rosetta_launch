from glob import glob
from setuptools import setup

package_name = 'donatello'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='David V. Lu!!',
    maintainer_email='davidvlu@gmail.com',
    description='ROS 2 Launch example code for rosetta_launch',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'donatello_node = donatello.donatello_node:main',
            'five_seconds = donatello.five_seconds:main',
        ]
    },
)
