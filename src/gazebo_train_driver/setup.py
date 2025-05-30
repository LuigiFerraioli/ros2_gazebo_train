import os
from setuptools import setup
from glob import glob

package_name = 'gazebo_train_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luigi',
    maintainer_email='luigi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
           'teleop = gazebo_train_driver.teleop:main',
           'controller = gazebo_train_driver.controller:main'
           ],
    },
)

