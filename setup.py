from setuptools import setup
import os
from glob import glob

package_name = 'susumu_tenkey_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sato-susumu',
    maintainer_email='75652942+sato-susumu@users.noreply.github.com',
    description='ROS2 package for controlling topics with a tenkey.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tenkey_controller = susumu_tenkey_controller.tenkey_controller:main'
        ],
    },
)
