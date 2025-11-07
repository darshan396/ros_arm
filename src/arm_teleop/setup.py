from setuptools import setup
import os
from glob import glob

package_name = 'arm_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.py')),
        # Include URDF files
        (os.path.join('share', package_name, 'urdf'), 
         glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Robotic arm keyboard teleoperation package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_teleop_keyboard = arm_teleop.arm_teleop_keyboard:main',
        ],
    },
)