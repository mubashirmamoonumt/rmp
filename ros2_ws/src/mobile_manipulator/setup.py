from setuptools import setup
import os
from glob import glob

package_name = 'mobile_manipulator'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # 1. Install Launch Files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # 2. Install URDF Files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        
        # 3. Install World Files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        
        # 4. Install Map Files
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Mobile Manipulator Project',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Link your python script here
            'move_arm = mobile_manipulator.move_arm:main',
            'wasd_teleop = mobile_manipulator.wasd_teleop:main'
        ],
    },
)