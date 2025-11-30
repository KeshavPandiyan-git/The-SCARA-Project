from setuptools import setup
import os
from glob import glob

package_name = 'scara_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='SCARA Robot Control Package - Kinematics and Control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'forward_kinematics = scara_control.forward_kinematics:main',
            'inverse_kinematics = scara_control.inverse_kinematics:main',
            'joint_state_publisher = scara_control.joint_state_publisher:main',
        ],
    },
)

