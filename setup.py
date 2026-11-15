import os
from glob import glob
from setuptools import setup

package_name = 'quad_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rachitha',
    maintainer_email='your_email@example.com',
    description='Quadruped robot description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['posture = quad_description.posture:main', 'height = quad_description.height:main', 'gait = quad_description.gait:main'],
    },
)
