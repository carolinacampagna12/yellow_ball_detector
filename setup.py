# setup.py
from setuptools import setup
import os
from glob import glob

package_name = 'yellow_ball_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='barbie',
    maintainer_email='barbie@todo.todo',
    description='Package for detecting yellow balls in camera feed.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yellow_ball_detector = yellow_ball_detector.yellow_ball_detector:main',
        ],
    },
)



