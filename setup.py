from setuptools import setup
from glob import glob


package_name = 'ME465_Lab5'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*launch.py')),
        ('share/' + package_name, glob('rviz/*.rviz')),
        ('share/' + package_name, glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@todo.todo',
    description='ME 465 Lab 5',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lab5_node = ME465_Lab5.lab5_node:main'
        ],
    },
)
