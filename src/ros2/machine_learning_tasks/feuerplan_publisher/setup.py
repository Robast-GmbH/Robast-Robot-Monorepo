import os

from glob import glob
from setuptools import find_packages, setup

package_name = 'feuerplan_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'images'), glob('images/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sagar V Lingaraj',
    maintainer_email='s.lingaraj@robast.de',
    description='Publishes feuerplan as an occupancy grid',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'feuerplan_publisher_node = feuerplan_publisher.publish_feuerplan:main'
        ],
    },
)
