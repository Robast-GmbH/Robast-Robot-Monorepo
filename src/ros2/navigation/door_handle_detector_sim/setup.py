import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'door_handle_detector_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resources/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'weights'), glob('weights/*.pt')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='s.lingaraj@robast.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'door_handle_detector = door_handle_detector_sim.door_handle_node:main'
        ],
    },
)
