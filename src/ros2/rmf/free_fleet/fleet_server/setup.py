from setuptools import setup
import os
from glob import glob

<<<<<<<< HEAD:src/ros2/rmf/free_fleet/fleet_adapter_robast/setup.py
package_name = 'fleet_adapter_robast'
========

package_name = 'fleet_server'
>>>>>>>> dev/RE-1636-3:src/ros2/rmf/free_fleet/fleet_server/setup.py

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',
                      package_name,
                      'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robast',
    maintainer_email='t.zurhelle@robast.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
<<<<<<<< HEAD:src/ros2/rmf/free_fleet/fleet_adapter_robast/setup.py
            'fleet_adapter=fleet_adapter_robast.fleet_adapter_robast:main'
========
                        'web_api = fleet_server.main:main'
>>>>>>>> dev/RE-1636-3:src/ros2/rmf/free_fleet/fleet_server/setup.py
        ],
    },
)
