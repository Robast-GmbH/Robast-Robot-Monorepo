from setuptools import setup
import os
from glob import glob

<<<<<<< HEAD:src/api_server/setup.py
package_name = 'api_server'
=======
package_name = 'room_selection'
>>>>>>> master:src/room_selection/setup.py

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
<<<<<<< HEAD:src/api_server/setup.py
    maintainer_email='ubuntu@todo.todo',
=======
    maintainer_email='t.alscher@tiplu.de',
>>>>>>> master:src/room_selection/setup.py
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
<<<<<<< HEAD:src/api_server/setup.py
            'ros_server = api_server.server:main',
=======
            'room_selection_nav_goal = room_selection.room_selection_nav_goal:main'
>>>>>>> master:src/room_selection/setup.py
        ],
    },
)
