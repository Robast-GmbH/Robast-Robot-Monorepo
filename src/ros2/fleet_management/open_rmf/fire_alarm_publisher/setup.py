from setuptools import find_packages, setup

package_name = 'fire_alarm_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andreas Neumann',
    maintainer_email='a.neumann@robast.de',
    description='Polls a fire alarm trigger state from a remote server and publishes it to the related rmf topic',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "fire_alarm_publisher = fire_alarm_publisher.fire_alarm_publisher:main"
        ],
    },
)
