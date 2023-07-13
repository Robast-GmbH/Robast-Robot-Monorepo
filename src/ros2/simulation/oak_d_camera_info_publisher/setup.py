from setuptools import setup

package_name = 'oak_d_camera_info_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robast',
    maintainer_email='s.lingaraj@robast.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'oak_d_camera_info = oak_d_camera_info_publisher.oak_d_camera_info_node:main'
        ],
    },
)
