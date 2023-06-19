from setuptools import setup

package_name = 'free_fleet_server_fastapi'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'fastapi>=0.79.0', 'uvicorn>=0.18.2'],
    zip_safe=True,
    maintainer='robast',
    maintainer_email='t.zurhelle@robast.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'server = free_fleet_server_fastapi.main:main',
        ],
    },
)
