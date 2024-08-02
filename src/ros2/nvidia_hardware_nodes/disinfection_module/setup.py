from setuptools import find_packages, setup

package_name = 'disinfection_module'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ("share/" + package_name, ["launch/disinfection_module_launch.py"]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='J. Ritterbach',
    maintainer_email='j.ritterbach@tiplu.de',
    description='This package is responsible for the checking if the disinfection was triggered.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'disinfection_publisher = disinfection_module.disinfection_publisher:main',
        ],
    },
)
