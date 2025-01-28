from setuptools import find_packages, setup

package_name = "initial_pose_publisher"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, [f"launch/{package_name}.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Andreas Neumann",
    maintainer_email="a.neumann@robast.de",
    description="Creates a subscriber with message type point /set_initial_pose, transforms it to a pose message and publishes it to /initialpose",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [f"{package_name}={package_name}.{package_name}:main"],
    },
)
