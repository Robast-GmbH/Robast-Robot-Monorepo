from setuptools import find_packages, setup

package_name = "robot_pose_publisher"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, ["launch/robot_pose_publisher.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Andreas Neumann",
    maintainer_email="a.neumann@robast.de",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "robot_pose_publisher=robot_pose_publisher.robot_pose_publisher:main",
        ],
    },
)
