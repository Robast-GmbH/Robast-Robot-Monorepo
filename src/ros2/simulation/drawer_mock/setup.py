from setuptools import find_packages, setup

package_name = "drawer_mock"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, ["launch/drawer_mock_launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Andreas Neumann",
    maintainer_email="a.neumann@robast.de",
    description="Simple mock which imitates the behaviour of our drawer_trees",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["drawer_mock = drawer_mock.drawer_mock:main"],
    },
)
