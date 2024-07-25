from setuptools import setup

package_name = "fleet_adapter_rb_theron"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, ["config.yaml"]),
        ("share/" + package_name + "/launch", ["launch/rmf_launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Andreas Neumann",
    maintainer_email="a.neumann@robast.de",
    description="A RMF fleet adapter for the rb_theron",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fleet_adapter=fleet_adapter_rb_theron.fleet_adapter:main",
        ],
    },
)
