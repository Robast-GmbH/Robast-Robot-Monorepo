from setuptools import setup

package_name = "door_adapter_template"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Andreas Neumann",
    maintainer_email="a.neumann@robast.de",
    description="Door adapter for RMF",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["door_adapter = door_adapter_template.door_adapter:main"],
    },
)
