from setuptools import setup

package_name = "ur5_high_level_controller"

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
    maintainer="Valentin Yuryev",
    maintainer_email="valentin.yuryev@gmail.com",
    description="UR5 high-level controller package.",
    license="BSD3",
    entry_points={
        "console_scripts": [
            "ur5_high_level_controller = ur5_high_level_controller.ur5_high_level_controller:main",
        ],
    },
)
