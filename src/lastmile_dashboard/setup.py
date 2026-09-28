import os
from glob import glob
from setuptools import find_packages, setup

package_name = "lastmile_dashboard"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "web"), glob("web/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Surya",
    maintainer_email="karthikeya7890@gmail.com",
    description="Live browser dashboard for the lastmile navigation stack.",
    license="MIT",
    entry_points={"console_scripts": ["dashboard_node = lastmile_dashboard.dashboard_node:main",
                                      "record_demo = lastmile_dashboard.record_demo:main",
                                      "delivery_node = lastmile_dashboard.delivery_node:main"]},
)
