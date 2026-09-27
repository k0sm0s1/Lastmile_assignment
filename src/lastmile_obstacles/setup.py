from setuptools import find_packages, setup

package_name = "lastmile_obstacles"
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Surya",
    maintainer_email="karthikeya7890@gmail.com",
    description="Live lidar obstacle detection feeding Nav2 through a second static layer.",
    license="MIT",
    entry_points={"console_scripts": ["obstacle_mapper = lastmile_obstacles.obstacle_mapper:main"]},
)
