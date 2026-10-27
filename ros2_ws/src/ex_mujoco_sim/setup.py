from setuptools import find_packages, setup

package_name = "ex_mujoco_sim"

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
    maintainer="embodied-x",
    maintainer_email="slamshenxin@gmail.com",
    description="EMBODIED-X MuJoCo plant behind ros2_control (simulation only)",
    license="MIT",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["mujoco_sim = ex_mujoco_sim.sim_node:main"]},
)
