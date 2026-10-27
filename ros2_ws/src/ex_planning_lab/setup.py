from glob import glob

from setuptools import find_packages, setup

package_name = "ex_planning_lab"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/scenes", glob("scenes/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="embodied-x",
    maintainer_email="slamshenxin@gmail.com",
    description="EMBODIED-X M2 planning lab",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "benchmark = ex_planning_lab.nodes.benchmark:main",
            "execute_demo = ex_planning_lab.nodes.execute_demo:main",
            "validity_probe = ex_planning_lab.nodes.validity_probe:main",
        ],
    },
)
