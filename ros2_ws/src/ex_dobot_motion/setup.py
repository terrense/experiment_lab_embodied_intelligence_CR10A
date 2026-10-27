from setuptools import find_packages, setup

package_name = "ex_dobot_motion"

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
    description="EMBODIED-X guarded motion tooling for the real CR10A (dry-run by default)",
    license="MIT",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["excitation_runner = ex_dobot_motion.runner:main"]},
)
