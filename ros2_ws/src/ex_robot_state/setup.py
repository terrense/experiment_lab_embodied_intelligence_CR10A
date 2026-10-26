from setuptools import find_packages, setup

package_name = "ex_robot_state"

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
    description="EMBODIED-X robot state pipeline",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mock_joint_publisher = ex_robot_state.nodes.mock_joint_publisher:main",
            "joint_state_adapter = ex_robot_state.nodes.joint_state_adapter:main",
            "robot_state_monitor = ex_robot_state.nodes.robot_state_monitor:main",
            "dobot_feedback_observer = ex_robot_state.nodes.dobot_feedback_observer:main",
            "kinematics_crosscheck = ex_robot_state.nodes.kinematics_crosscheck:main",
            "pipeline_check = ex_robot_state.nodes.pipeline_check:main",
            "mock_dobot_controller = ex_robot_state.tools.mock_dobot_controller:main",
        ],
    },
)
