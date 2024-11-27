from setuptools import find_packages, setup

package_name = "blue_planner_py"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="blue",
    maintainer_email="agrawaak@oregonstate.edu",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "plan_node = blue_planner_py.plan_node:main",
            "publish_to_ismc_ref_node = blue_planner_py.publish_to_ismc_ref_node:main",
            "subscribe_to_odom_node = blue_planner_py.subscribe_to_odom_node:main",
        ],
    },
)
