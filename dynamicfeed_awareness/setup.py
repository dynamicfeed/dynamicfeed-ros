from setuptools import find_packages, setup

package_name = "dynamicfeed_awareness"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "cryptography"],
    zip_safe=True,
    maintainer="Dynamic Feed",
    maintainer_email="hello@dynamicfeed.ai",
    description="ROS 2 node publishing Dynamic Feed signed situational awareness (DF-VERIFY/1).",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "awareness_node = dynamicfeed_awareness.awareness_node:main",
        ],
    },
)
