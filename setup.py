"""This file defines the installation procedure of the human-robot-gym.

Owner:
    Jakob Thumm (JT)

Contributors:

Changelog:
    2.5.22 JT Formatted docstrings
    11.5.22 JT Moved install requirements to setup.cfg
    11.5.22 JT Removed long description
"""
from setuptools import find_packages, setup

if __name__ == "__main__":
    setup(
        packages=[package for package in find_packages() if package.startswith("human_robot_gym")],
        python_requires=">=3",
        eager_resources=["*"],
        include_package_data=True
    )
