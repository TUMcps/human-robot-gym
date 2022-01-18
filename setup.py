# read the contents of your README file
from os import path

from setuptools import find_packages, setup

this_directory = path.abspath(path.dirname(__file__))
with open(path.join(this_directory, "README.md"), encoding="utf-8") as f:
    lines = f.readlines()

# remove images from README
lines = [x for x in lines if ".png" not in x]
long_description = "".join(lines)

setup(
    name="human_robot_gym",
    packages=[package for package in find_packages() if package.startswith("human_gym")],
    install_requires=[
        "numpy>=1.20.0",
        "numba>=0.52.0,<=0.53.1",
        "scipy>=1.2.3",
        "free-mujoco-py==2.1.6",
        "robosuite"
    ],
    eager_resources=["*"],
    include_package_data=True,
    python_requires=">=3",
    description="human_robot_gym is an extension to the robosuite package to train RL algorithms on robots in human environments.",
    author="Jakob Thumm",
    url="TODO",
    author_email="jakob.thumm@tum.de",
    version="0.1.0",
    long_description=long_description,
    long_description_content_type="text/markdown",
)
