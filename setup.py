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
    packages=[package for package in find_packages() if package.startswith("human_robot_gym")],
    install_requires=[
        "bvh==0.3",
        "gym>=0.21",
        "meshcat==0.3.2",
        "mujoco_py==2.1.2.14",
        "numpy==1.22.3",
        "opencv_python==4.5.5.64",
        "pinocchio==0.4.3",
        "robosuite==1.3.2",
        "scipy==1.8.0",
        "stable_baselines3==1.5.0",
        "numba>=0.52.0,<=0.53.1",
        "mujoco-py<2.2,>=2.1",
        "wandb>=0.12",
        "flake8>=4.0",
        "black>=22.3",
        "patchelf>=0.14",
        "h5py>=3.6",
        "hpp-fcl>=1.7",
        "tensorboard>=2.8"
    ],
    eager_resources=["*"],
    include_package_data=True,
    python_requires=">=3",
    description="human_robot_gym is an extension to the robosuite package\
                 to train RL algorithms on robots in human environments.",
    author="Jakob Thumm",
    url="TODO",
    author_email="jakob.thumm@tum.de",
    version="0.1.0",
    long_description=long_description,
    long_description_content_type="text/markdown",
)
