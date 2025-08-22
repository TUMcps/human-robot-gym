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
from setuptools.command.develop import develop
from setuptools.command.install import install

def build_sara_shield():
    """Build sara-shield after main installation."""
    try:
        from human_robot_gym.controllers.failsafe_controller.sara_shield_build import build_sara_shield
        build_sara_shield()
    except Exception as e:
        print(f"Warning: Failed to build sara-shield: {e}")
        print("You may need to manually run: cd human_robot_gym/controllers/failsafe_controller/sara-shield && python setup.py install")

class PostDevelopCommand(develop):
    """Post-installation for development mode."""
    def run(self):
        develop.run(self)
        build_sara_shield()

class PostInstallCommand(install):
    """Post-installation for installation mode."""
    def run(self):
        install.run(self)
        build_sara_shield()

if __name__ == "__main__":
    setup(
        packages=[package for package in find_packages() if package.startswith("human_robot_gym")],
        python_requires=">=3",
        eager_resources=["*"],
        include_package_data=True,
        cmdclass={
            'develop': PostDevelopCommand,
            'install': PostInstallCommand,
        },
    )
