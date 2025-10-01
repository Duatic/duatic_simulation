from setuptools import find_packages, setup
import os
from glob import glob

package_name = "alpha_simulation"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Include all files in launch folder
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Marc Blöchlinger",
    maintainer_email="mbloechli@student.ethz.ch",
    description="Simulation Interface for Alpha Robot",
    license="Apache-2.0",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [],
    },
)
