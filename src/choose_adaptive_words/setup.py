# https://docs.ros.org/en/foxy/How-To-Guides/Developing-a-ROS-2-Package.html

import os
from glob import glob
from setuptools import setup

package_name = "choose_adaptive_words"

setup(
    name=package_name,
    version="0.0.0",
    # Packages to export
    packages=[package_name],
    # Files we want to install, specifically launch files
    # package_data={'': ['*.svg', "*.ui", "*.png"]},
    # include_package_data=True,
    data_files=[
        # Install marker file in the package index
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        # Include our package.xml file
        (os.path.join("share", package_name), ["package.xml"]),
        # Include all launch files.
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join(package_name, "launch", "*.launch.py")),
        ),
        (os.path.join('lib/python3.10/site-packages/choose_adaptive_words/design/assets'), glob(package_name+'/design/assets/*')),
        (os.path.join('lib/python3.10/site-packages/choose_adaptive_words/design/'), glob(package_name+'/design/*.ui')),
    ],
    # This is important as well
    install_requires=["setuptools"],
    zip_safe=True,
    author="ROS 2 Developer",
    author_email="ros2@ros.com",
    maintainer="ROS 2 Developer",
    maintainer_email="ros2@ros.com",
    keywords=["foo", "bar"],
    classifiers=[
        "Intended Audience :: Developers",
        "License :: TODO",
        "Programming Language :: Python",
        "Topic :: Software Development",
    ],
    description="My awesome package.",
    license="TODO",
    # Like the CMakeLists add_executable macro, you can add your python
    # scripts here.
    entry_points={
        "console_scripts": [
            "child_ui = choose_adaptive_words.child_ui:main",
            "manager_ui = choose_adaptive_words.manager_ui:main",
            "temp_backend = choose_adaptive_words.temp_backend:main",
        ]
    },
)
