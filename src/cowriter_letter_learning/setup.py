import os
from glob import glob
from setuptools import setup

package_name = "letter_learning_interaction"

setup(
    name=package_name,
    version="0.0.0",
    # Packages to export
    packages=[package_name, package_name + "/include"],
    # Files we want to install, specifically launch files
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
        (
            os.path.join(
                "share", package_name, "datasets/alexis_set_for_children"
            ),
            glob(
                os.path.join(
                    package_name, "datasets", "alexis_set_for_children/*.dat"
                )
            ),
        ),
        (
            os.path.join("share", package_name, "datasets/bad_letters"),
            glob(os.path.join(package_name, "datasets", "bad_letters/*.dat")),
        ),
        (
            os.path.join("share", package_name, "datasets/uji_pen_chars2"),
            glob(
                os.path.join(package_name, "datasets", "uji_pen_chars2/*.dat")
            ),
        ),
        (
            os.path.join("share", package_name, "datasets/uji_pen_subset"),
            glob(
                os.path.join(package_name, "datasets", "uji_pen_subset/*.dat")
            ),
        ),
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
            "display_manager_server = letter_learning_interaction.display_manager_server:main",  # noqa: E501
            "learning_words_nao = letter_learning_interaction.learning_words_nao:main",  # noqa: E501
            "tablet_input_interpreter = letter_learning_interaction.tablet_input_interpreter:main",  # noqa: E501
            "word_card_detector = letter_learning_interaction.word_card_detector:main",  # noqa: E501
        ]
    },
)
