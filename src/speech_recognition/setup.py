import os
from glob import glob
from setuptools import find_packages, setup

package_name = "speech_recognition"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.yaml")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Chien-Pu Lin",
    maintainer_email="chienpu.lin@student.unimelb.edu.au",
    description="Processing audio data from audio_capture into transcript by using google speech to text",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "speech_recognition = speech_recognition.speech_recognition:main"
        ],
    },
)
