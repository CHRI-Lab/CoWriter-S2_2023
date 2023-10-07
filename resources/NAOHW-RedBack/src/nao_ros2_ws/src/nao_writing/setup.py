from setuptools import setup

package_name = "nao_writing"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="minyic",
    maintainer_email="minyic.22@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "input_interpreter = nao_writing.input_interpreter:main",
            "learning_word = nao_writing.learning_word:main",
            "nao_writer_naoqi = nao_writing.nao_writer_naoqi:main",
            "audio_chat = nao_writing.audio_chat:main",
            "ai_image = nao_writing.ai_image:main",
        ],
    },
)
