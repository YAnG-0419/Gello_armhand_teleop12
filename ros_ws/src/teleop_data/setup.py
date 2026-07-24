from glob import glob

from setuptools import find_packages, setup


setup(
    name="teleop_data",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/teleop_data"]),
        ("share/teleop_data", ["package.xml"]),
        ("share/teleop_data/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "numpy", "PyYAML"],
    zip_safe=True,
    maintainer="descfly",
    maintainer_email="descfly@example.com",
    description="Record, normalize, and replay bimanual teleoperation episodes.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "record = teleop_data.recorder:main",
            "convert = teleop_data.converter:main",
            "replay = teleop_data.replay:main",
        ]
    },
)
