"""Setup the ezrassor_keyboard_controller module."""
import glob
import setuptools


setuptools.setup(
    name="ezrassor_keyboard_controller",
    version="2.0.0",
    description="Control the EZRASSOR with a keyboard.",
    maintainer="EZRASSOR Team",
    maintainer_email="ez.rassor@gmail.com",
    license="MIT",
    keywords=["EZRASSOR", "ROS", "ISRU", "NASA", "Rover", "UCF", "Robotics"],
    classifiers=[
        "Intended Audience :: Education",
        "Intended Audience :: Science/Research",
        "Programming Language :: Python",
        "Topic :: Education",
        "Topic :: Scientific/Engineering :: Astronomy",
        "Topic :: Scientific/Engineering :: Physics",
    ],
    packages=["ezrassor_keyboard_controller"],
    package_dir={"": "source"},
    install_requires=["setuptools"],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resources/ezrassor_keyboard_controller"],
        ),
        ("share/ezrassor_keyboard_controller", ["package.xml"]),
        ("share/ezrassor_keyboard_controller/launch", glob.glob("launch/*")),
    ],
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "keyboard_controller = ezrassor_keyboard_controller.__main__:main",
        ],
    },
)
