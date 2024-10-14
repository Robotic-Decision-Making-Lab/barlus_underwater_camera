import os
from glob import glob

from generate_parameter_library_py.setup_helper import generate_parameter_module
from setuptools import find_packages, setup

generate_parameter_module(
    "gstreamer_proxy_parameters",
    "barlus_gstreamer_proxy/gstreamer_proxy_parameters.yaml",
)

package_name = "barlus_gstreamer_proxy"
setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*")),
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Evan Palmer",
    maintainer_email="evanp922@gmail.com",
    description="A GStreamer wrapper used to convert an RTSP stream into a ROS 2 feed",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gstreamer_proxy = barlus_gstreamer_proxy.gstreamer_proxy:main",
        ],
    },
)
