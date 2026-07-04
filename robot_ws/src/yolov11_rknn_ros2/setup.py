from glob import glob
from setuptools import find_packages, setup


package_name = "yolov11_rknn_ros2"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/rknnModel", glob("rknnModel/*.rknn")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Codex",
    maintainer_email="codex@example.com",
    description="ROS 2 RKNN detector node for a converted YOLOv11n model.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "detector_node = yolov11_rknn_ros2.detector_node:main",
        ],
    },
)
