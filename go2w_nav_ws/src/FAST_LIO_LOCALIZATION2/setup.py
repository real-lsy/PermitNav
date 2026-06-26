from setuptools import setup
import os
import glob

package_name = "fast_lio_localization"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],  # Ensure this matches your Python module
    install_requires=["setuptools"],
    zip_safe=True,
    description="Fast LIO Localization ROS2 package",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "global_localization = fast_lio_localization.global_localization:main",
            "publish_initial_pose = fast_lio_localization.publish_initial_pose:main",
            "transform_fusion = fast_lio_localization.transform_fusion:main",
            "invert_livox_scan = fast_lio_localization.invert_livox_scan:main",
        ],
    },
    data_files=[
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
)
