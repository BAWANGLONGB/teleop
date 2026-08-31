import os
from pathlib import Path

from setuptools import Extension, setup


sdk_root = Path(
    os.environ.get("XROBOTOOLKIT_SDK_ROOT", "/opt/apps/roboticsservice/SDK")
)
sdk_library_directory = Path(
    os.environ.get("XROBOTOOLKIT_SDK_LIBRARY_DIR", sdk_root / "x64")
)

setup(
    ext_modules=[
        Extension(
            "xr_marvin_teleop._xrobotoolkit_sdk",
            sources=["native/xrobotoolkit_sdk.cpp"],
            include_dirs=[str(sdk_root / "include")],
            library_dirs=[str(sdk_library_directory)],
            runtime_library_dirs=[str(sdk_library_directory)],
            libraries=["PXREARobotSDK", "json-c"],
            language="c++",
            extra_compile_args=["-std=c++17"],
        )
    ]
)
