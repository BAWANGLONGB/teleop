#!/usr/bin/env python3
"""Convert one recorded episode directory into one self-contained MCAP."""

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from xr_marvin_teleop.common.episode_postprocessor import postprocess_episode
from xr_marvin_teleop.common.episode_package import package_episode
from xr_marvin_teleop.common.episode_validator import validate_episode


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episode", type=Path)
    parser.add_argument("--urdf", type=Path)
    arguments = parser.parse_args()
    options = {}
    if arguments.urdf is not None:
        options["urdf_path"] = arguments.urdf
    summary = postprocess_episode(arguments.episode, **options)
    manifest = validate_episode(arguments.episode)
    package = package_episode(arguments.episode)
    print(json.dumps({"postprocessing": summary, "validation": manifest, "package": str(package)}, ensure_ascii=False, indent=2))
    if manifest["status"] == "rejected":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
