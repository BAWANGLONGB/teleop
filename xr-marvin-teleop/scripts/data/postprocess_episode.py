#!/usr/bin/env python3
"""Merge a completed state/vision episode into one enriched MCAP bag."""

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from xr_marvin_teleop.common.episode_postprocessor import postprocess_episode


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episode", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--urdf", type=Path)
    arguments = parser.parse_args()
    options = {}
    if arguments.output is not None:
        options["output_path"] = arguments.output
    if arguments.urdf is not None:
        options["urdf_path"] = arguments.urdf
    summary = postprocess_episode(arguments.episode, **options)
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
