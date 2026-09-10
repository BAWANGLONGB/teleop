#!/usr/bin/env python3
"""Extract a self-contained episode MCAP into data/meta/videos."""

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from xr_marvin_teleop.common.episode_package import extract_episode_mcap


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episode", type=Path)
    parser.add_argument("output", type=Path)
    arguments = parser.parse_args()
    print(extract_episode_mcap(arguments.episode, arguments.output))


if __name__ == "__main__":
    main()
