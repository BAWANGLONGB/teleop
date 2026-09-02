#!/usr/bin/env python3
"""Validate a recorded episode and regenerate its manifest."""

import argparse
import json
from pathlib import Path

from xr_marvin_teleop.common.episode_validator import validate_episode


def main():
    parser = argparse.ArgumentParser(description="Validate one collection episode")
    parser.add_argument("episode", type=Path)
    arguments = parser.parse_args()
    manifest = validate_episode(arguments.episode)
    print(json.dumps(manifest, ensure_ascii=False, indent=2))
    raise SystemExit(0 if manifest["status"] != "rejected" else 1)


if __name__ == "__main__":
    main()
