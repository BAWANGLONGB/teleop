#!/usr/bin/env python3
"""Offline export of Foxglove AV1, H.264, and/or MJPEG MCAPs while collection is idle."""

import argparse
import json
import os
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[3]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from xr_marvin_teleop.collection.episode_postprocessor import postprocess_episode
from xr_marvin_teleop.collection.episode_video import (
    activity_lock, add_video_arguments, export_episode,
)
from xr_marvin_teleop.collection.episode_validator import validate_episode
from xr_marvin_teleop.collection.config import (
    configure_parser, apply_config, load_config, read_json, write_json,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episode", type=Path)
    parser.add_argument("--add-missing", action="store_true", help="generate missing formats without replacing existing exports")
    parser.add_argument("--urdf", type=Path)
    parser.add_argument("--output-root", type=Path,
                        help="collection root for the idle lock; defaults to episode's grandparent")
    add_video_arguments(parser, inherit=True)
    configure_parser(parser)
    arguments = parser.parse_args()
    arguments.episode = arguments.episode.expanduser().resolve()
    metadata_path = arguments.episode / "metadata.json"
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    explicit_urdf = getattr(arguments, "urdf", None)
    try:
        snapshot = metadata.get("config_snapshot")
        if snapshot is not None:
            snapshot_path = (arguments.episode / snapshot).resolve()
            if not snapshot_path.is_relative_to(arguments.episode):
                raise ValueError("config snapshot must be inside the episode")
            saved = load_config(snapshot_path)
        else:
            saved = load_config()
            saved_outputs = metadata.get("video_outputs", {})
            saved["export"].update(saved_outputs)
            if saved_outputs and "av1" not in saved_outputs:
                saved["export"]["av1"] = False
        # Relocated episodes use their actual collection root for the idle lock.
        saved["paths"]["output_root"] = str(arguments.episode.parent.parent)
        config = apply_config(arguments, saved=saved)
        if arguments.config is not None:
            patch = read_json(arguments.config)
            if set(patch) - {"schema_version", "export", "runtime"} or set(patch.get("runtime", {})) - {"export_nice"}:
                raise ValueError("offline --config may only override export and runtime.export_nice")
        outputs = config["export"]
    except (ValueError, OSError) as error:
        parser.error(str(error))
    if arguments.print_effective_config:
        print(json.dumps(config, ensure_ascii=False, indent=2))
        return
    options = {"urdf_path": arguments.urdf, "storage_config_path": config["recording"]["processed_storage"]}
    with activity_lock(arguments.output_root, exclusive=True):
        os.nice(config["runtime"]["export_nice"])
        if (arguments.episode / "data").exists():
            summary = metadata.get("postprocessing", {})
            if explicit_urdf is not None:
                parser.error("--urdf cannot change an existing processed bag")
            if summary.get("alignment", {}).get("clock") != "CLOCK_REALTIME":
                parser.error("existing data uses old clock alignment; move data/ aside and retry")
        else:
            summary = postprocess_episode(arguments.episode, **options)
        manifest = validate_episode(arguments.episode)
        packages = export_episode(arguments.episode, outputs, add_missing=arguments.add_missing)
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        metadata.update(export_status="completed", export_options=outputs,
                        export_config={"export": outputs, "urdf": str(arguments.urdf),
                                       "processed_storage": config["recording"]["processed_storage"]},
                        final_outputs=[str(path.relative_to(arguments.episode)) for path in sorted((arguments.episode / "final").glob("*.mcap"))])
        write_json(metadata_path, metadata)
    print(json.dumps({"postprocessing": summary, "validation": manifest, "packages": [str(p) for p in packages]}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
