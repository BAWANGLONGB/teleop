#!/usr/bin/env python3
"""Copy successful episodes (unmarked completed episodes default to success)."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))
from xr_marvin_teleop.common.collection_config import load_config, read_json, write_json
from xr_marvin_teleop.common.episode_review import (
    EPISODE_ID, annotation_lock, episode_path, read_review, session_path, session_record,
)
from xr_marvin_teleop.common.episode_video import VIDEO_VARIANTS, activity_lock


def collection_plan(root):
    entries, missing = [], []
    for session in sorted(root.glob("session_*")):
        session = session_path(root, session.name)
        if not session.is_dir():
            continue
        for candidate in sorted(session.glob("episode_*")):
            if not EPISODE_ID.fullmatch(candidate.name):
                continue
            episode = episode_path(root, session.name, candidate.name)
            review = read_review(episode)
            if review.get("result") == "failure":
                continue
            metadata = read_json(episode / "metadata.json")
            if review["result"] == "unmarked" and metadata.get("status") not in ("completed", "validated", "degraded"):
                continue  # Default success is a human outcome, not a successful recorder exit.
            if metadata.get("status") in ("starting", "recording", "finalizing"):
                raise ValueError(f"成功标注对应未结束段落：{episode}")
            options = metadata.get("export_options", metadata.get("video_outputs", {}))
            files, disabled = [], []
            for variant in VIDEO_VARIANTS:
                source = episode / "final" / f"{episode.name}.{variant}.mcap"
                if source.parent.is_symlink() or source.is_symlink() or source.resolve().parent != episode / "final":
                    raise ValueError(f"不安全的 MCAP 路径：{source}")
                if not source.is_file():
                    (missing if options.get(variant) is True else disabled).append(str(source))
                    continue
                files.append({"source": str(source),
                              "destination": f"{variant}/{session.name}__{source.name}",
                              "size_bytes": source.stat().st_size})
            entries.append({"session": session_record(session), "episode_id": episode.name,
                            "review": review, "files": files, "disabled_variants": disabled})
    return {"schema_version": 1, "selection": "explicit success OR unmarked with completed/validated/degraded status",
            "episodes": entries, "missing_files": missing}


def collect(root, destination, dry_run=False):
    root, destination = Path(root).resolve(), Path(destination).resolve()
    if not root.is_dir():
        raise ValueError("数据集根目录不存在")
    if root == destination or root.is_relative_to(destination):
        raise ValueError("输出不能覆盖数据集或其父目录")
    if destination.is_relative_to(root):
        first = destination.relative_to(root).parts[0]
        if first.startswith(("session_", ".")) or first == "data":
            raise ValueError("输出不能位于原始 Session、旧数据或归档目录内")
    if destination.exists():
        raise FileExistsError("输出目录已存在，请使用新目录以避免覆盖或混入过期标注")
    with activity_lock(root, exclusive=True), annotation_lock(root):
        plan = collection_plan(root)
        if dry_run:
            return plan
        if plan["missing_files"]:
            raise ValueError("成功段落缺少启用的最终 MCAP，请先离线导出：" + ", ".join(plan["missing_files"]))
        if not plan["episodes"] or not any(e["files"] for e in plan["episodes"]):
            raise ValueError("没有成功且可整理的 MCAP（正常结束的未标注段默认成功）")
        destination.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(prefix=".collect-success-", dir=destination.parent) as temporary:
            staging = Path(temporary) / "ready"
            for variant in VIDEO_VARIANTS:
                (staging / variant).mkdir(parents=True)
            for entry in plan["episodes"]:
                for file in entry["files"]:
                    target = staging / file["destination"]
                    digest = hashlib.sha256()
                    with Path(file["source"]).open("rb") as source, target.open("xb") as output:
                        for block in iter(lambda: source.read(1024 * 1024), b""):
                            digest.update(block)
                            output.write(block)
                        output.flush()
                        os.fsync(output.fileno())
                    if target.stat().st_size != file["size_bytes"]:
                        raise ValueError(f"源文件大小发生变化：{file['source']}")
                    file["sha256"] = digest.hexdigest()
            write_json(staging / "manifest.json", plan)
            if destination.exists():
                raise FileExistsError(destination)
            staging.rename(destination)
        return plan


def main():
    config = load_config()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, default=Path(config["paths"]["output_root"]))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    os.nice(config["runtime"]["export_nice"])
    try:
        plan = collect(args.output_root, args.output, args.dry_run)
    except (ValueError, OSError, RuntimeError) as error:
        parser.exit(1, f"{error}\n")
    print(json.dumps(plan, ensure_ascii=False, indent=2))
    if plan["missing_files"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
