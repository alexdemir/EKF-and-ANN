#!/usr/bin/env python3
"""Apply paper-style mismatch trajectory update.

Usage on Ubuntu:
    cd ~/bumperbot_ws/scripted_mismatch_route_update
    python3 apply_scripted_mismatch_route_update.py ~/bumperbot_ws/src
"""

from __future__ import annotations

import argparse
import datetime as _dt
import os
import shutil
from pathlib import Path


FILES = {
    "scripted_trajectory.py": "bumperbot_controller/bumperbot_controller/scripted_trajectory.py",
}


def copy_with_backup(source: Path, target: Path, backup_root: Path, workspace: Path) -> None:
    if not source.is_file():
        raise FileNotFoundError(f"Missing patch file: {source}")
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists():
        relative = target.relative_to(workspace)
        backup = backup_root / relative
        backup.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(target, backup)
    shutil.copy2(source, target)
    if target.suffix == ".py":
        target.chmod(target.stat().st_mode | 0o111)
    print(f"Updated: {target}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "workspace_src",
        nargs="?",
        default="~/bumperbot_ws/src",
        help="Path to the ROS workspace src directory.",
    )
    args = parser.parse_args()

    patch_dir = Path(__file__).resolve().parent
    workspace = Path(os.path.expanduser(args.workspace_src)).resolve()
    if not workspace.is_dir():
        raise NotADirectoryError(f"Workspace src directory not found: {workspace}")

    stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    backup_root = patch_dir / f"backup_before_scripted_mismatch_route_update_{stamp}"
    backup_root.mkdir(parents=True, exist_ok=True)

    print("Applying scripted mismatch trajectory update")
    print(f"Patch folder: {patch_dir}")
    print(f"Workspace src: {workspace}")
    print(f"Backup folder: {backup_root}")

    for source_name, target_rel in FILES.items():
        copy_with_backup(patch_dir / source_name, workspace / target_rel, backup_root, workspace)

    print()
    print("Done. Now rebuild:")
    print("  cd ~/bumperbot_ws")
    print("  colcon build --packages-select bumperbot_controller")
    print("  source install/setup.bash")


if __name__ == "__main__":
    main()
