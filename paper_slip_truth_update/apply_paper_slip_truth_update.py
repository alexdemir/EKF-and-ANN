#!/usr/bin/env python3
"""Apply the paper-style slip + truth-reference FLS update.

Usage on Ubuntu:
    cd ~/bumperbot_ws/paper_slip_truth_update
    python3 apply_paper_slip_truth_update.py ~/bumperbot_ws/src

The script copies the updated source/config/launch files from this folder into
the ROS 2 workspace and creates timestamped backups beside the patch files.
"""

from __future__ import annotations

import argparse
import datetime as _dt
import os
import shutil
from pathlib import Path


FILES = {
    "noisy_controller.py": "bumperbot_controller/bumperbot_controller/noisy_controller.py",
    "scripted_trajectory.py": "bumperbot_controller/bumperbot_controller/scripted_trajectory.py",
    "controller.launch.py": "bumperbot_controller/launch/controller.launch.py",
    "fuzzy_localization.py": "bumperbot_localization/bumperbot_localization/fuzzy_localization.py",
    "fuzzy_localization.yaml": "bumperbot_localization/config/fuzzy_localization.yaml",
    "ekf_output_logger.py": "bumperbot_localization/bumperbot_localization/ekf_output_logger.py",
    "ekf_output_logger.yaml": "bumperbot_localization/config/ekf_output_logger.yaml",
    "local_localization.launch.py": "bumperbot_localization/launch/local_localization.launch.py",
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
        mode = target.stat().st_mode
        target.chmod(mode | 0o111)
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
    backup_root = patch_dir / f"backup_before_paper_slip_truth_update_{stamp}"
    backup_root.mkdir(parents=True, exist_ok=True)

    print("Applying paper-style slip + truth-reference FLS update")
    print(f"Patch folder: {patch_dir}")
    print(f"Workspace src: {workspace}")
    print(f"Backup folder: {backup_root}")

    for source_name, target_rel in FILES.items():
        copy_with_backup(patch_dir / source_name, workspace / target_rel, backup_root, workspace)

    print()
    print("Done. Now rebuild:")
    print("  cd ~/bumperbot_ws")
    print("  colcon build --packages-select bumperbot_localization bumperbot_controller")
    print("  source install/setup.bash")


if __name__ == "__main__":
    main()
