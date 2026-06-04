#!/usr/bin/env python3
"""Apply FLS latch hold-time update for shorter paper-style tests.

Usage on Ubuntu:
    cd ~/bumperbot_ws/fls_latch_hold_35_update
    python3 apply_fls_latch_hold_35_update.py ~/bumperbot_ws/src
"""

from __future__ import annotations

import argparse
import datetime as _dt
import os
import shutil
from pathlib import Path


FILES = {
    "fuzzy_localization.py": "bumperbot_localization/bumperbot_localization/fuzzy_localization.py",
    "fuzzy_localization.yaml": "bumperbot_localization/config/fuzzy_localization.yaml",
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
    parser.add_argument("workspace_src", nargs="?", default="~/bumperbot_ws/src")
    args = parser.parse_args()

    patch_dir = Path(__file__).resolve().parent
    workspace = Path(os.path.expanduser(args.workspace_src)).resolve()
    if not workspace.is_dir():
        raise NotADirectoryError(f"Workspace src directory not found: {workspace}")

    backup_root = patch_dir / f"backup_before_fls_latch_hold_35_{_dt.datetime.now():%Y%m%d_%H%M%S}"
    backup_root.mkdir(parents=True, exist_ok=True)

    print("Applying FLS latch hold-time update")
    print(f"Patch folder: {patch_dir}")
    print(f"Workspace src: {workspace}")
    print(f"Backup folder: {backup_root}")
    for source_name, target_rel in FILES.items():
        copy_with_backup(patch_dir / source_name, workspace / target_rel, backup_root, workspace)

    print()
    print("Done. Now rebuild:")
    print("  cd ~/bumperbot_ws")
    print("  colcon build --packages-select bumperbot_localization")
    print("  source install/setup.bash")


if __name__ == "__main__":
    main()
