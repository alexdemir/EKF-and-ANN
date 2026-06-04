#!/usr/bin/env python3

import shutil
import sys
from datetime import datetime
from pathlib import Path


FILES = {
    "fuzzy_localization.py": Path("bumperbot_localization/bumperbot_localization/fuzzy_localization.py"),
    "fuzzy_localization.yaml": Path("bumperbot_localization/config/fuzzy_localization.yaml"),
}


def main():
    if len(sys.argv) != 2:
        print("Usage: python3 apply_fls_decision_guard_update.py ~/bumperbot_ws/src")
        return 2

    target_root = Path(sys.argv[1]).expanduser().resolve()
    source_root = Path(__file__).resolve().parent
    if not target_root.exists():
        print(f"Target does not exist: {target_root}")
        return 2

    backup_root = source_root / (
        "backup_before_fls_decision_guard_"
        + datetime.now().strftime("%Y%m%d_%H%M%S")
    )
    backup_root.mkdir(parents=True, exist_ok=True)

    print("Applying FLS decision guard update")
    print(f"Source: {source_root}")
    print(f"Target: {target_root}")
    print(f"Backup: {backup_root}")

    for source_name, relative_target in FILES.items():
        src = source_root / source_name
        dst = target_root / relative_target
        if not src.exists():
            print(f"Missing source file: {src}")
            return 1
        if not dst.exists():
            print(f"Missing target file: {dst}")
            return 1

        backup_dst = backup_root / relative_target
        backup_dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(dst, backup_dst)
        shutil.copy2(src, dst)
        print(f"Updated: {dst}")

    print("")
    print("Done. Rebuild with:")
    print("  cd ~/bumperbot_ws")
    print("  colcon build --packages-select bumperbot_localization bumperbot_controller")
    print("  source install/setup.bash")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
