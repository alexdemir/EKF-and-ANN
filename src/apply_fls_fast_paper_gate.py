#!/usr/bin/env python3

from __future__ import annotations

import re
import shutil
import sys
from datetime import datetime
from pathlib import Path


YAML_VALUES = {
    "small_error": "0.40",
    "large_error": "0.55",
    "ann_weight_zero": "0.0",
    "ann_weight_small": "0.35",
    "ann_weight_large": "1.0",
    "adaptive_error_enabled": "false",
    "velocity_error_source": "max",
    "use_normalized_velocity_error": "false",
    "alpha_rise_tau": "0.08",
    "alpha_fall_tau": "0.35",
    "disagreement_boost_enabled": "false",
}

PY_VALUES = {
    "small_error": "0.40",
    "large_error": "0.55",
    "ann_weight_zero": "0.0",
    "ann_weight_small": "0.35",
    "ann_weight_large": "1.0",
    "adaptive_error_enabled": "False",
    "velocity_error_source": '"max"',
    "use_normalized_velocity_error": "False",
    "alpha_rise_tau": "0.08",
    "alpha_fall_tau": "0.35",
    "disagreement_boost_enabled": "False",
}


def backup(path: Path, stamp: str) -> None:
    if path.exists():
        shutil.copy2(path, path.with_suffix(path.suffix + f".bak_{stamp}"))


def set_yaml_value(text: str, key: str, value: str) -> str:
    pattern = rf"^(\s*{re.escape(key)}:\s*).*$"
    text, count = re.subn(pattern, rf"\g<1>{value}", text, flags=re.MULTILINE)
    if count == 0:
        raise RuntimeError(f"Missing YAML key: {key}")
    return text


def set_py_default(text: str, key: str, value: str) -> str:
    pattern = rf'self\.declare_parameter\("{re.escape(key)}", [^)]+\)'
    replacement = rf'self.declare_parameter("{key}", {value})'
    text, count = re.subn(pattern, replacement, text, count=1)
    if count == 0:
        raise RuntimeError(f"Missing Python default: {key}")
    return text


def main() -> int:
    target = Path(sys.argv[1]).expanduser() if len(sys.argv) > 1 else Path.cwd()
    if not (target / "bumperbot_localization").exists():
        raise SystemExit(
            "Run this from ~/bumperbot_ws/src or pass ~/bumperbot_ws/src as an argument."
        )

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    yaml_path = target / "bumperbot_localization/config/fuzzy_localization.yaml"
    py_path = target / "bumperbot_localization/bumperbot_localization/fuzzy_localization.py"

    backup(yaml_path, stamp)
    text = yaml_path.read_text(encoding="utf-8")
    for key, value in YAML_VALUES.items():
        text = set_yaml_value(text, key, value)
    yaml_path.write_text(text, encoding="utf-8")

    backup(py_path, stamp)
    text = py_path.read_text(encoding="utf-8")
    for key, value in PY_VALUES.items():
        text = set_py_default(text, key, value)
    py_path.write_text(text, encoding="utf-8")

    print("Updated FLS to the fast paper-style 3-rule gate.")
    print("Expected behavior on the last uploaded log replay:")
    print("  - no-slip outage: FLS remains better than ANN")
    print("  - slip outage: alpha_ann rises close to 1.0 instead of staying near 0.12")
    print("  - all forced windows: FLS should be close to, or slightly better than, ANN")
    print("")
    print("Next:")
    print("  cd ~/bumperbot_ws")
    print("  colcon build --packages-select bumperbot_localization")
    print("  source install/setup.bash")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
