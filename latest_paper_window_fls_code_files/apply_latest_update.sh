#!/usr/bin/env bash
set -euo pipefail

# Apply the latest EKF-ANN-FLS update by replacing the affected files.
# Usage:
#   chmod +x apply_latest_update.sh
#   ./apply_latest_update.sh
#
# Optional custom ROS workspace src path:
#   ./apply_latest_update.sh ~/bumperbot_ws/src

SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET_ROOT="${1:-$HOME/bumperbot_ws/src}"
BACKUP_DIR="$SRC_DIR/backup_before_latest_update_$(date +%Y%m%d_%H%M%S)"

copy_file() {
  local file_name="$1"
  local relative_target="$2"
  local source_path="$SRC_DIR/$file_name"
  local target_path="$TARGET_ROOT/$relative_target"

  if [[ ! -f "$source_path" ]]; then
    echo "Missing update file: $source_path" >&2
    exit 1
  fi

  if [[ ! -f "$target_path" ]]; then
    echo "Target file not found: $target_path" >&2
    exit 1
  fi

  mkdir -p "$BACKUP_DIR/$(dirname "$relative_target")"
  cp "$target_path" "$BACKUP_DIR/$relative_target"
  cp "$source_path" "$target_path"
  echo "Updated: $target_path"
}

echo "Applying latest EKF-ANN-FLS update"
echo "Source: $SRC_DIR"
echo "Target: $TARGET_ROOT"
echo "Backup: $BACKUP_DIR"

copy_file "ann_pseudo_gps.py" "bumperbot_localization/bumperbot_localization/ann_pseudo_gps.py"
copy_file "ann_pseudo_gps.yaml" "bumperbot_localization/config/ann_pseudo_gps.yaml"
copy_file "ann_trainer.py" "bumperbot_localization/bumperbot_localization/ann_trainer.py"
copy_file "ann_trainer.yaml" "bumperbot_localization/config/ann_trainer.yaml"
copy_file "fuzzy_localization.py" "bumperbot_localization/bumperbot_localization/fuzzy_localization.py"
copy_file "fuzzy_localization.yaml" "bumperbot_localization/config/fuzzy_localization.yaml"
copy_file "gps_hold.py" "bumperbot_localization/bumperbot_localization/gps_hold.py"
copy_file "gps_hold.yaml" "bumperbot_localization/config/gps_hold.yaml"
copy_file "ekf_output_logger.py" "bumperbot_localization/bumperbot_localization/ekf_output_logger.py"
copy_file "ekf_output_logger.yaml" "bumperbot_localization/config/ekf_output_logger.yaml"
copy_file "local_localization.launch.py" "bumperbot_localization/launch/local_localization.launch.py"
copy_file "noisy_controller.py" "bumperbot_controller/bumperbot_controller/noisy_controller.py"
copy_file "controller.launch.py" "bumperbot_controller/launch/controller.launch.py"

echo
echo "Done. Now rebuild:"
echo "  cd ~/bumperbot_ws"
echo "  colcon build --packages-select bumperbot_localization bumperbot_controller"
echo "  source install/setup.bash"
