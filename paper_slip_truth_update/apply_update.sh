#!/usr/bin/env bash
set -euo pipefail

TARGET="${1:-$HOME/bumperbot_ws/src}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BACKUP_DIR="$SCRIPT_DIR/backup_before_paper_slip_truth_update_$(date +%Y%m%d_%H%M%S)"

echo "Applying paper-style slip + truth-reference FLS update"
echo "Source: $SCRIPT_DIR"
echo "Target: $TARGET"
echo "Backup: $BACKUP_DIR"

mkdir -p "$BACKUP_DIR"

copy_with_backup() {
  local src="$1"
  local dst="$2"
  mkdir -p "$(dirname "$dst")"
  if [[ -f "$dst" ]]; then
    local rel="${dst#$TARGET/}"
    mkdir -p "$BACKUP_DIR/$(dirname "$rel")"
    cp "$dst" "$BACKUP_DIR/$rel"
  fi
  cp "$src" "$dst"
  echo "Updated: $dst"
}

copy_with_backup "$SCRIPT_DIR/noisy_controller.py" \
  "$TARGET/bumperbot_controller/bumperbot_controller/noisy_controller.py"
copy_with_backup "$SCRIPT_DIR/scripted_trajectory.py" \
  "$TARGET/bumperbot_controller/bumperbot_controller/scripted_trajectory.py"
copy_with_backup "$SCRIPT_DIR/controller.launch.py" \
  "$TARGET/bumperbot_controller/launch/controller.launch.py"
copy_with_backup "$SCRIPT_DIR/fuzzy_localization.py" \
  "$TARGET/bumperbot_localization/bumperbot_localization/fuzzy_localization.py"
copy_with_backup "$SCRIPT_DIR/fuzzy_localization.yaml" \
  "$TARGET/bumperbot_localization/config/fuzzy_localization.yaml"
copy_with_backup "$SCRIPT_DIR/ekf_output_logger.py" \
  "$TARGET/bumperbot_localization/bumperbot_localization/ekf_output_logger.py"
copy_with_backup "$SCRIPT_DIR/ekf_output_logger.yaml" \
  "$TARGET/bumperbot_localization/config/ekf_output_logger.yaml"
copy_with_backup "$SCRIPT_DIR/local_localization.launch.py" \
  "$TARGET/bumperbot_localization/launch/local_localization.launch.py"

chmod +x "$TARGET/bumperbot_controller/bumperbot_controller/noisy_controller.py"
chmod +x "$TARGET/bumperbot_controller/bumperbot_controller/scripted_trajectory.py"
chmod +x "$TARGET/bumperbot_localization/bumperbot_localization/fuzzy_localization.py"
chmod +x "$TARGET/bumperbot_localization/bumperbot_localization/ekf_output_logger.py"

echo
echo "Done. Rebuild with:"
echo "  cd ~/bumperbot_ws"
echo "  colcon build --packages-select bumperbot_localization bumperbot_controller"
echo "  source install/setup.bash"
