# How to Apply This Update

This folder is a patch-style replacement bundle for the latest EKF-ANN-FLS code.

## Recommended Method

Copy or unzip this folder inside Ubuntu, then run:

```bash
cd /path/to/latest_paper_window_fls_code_files
chmod +x apply_latest_update.sh
./apply_latest_update.sh
```

By default, the script updates:

```txt
~/bumperbot_ws/src
```

If your workspace is somewhere else:

```bash
./apply_latest_update.sh /path/to/your/bumperbot_ws/src
```

The script backs up every replaced file into:

```txt
backup_before_latest_update_<date>_<time>
```

## Rebuild

After applying:

```bash
cd ~/bumperbot_ws
colcon build --packages-select bumperbot_localization bumperbot_controller
source install/setup.bash
```

## Why Not a Normal `.patch`?

A normal unified diff patch can fail if your local files have drifted from the exact version used to create the patch.
This bundle is safer for the current project because it replaces only the affected files and keeps backups.
