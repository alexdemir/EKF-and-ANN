# Paper-Style Slip + Truth-Reference FLS Update

This update is meant to test the FLS under a scenario closer to the paper:

- The scripted route publishes both a nominal wheel command and the actual body command.
- During physical slip, the actual robot body command is scaled down.
- During the same slip window, noisy odometry can integrate the nominal wheel command as if the wheels continued spinning.
- The EKF logger can evaluate errors against a truth-like simulated odometry topic instead of noisy GPS.
- FLS can use an adaptive no-slip velocity-error baseline and then react to excess velocity disagreement.

Apply on Ubuntu:

```bash
cd ~/bumperbot_ws
unzip -o paper_slip_truth_update.zip -d paper_slip_truth_update
cd paper_slip_truth_update
chmod +x apply_update.sh
./apply_update.sh ~/bumperbot_ws/src

cd ~/bumperbot_ws
colcon build --packages-select bumperbot_localization bumperbot_controller
source install/setup.bash
```

Quick verification:

```bash
grep -n "paper_slip_mode" ~/bumperbot_ws/src/bumperbot_controller/launch/controller.launch.py
grep -n "nominal_cmd_topic" ~/bumperbot_ws/src/bumperbot_controller/bumperbot_controller/scripted_trajectory.py
grep -n "reference_source" ~/bumperbot_ws/src/bumperbot_localization/config/ekf_output_logger.yaml
grep -n "adaptive_error_mode" ~/bumperbot_ws/src/bumperbot_localization/config/fuzzy_localization.yaml
```
