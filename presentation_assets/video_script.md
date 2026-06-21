# 7-Minute Video Script - Presentation Version

Use simple English and speak calmly. This script explains the project, the training process, the original paper method, and the improvement. The result numbers are mostly left for you to explain from the figures.

## 0:00-0:50 - Project, Problem, and Goal

**Show:** Gazebo robot moving, then `original_paper_flowchart.png` only briefly.

Hello, this project is about mobile robot localization.

Localization means estimating the position of the robot while it moves. This is important because a robot cannot navigate safely if it does not know where it is.

The project is based on the paper *Information Fusion of GPS, INS and Odometer Sensors for Improving Localization Accuracy of Mobile Robots in Indoor and Outdoor Applications*.

The main idea of the paper is sensor fusion. Instead of trusting only one sensor, the system combines GPS, IMU or INS, and wheel odometry.

The problem is that each sensor can fail in a different way. GPS can be lost indoors or near obstacles. Wheel odometry can drift over time. During wheel slip, odometry can be very wrong because the wheels rotate but the robot body does not move correctly. IMU measurements are fast, but they include noise and drift.

So the goal of this project is to improve position estimation when GPS is temporarily unavailable and wheel odometry is corrupted by slip.

## 0:50-1:35 - How We Tested the Problem

**Show:** Gazebo robot moving on the scripted route.

I implemented the system in ROS 2 and Gazebo.

The robot publishes GPS, IMU, and wheel odometry data. I also created repeatable test cases so the methods can be compared under the same conditions.

The first test condition is GPS dropout. This means GPS is unavailable in selected time windows.

The second test condition is wheel slip. In this case, the simulated robot body is stopped during the slip window, but the wheel odometry still reports motion. This creates a strong difference between the real robot position and the odometry estimate.

I used a short route with one lap and a longer route with two laps. The error is measured using RMSE in meters, compared with the ground-truth position from simulation.

## 1:35-2:35 - How the ANN Was Trained

**Show:** `training_process_clear.png`.

Before explaining the fuzzy logic part, I want to explain how the ANN was trained.

Strictly speaking, I did not train the robot itself. I trained the ANN localization model using data collected from the simulated robot.

During training, the robot moved in Gazebo and the system recorded sensor data. The collected data included GPS, IMU, and wheel odometry.

The ANN inputs were based on IMU and odometry features. This is important because these signals can still be available when GPS is lost.

The target output was a GPS-aided fused position. So while GPS was available, the system used a stronger fused localization estimate as the reference for training.

After collecting the dataset, the ANN was trained offline. The active model used 1545 samples, standard input normalization, and an absolute position target. The trained model was saved as `ann_model.npz`.

In the final tests, the ANN is not retrained online. It uses the saved model and publishes a pseudo-GPS position estimate when live GPS is missing or unreliable.

## 2:35-3:35 - Original Paper Method

**Show:** `original_paper_flowchart.png`.

Now I will explain the original paper method that I reproduced.

The paper uses Kalman filters and ANN together. One part combines GPS and IMU. Another part, called KF2 in my project, combines GPS or held GPS with wheel odometry.

The ANN produces a pseudo-GPS estimate. Then the fuzzy logic system decides how much weight should be given to ANN and how much weight should be given to KF2.

In the original paper-style fuzzy system, the main input is the speed difference between odometry and IMU or INS. If this speed difference is small, the system assumes there is no serious slip. If this speed difference is large, the system assumes that slip may be happening.

The fuzzy system produces an alpha value. The final position is calculated as:

```text
Final position = alpha * ANN + (1 - alpha) * KF2
```

In my figures, I call this reproduced method **Original Paper FLS**.

## 3:35-4:10 - Original Paper Results

**Show:** `original_paper_results_all_cases.png`.

This figure shows the results of the Original Paper FLS method.

The figure includes both the short route and the long route. It also includes the whole route, GPS-unavailable cases, the wheel-slip interval, and the period after wheel slip.

One important detail is that the GPS-unavailable cases are not all equally difficult. In the first two cases, GPS is unavailable but odometry is still accurate, so KF2 can still work well. In the harder case, GPS is unavailable while odometry is also corrupted by slip. That is why KF2 error increases strongly there.

Each value is RMSE in meters, so lower is better.

Here I compare KF2, ANN, the final Original Paper FLS output, and wheel odometry.

From this figure, we can see that the reproduced paper method improves over raw wheel odometry, but it is not always the best choice during wheel slip. In some slip cases, ANN is better than the final fuzzy output, which means the fuzzy system still gives too much weight to KF2.

This result motivated the improvement in the FLS part.

## 4:10-5:05 - What We Changed in the FLS

**Show:** `proposed_gps_gated_flowchart.png` or explain without a flowchart.

The main limitation of the original method is that the fuzzy system mostly depends on one input: the speed mismatch between odometry and IMU.

This input is useful, but it does not directly check which position estimate is more reliable at that moment.

For example, during slip, KF2 can be wrong because it uses corrupted odometry. ANN can be closer to the true position, but the original fuzzy system may still mix too much KF2 into the final output.

To solve this, I added the **GPS-Gated FLS** method.

The final output is still fuzzy fusion between ANN and KF2. I did not replace the output with raw GPS.

The change is in how alpha is selected.

The GPS-Gated FLS still uses the velocity mismatch to detect slip behavior. But it also checks GPS reliability. When GPS is reliable, it compares ANN and KF2 against GPS as a reference. If ANN is closer, ANN gets more weight. If KF2 is closer, KF2 gets more weight.

When GPS is not reliable and slip is active, the system allows ANN to dominate more, because KF2 and odometry are more likely to be corrupted.

So the innovation is a supervisory gate for the fuzzy system. GPS is used as a reliability reference, not as the final position output.

## 5:05-5:35 - GPS-Gated FLS Results

**Show:** `gps_gated_results_all_cases.png`.

This figure shows the results of the GPS-Gated FLS method.

The structure of the figure is the same as the previous one, so the comparison is fair. It includes the short route, the long route, GPS-unavailable cases, the wheel-slip interval, and the period after wheel slip.

Again, the important distinction is this: GPS unavailable alone is not always difficult for KF2, because KF2 can still propagate with odometry. The difficult case is GPS unavailable together with corrupted odometry.

Again, the values are RMSE in meters, and lower is better.

Here, you can see how the GPS-Gated FLS changes the final FLS error in the different test parts. I will use this figure to discuss where the gated method improves the localization and where the behavior is almost the same.

## 5:35-6:25 - Percentage Improvement Comparison

**Show:** `percentage_improvement_comparison.png`.

This final result figure compares only the final FLS outputs.

It shows how much the GPS-Gated FLS reduces RMSE compared with the Original Paper FLS.

Positive values mean the GPS-Gated FLS reduced the error. A negative value means the GPS-Gated FLS was slightly worse in that specific part.

This figure is useful because it directly answers the main question of the project: did the FLS modification improve localization?

The important point is that the GPS-Gated FLS improves the most in the difficult cases, especially when slip affects odometry and when the system needs to recover after slip.

## 6:25-6:50 - Limitations

**Show:** Gazebo robot or `percentage_improvement_comparison.png`.

There are also limitations.

First, the tests are done in simulation. A real robot would need real sensor calibration and real-world data.

Second, the GPS-Gated FLS uses GPS as a reliability reference when GPS is available. If GPS has a large bias, the gate could make a wrong decision.

Third, the current ANN is an absolute position model. For very different routes or much longer trajectories, a residual or velocity-based ANN model may generalize better.

## 6:50-7:00 - Conclusion

**Show:** Gazebo robot moving.

In conclusion, I reproduced the original paper method using ROS 2 and Gazebo.

Then I tested it under GPS dropout and wheel slip.

Because the original fuzzy logic system was not always robust enough during slip, I added a GPS-gated fuzzy fusion method.

The final comparison shows that this modification can reduce localization error in the most difficult parts of the test.

## Recommended Visual Order

1. Gazebo robot moving.
2. `training_process_clear.png`
3. `original_paper_flowchart.png`
4. `original_paper_results_all_cases.png`
5. `proposed_gps_gated_flowchart.png` if you want a method diagram
6. `gps_gated_results_all_cases.png`
7. `percentage_improvement_comparison.png`
8. Gazebo robot final motion shot.
