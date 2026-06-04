# FLS Comparison Summary - 2026-06-04

Test setup:
- Route profile: `paper_mismatch_waypoint`
- GPS dropout windows: `18:8,34:14,54:12`
- Slip: physical body stopped, wheel odometry scaled by 2.5x
- Short: 1 lap
- Long: 2 laps

Methods:
- `paper_cog_*`: paper-style SISO FLS. Input is raw INS/odometry velocity error. Triangular input MFs and Mamdani-style COG defuzzification are enabled. GPS gate, slip latch, severe boost, adaptive correction, and alpha smoothing are disabled.
- `gps_*`: proposed GPS-gated FLS. Corrected velocity error, slip latch, severe slip boost, and GPS consistency selector/cap are enabled.

All values are RMSE in meters.

| Case | Window | KF2 | ANN | FLS | Odom |
|---|---:|---:|---:|---:|---:|
| paper_cog_short | overall | 2.1676 | 0.8849 | 0.9282 | 5.6566 |
| paper_cog_short | dropout1 | 0.0062 | 0.7228 | 0.5315 | 0.0016 |
| paper_cog_short | dropout2 | 0.0238 | 1.0590 | 0.8410 | 0.0224 |
| paper_cog_short | dropout3 | 4.8548 | 0.4129 | 1.3288 | 4.8548 |
| paper_cog_short | slip | 4.0132 | 0.5751 | 1.2006 | 7.1292 |
| paper_cog_short | post-slip | 1.1256 | 0.8656 | 0.8902 | 12.2957 |
| paper_cog_long | overall | 2.3172 | 2.2475 | 2.0707 | 8.7293 |
| paper_cog_long | dropout1 | 0.0152 | 0.7942 | 0.5605 | 0.0109 |
| paper_cog_long | dropout2 | 0.0296 | 1.0067 | 0.5563 | 0.0246 |
| paper_cog_long | dropout3 | 5.3641 | 0.5918 | 1.5959 | 5.4224 |
| paper_cog_long | slip | 4.5517 | 0.7059 | 1.4535 | 7.0350 |
| paper_cog_long | post-slip | 2.0341 | 3.1082 | 2.8191 | 11.9320 |
| gps_short | overall | 3.3663 | 0.8989 | 0.4838 | 5.6639 |
| gps_short | dropout1 | 0.0116 | 0.7993 | 0.1731 | 0.0109 |
| gps_short | dropout2 | 0.0118 | 0.8452 | 0.1785 | 0.0096 |
| gps_short | dropout3 | 7.5539 | 0.7067 | 0.7050 | 7.5575 |
| gps_short | slip | 6.1936 | 0.6628 | 0.6580 | 7.1590 |
| gps_short | post-slip | 2.1039 | 0.9199 | 0.9164 | 12.3009 |
| gps_long | overall | 1.2691 | 2.1877 | 0.7373 | 8.7344 |
| gps_long | dropout1 | 0.0092 | 0.6818 | 0.1667 | 0.0046 |
| gps_long | dropout2 | 0.0284 | 1.2974 | 0.3349 | 0.0275 |
| gps_long | dropout3 | 3.0942 | 0.4206 | 0.3813 | 3.0940 |
| gps_long | slip | 2.5921 | 0.6491 | 0.6182 | 7.0235 |
| gps_long | post-slip | 1.0062 | 3.0303 | 0.9863 | 11.9927 |

Key result:
- Paper-COG FLS improves some GPS-dropout windows but can become worse than ANN during the wheel-spin slip window because it still blends in KF2 while KF2/odometry are badly corrupted.
- GPS-gated FLS is best overall in both short and long routes: 0.4838 m short and 0.7373 m long.
- Long-route recovery is the clearest difference: after slip, paper-COG FLS is 2.8191 m RMSE, while GPS-gated FLS is 0.9863 m.
