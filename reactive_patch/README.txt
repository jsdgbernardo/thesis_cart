OBJECTIVE 4 - INDEPENDENT LOCAL REACTIVE AVOIDANCE (NO INTER-CART MESSAGES)
============================================================================
Purpose: Add an apples-to-apples scenario-level baseline with actual lateral
maneuver attempts, not just stop-only behavior. The new METHOD is reactive_avoid.
It is distinct from the previous no_comm and local_avoid baselines.

It uses /gazebo/model_states as an IDEALIZED local perception sensor proxy,
without subscribing to /cart_states or transferring priority, intent, identity
or right-of-way. It detects nearby cart geometry/velocity and may TURN_OUT,
PASS, RETURN, or HOLD. There is no guarantee of full completion: failures and
deadlocks are legitimate measured outcomes. It is not a LiDAR-enabled baseline.

INSTALL (the ZIP contents should be placed in ~/thesis_cart/reactive_patch):
  cd ~/thesis_cart
  unzip -o ~/Downloads/objective4_reactive_baseline.zip -d reactive_patch
  /usr/bin/python3 reactive_patch/install_reactive_baseline.py
  bash -n run_baseline_trial.sh
  /usr/bin/python3 -m py_compile reactive_avoid_cart.py analyze_baselines.py

Works with the original runner/analyzer, OR after the previous stop-only
local_avoid patch. It creates backups *.pre_reactive_avoid.bak before replacing.
It does NOT overwrite previously recorded test data. Do not rerun installer
without reviewing the existing edits and backup files.

RUN (make sure Gazebo world is running, ROS_DOMAIN_ID=23):
  ./run_baseline_trial.sh reactive_avoid T2 1   # original 1.8m world
  ./run_baseline_trial.sh reactive_avoid T4 1   # original 2.8m world
  ./run_baseline_trial.sh reactive_avoid T6 1   # original 2.8m world

Stop the trial by ENTER once maneuver completes or a lasting stall occurs.
The runner has an 85-second upper time limit to avoid unbounded logging.
Confirm previous carts were removed before starting the next trial.
After checking trial1 results and removing models, run each test for trials
2 through 10 in the correct Gazebo world.

ANALYZE:
  source /opt/ros/humble/setup.bash
  source ~/thesis_cart/install/setup.bash
  /usr/bin/python3 ~/thesis_cart/analyze_baselines.py
  cat ~/thesis_cart/objective4_results/comparison_summary_T2_T4_T6.csv

The summary compares reactive_avoid, fcfs, proposed. Old no_comm/local_avoid
trial folders remain intact and are still included in detailed metrics.csv,
but are omitted from the three-way aggregate summary.

IMPORTANT LIMITATIONS
- Sensors are simulated directly from Gazebo model_states, not from a physical
  LiDAR scan or noisy/delayed onboard perception.
- An avoidance controller can make turns and passes, but may hold or deadlock.
- It does not reproduce the proposed inter-cart algorithm, and cannot infer
  neighbors' transmitted intents or user priority.
- Do NOT reuse historical no_comm statistics (24/30 footprint overlap etc.)
  for this new baseline. Report only new measured statistics.
- The previous FCFS vs proposed 5.0% and 9.0% comparisons are unchanged.
