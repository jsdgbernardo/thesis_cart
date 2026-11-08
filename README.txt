OBJECTIVE 4: NO-COMMUNICATION VS FCFS BASELINE TESTS
===================================================

Environment: Ubuntu 22.04 / ROS 2 Humble / Gazebo Classic 11
Requires the existing ~/thesis_cart workspace, generated/cart_N.urdf files,
working intercart_comms package, and two Gazebo world files.

1. Put no_comm_drive.py, run_baseline_trial.sh, and analyze_baselines.py
   directly in ~/thesis_cart/ and mark the runner executable:

   chmod +x ~/thesis_cart/run_baseline_trial.sh

2. First, close any leftover intercart_comm_node processes.
   Only when no trial is running:

   pgrep -af intercart_comm_node
   pkill -TERM -f '[/]intercart_comm_node'   # only if leftovers exist

3. Start Gazebo in terminal A. For T1-T2 use:
   source /opt/ros/humble/setup.bash
   source ~/thesis_cart/install/setup.bash
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   export ROS_DOMAIN_ID=23
   export ROS_LOCALHOST_ONLY=0
   gazebo --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so \
       ~/thesis_cart/src/intercart_comms/worlds/intercart_aisles.world

   For T3-T6 use:
   gazebo --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so \
       ~/thesis_cart/src/intercart_comms/worlds/intercart_wide_aisles.world

4. In terminal B, do one trial, e.g.:
   cd ~/thesis_cart
   ./run_baseline_trial.sh fcfs T2 1
   OR
   ./run_baseline_trial.sh no_comm T2 1

   Press Enter once the conflict is resolved, or when a collision is observed.
   Script saves bag and logs in objective4_results/T2_fcfs_trial1, etc.
   The runner will also try to delete the cart models after each trial.
   If /delete_entity works, you can keep Gazebo open with the same camera.
   Restart Gazebo before switching between 1.8m and 2.8m worlds, or if
   deletion did not succeed.

5. Confirm bag saved:
   ros2 bag info ~/thesis_cart/objective4_results/T2_fcfs_trial1/bag

6. Analyze the baseline and your previous proposed trial bags:
   source /opt/ros/humble/setup.bash
   source ~/thesis_cart/install/setup.bash
   /usr/bin/python3 ~/thesis_cart/analyze_baselines.py

   Open ~/thesis_cart/objective4_results/comparison_metrics.csv
   and ~/thesis_cart/objective4_results/comparison_summary_T2_T4_T6.csv

NOTES FOR THE THESIS
- FCFS uses the existing full collision avoidance and safety controller, but sets
  priority_class=0 for ALL carts; ties are resolved per your code's arrival-time,
  then cart-ID rule. This isolates removal of preferential right-of-way.
- No communication uses fixed 0.5 m/s motion commands. ROS is still used as a
  simulation interface, but no cart-state exchange, prediction, or coordination
  is performed. It is a deliberately simple OPEN-LOOP baseline.
- Run the SAME geometry and initial conditions in each comparison group.
- T2/T4/T6 isolate priority handling; T1/T3/T5 are optional further controls.
- A minimum distance measurement cannot alone prove physical contact.
  The analyzer additionally flags 2D rectangular footprint overlap as a proxy.
  Confirm collisions with Gazebo videos and label this proxy honestly.
- Automated time-to-clear is defined as time from first cart movement to all
  carts crossing their assigned 1.5 m exit line and aligned with their lanes.
  This is DIFFERENT from the earlier definition starting with conflict detection;
  if you use automated values, update the table caption accordingly.
- Automated waiting is time with near-zero commanded forward translation during
  the clearance interval. This may include turning in place.
- Failure to clear is "not_cleared", not zero seconds. Do not average failed
  trials as if they cleared. Report completion fraction separately.
- If only 3 trials per baseline and 10 per proposed, disclose the differing N.
