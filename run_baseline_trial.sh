#!/usr/bin/env bash
# Usage: ./run_baseline_trial.sh no_comm|fcfs T1|T2|T3|T4|T5|T6 trial_number
set -Eeo pipefail

BASE="$HOME/thesis_cart"
source /opt/ros/humble/setup.bash
source "$BASE/install/setup.bash"

set -u

export ROS_DOMAIN_ID=23
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

METHOD="${1:-}"
TEST="${2:-}"
TRIAL="${3:-1}"

if [[ "$METHOD" != "no_comm" && "$METHOD" != "fcfs" ]]; then
  echo 'Usage: ./run_baseline_trial.sh no_comm|fcfs T1|T2|T3|T4|T5|T6 trial_number'
  exit 1
fi
if ! [[ "$TRIAL" =~ ^[1-9][0-9]*$ ]]; then
  echo 'Trial number must be a positive integer.'
  exit 1
fi

case "$TEST" in
  T1|T2) COUNT=2; WIDTH=1.8; HALF=6.0; SCENARIO=cross ;;
  T3|T4) COUNT=3; WIDTH=2.8; HALF=12.0; SCENARIO=cross ;;
  T5|T6) COUNT=2; WIDTH=2.8; HALF=12.0; SCENARIO=headon ;;
  *) echo 'Invalid test. Select T1, T2, T3, T4, T5, or T6.'; exit 1 ;;
esac

RUN="$BASE/objective4_results/${TEST}_${METHOD}_trial${TRIAL}"
if [[ -e "$RUN" ]]; then
  echo "ERROR: $RUN already exists. Use a new trial number."
  exit 1
fi

if pgrep -f '[/]intercart_comm_node' >/dev/null; then
  echo 'ERROR: An old intercart_comm_node is running. Stop old nodes before starting.'
  echo "Check with: pgrep -af intercart_comm_node"
  exit 1
fi

SERVICES="$(ros2 service list)"

if ! grep -Fxq '/spawn_entity' <<< "$SERVICES"; then
  echo 'ERROR: Gazebo spawn service missing. Start the corresponding world first.'
  exit 1
fi

if [[ "$METHOD" == fcfs ]]; then
  NODE_BIN="$(ros2 pkg prefix intercart_comms)/lib/intercart_comms/intercart_comm_node"
  if [[ ! -x "$NODE_BIN" ]]; then
    echo "ERROR: Cannot find executable: $NODE_BIN"
    exit 1
  fi
fi

mkdir -p "$RUN"
printf 'method=%s\ntest=%s\ntrial=%s\nnumber_of_carts=%s\nscenario=%s\naisle_width_m=%s\nworld_half_width_m=%s\ncommanded_speed_mps=0.5\n' \
  "$METHOD" "$TEST" "$TRIAL" "$COUNT" "$SCENARIO" "$WIDTH" "$HALF" > "$RUN/setup.txt"
if [[ "$METHOD" == fcfs ]]; then
  printf 'priority_all_zero=yes\n' >> "$RUN/setup.txt"
else
  printf 'intercart_messages=disabled\n' >> "$RUN/setup.txt"
fi

PIDS=()
BAG_PID=''
cleanup() {
  trap - EXIT
  echo
  echo 'Stopping controllers and ROS bag...'
  for pid in "${PIDS[@]}"; do
    kill -TERM "$pid" 2>/dev/null || true
  done
  if [[ -n "$BAG_PID" ]]; then
    kill -TERM "$BAG_PID" 2>/dev/null || true
  fi
  for pid in "${PIDS[@]}"; do
    wait "$pid" 2>/dev/null || true
  done
  if [[ -n "$BAG_PID" ]]; then
    wait "$BAG_PID" 2>/dev/null || true
  fi
  # Remove carts after recording; Gazebo may remain open with camera unchanged.
  # /delete_entity is normally provided by libgazebo_ros_factory.so.
  SERVICES="$(ros2 service list 2>/dev/null || true)"

  if grep -Fxq '/delete_entity' <<< "$SERVICES"; then
    for ((i=1; i<=COUNT; i++)); do
      timeout 8s ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity \
        "{name: 'cart_$i'}" >> "$RUN/despawn.log" 2>&1 || true
    done
  else
    echo 'WARNING: /delete_entity unavailable; restart Gazebo before next trial.'
  fi
  echo "Saved: $RUN"
  echo 'Check: ros2 bag info '"$RUN/bag"
  echo 'Next trial can reuse Gazebo IF all previous cart models were deleted.'
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

spawn() {
  local id="$1" x="$2" y="$3" yaw="$4"
  echo "Spawning cart_$id at ($x, $y, $yaw)..."
  ros2 run gazebo_ros spawn_entity.py \
    -entity "cart_$id" \
    -file "$BASE/generated/cart_${id}.urdf" \
    -x "$x" -y "$y" -z 0.05 -Y "$yaw"
}

spawn 1 -3.0 0.0 0.0
if [[ "$SCENARIO" == headon ]]; then
  spawn 2 3.0 0.0 3.14159
else
  spawn 2 0.0 3.0 -1.5708
fi
if [[ "$COUNT" == 3 ]]; then
  spawn 3 3.0 0.0 3.14159
fi

TOPICS=(/gazebo/model_states /cart_states)
for ((i=1; i<=COUNT; i++)); do
  TOPICS+=("/cart_${i}/cmd_vel")
done

ros2 bag record -o "$RUN/bag" "${TOPICS[@]}" > "$RUN/rosbag.log" 2>&1 &
BAG_PID=$!
sleep 3

if [[ "$METHOD" == no_comm ]]; then
  /usr/bin/python3 "$BASE/no_comm_drive.py" --carts "$COUNT" --speed 0.5 \
    > "$RUN/no_comm_driver.log" 2>&1 &
  PIDS+=("$!")
else
  for ((i=1; i<=COUNT; i++)); do
    "$NODE_BIN" --ros-args \
      -p cart_id:="$i" \
      -p priority_class:=0 \
      -p gazebo_model_name:="cart_$i" \
      -p velocity_x:=0.5 \
      -p velocity_y:=0.0 \
      -p safety_radius:=1.2 \
      -p prediction_horizon:=3.0 \
      -p aisle_width:="$WIDTH" \
      -p world_half_width:="$HALF" \
      > "$RUN/cart${i}.log" 2>&1 &
    PIDS+=("$!")
  done
fi

echo
printf 'RUNNING %s / %s / trial %s\n' "$METHOD" "$TEST" "$TRIAL"
echo 'Observe Gazebo. When the interaction is complete (or a collision/stall occurs), press ENTER.'
read -r -p 'Press ENTER to stop this trial: ' _
