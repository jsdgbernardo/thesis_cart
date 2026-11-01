
#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source "$HOME/thesis_cart/install/setup.bash"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=23
export ROS_LOCALHOST_ONLY=0

TEST="${1:-}"
TRIAL="${2:-1}"

BASE="$HOME/thesis_cart"
RESULTS="$BASE/objective4_results"

# ------------------------------------------
# Scenario configuration
# ------------------------------------------

case "$TEST" in
  T1)
    N=2; WIDTH=1.8; HALF=6.0
    MODE=cross
    P1=0; P2=0; P3=0
    ;;
  T2)
    N=2; WIDTH=1.8; HALF=6.0
    MODE=cross
    P1=0; P2=1; P3=0
    ;;
  T3)
    N=3; WIDTH=2.8; HALF=12.0
    MODE=cross
    P1=0; P2=0; P3=0
    ;;
  T4)
    N=3; WIDTH=2.8; HALF=12.0
    MODE=cross
    P1=0; P2=0; P3=1
    ;;
  T5)
    N=2; WIDTH=2.8; HALF=12.0
    MODE=headon
    P1=0; P2=0; P3=0
    ;;
  T6)
    N=2; WIDTH=2.8; HALF=12.0
    MODE=headon
    P1=0; P2=1; P3=0
    ;;
  *)
    echo "Usage: $0 T1|T2|T3|T4|T5|T6 trial_number"
    exit 1
    ;;
esac

RUN="$RESULTS/${TEST}_trial${TRIAL}"

if [ -e "$RUN" ]; then
  echo "ERROR: $RUN already exists."
  echo "Choose another trial number."
  exit 1
fi

mkdir -p "$RUN"

printf "test=%s\ntrial=%s\nwidth=%s\npriority_classes=%s,%s,%s\n" \
  "$TEST" "$TRIAL" "$WIDTH" "$P1" "$P2" "$P3" \
  > "$RUN/setup.txt"

# ------------------------------------------
# Ensure Gazebo is already running
# ------------------------------------------

if ! ros2 service list | grep -q '^/spawn_entity$'; then
  echo "ERROR: Start the correct Gazebo world first."
  exit 1
fi

# ------------------------------------------
# Spawn carts
# ------------------------------------------

spawn_cart() {
  local id="$1"
  local x="$2"
  local y="$3"
  local yaw="$4"

  echo "Spawning Cart $id..."

  ros2 run gazebo_ros spawn_entity.py \
    -entity "cart_$id" \
    -file "$BASE/generated/cart_$id.urdf" \
    -x "$x" \
    -y "$y" \
    -z 0.05 \
    -Y "$yaw"
}

spawn_cart 1 -3.0 0.0 0.0

if [ "$MODE" = "headon" ]; then
  spawn_cart 2 3.0 0.0 3.14159
else
  spawn_cart 2 0.0 3.0 -1.5708
fi

if [ "$N" -eq 3 ]; then
  spawn_cart 3 3.0 0.0 3.14159
fi

# ------------------------------------------
# Start ROS bag recording
# ------------------------------------------

TOPICS=(
  /cart_states
  /gazebo/model_states
)

for ((i=1; i<=N; i++)); do
  TOPICS+=("/cart_${i}/cmd_vel")
done

echo "Starting recording..."

ros2 bag record \
  -o "$RUN/bag" \
  "${TOPICS[@]}" \
  > "$RUN/rosbag.log" 2>&1 &

BAG_PID=$!

sleep 3

# ------------------------------------------
# Start cart nodes
# ------------------------------------------

PIDS=()

start_cart() {
  local id="$1"
  local priority="$2"

  echo "Starting Cart $id, priority=$priority"

  "$BASE/install/intercart_comms/lib/intercart_comms/intercart_comm_node" \
    --ros-args \
    -p cart_id:="$id" \
    -p priority_class:="$priority" \
    -p gazebo_model_name:="cart_$id" \
    -p velocity_x:=0.5 \
    -p velocity_y:=0.0 \
    -p safety_radius:=1.2 \
    -p prediction_horizon:=3.0 \
    -p aisle_width:="$WIDTH" \
    -p world_half_width:="$HALF" \
    > "$RUN/cart${id}.log" 2>&1 &

  PIDS+=("$!")
}

# Stop everything cleanly when script exits
cleanup() {
  trap - EXIT

  echo
  echo "Stopping cart nodes and recording..."

  # Stop the cart nodes.
  # SIGTERM is more reliable for background ROS2 nodes
  # than SIGINT sent from a Bash script.
  for pid in "${PIDS[@]}"; do
    kill -TERM "$pid" 2>/dev/null || true
  done

  # Stop the ROS bag recorder gracefully
  # so it writes remaining data and metadata.
  kill -TERM "$BAG_PID" 2>/dev/null || true

  # Wait for all processes to shut down.
  for pid in "${PIDS[@]}"; do
    wait "$pid" 2>/dev/null || true
  done

  wait "$BAG_PID" 2>/dev/null || true

  echo
  echo "===================================="
  echo "Trial finished successfully!"
  echo "Saved to: $RUN"
  echo "===================================="
}

trap cleanup EXIT

start_cart 1 "$P1"
start_cart 2 "$P2"

if [ "$N" -eq 3 ]; then
  start_cart 3 "$P3"
fi

echo
echo "===================================="
echo "RUNNING $TEST / TRIAL $TRIAL"
echo "===================================="
echo "Observe the carts in Gazebo."
echo "Once the scenario is complete,"
echo "press ENTER to stop recording."
echo

read -r -p "Press ENTER when finished: " _
