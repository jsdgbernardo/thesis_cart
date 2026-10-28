
import time
import rclpy
from geometry_msgs.msg import Twist

SPEED = 0.07
DURATION = 30.0
RATE = 10.0

rclpy.init()
node = rclpy.create_node("short_motion_test")
pub = node.create_publisher(
    Twist, "/cmd_vel_nav", 10
)

try:
    print("Waiting for twist_mux subscriber...")

    while pub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.1)

    print("Ready. Ensure the path is clear.")
    input("Press Enter to begin the 15-second test...")

    cmd = Twist()
    cmd.linear.x = SPEED

    start = time.monotonic()

    while time.monotonic() - start < DURATION:
        pub.publish(cmd)
        time.sleep(1.0 / RATE)

finally:
    # Publish stop commands even if interrupted.
    stop = Twist()

    for _ in range(15):
        pub.publish(stop)
        time.sleep(0.1)

    node.destroy_node()
    rclpy.shutdown()

    print("Test finished. Stop command published.")

