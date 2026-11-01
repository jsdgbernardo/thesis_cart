
from pathlib import Path
from itertools import combinations
import sqlite3
import math
import csv

from rclpy.serialization import deserialize_message
from gazebo_msgs.msg import ModelStates

results = Path.home() / "thesis_cart/objective4_results"
output = []

for run in sorted(results.glob("T*_trial*")):
    db_files = sorted((run / "bag").glob("*.db3"))

    if not db_files:
        continue

    minimum = float("inf")
    closest_pair = ""

    for db_file in db_files:
        with sqlite3.connect(db_file) as conn:
            row = conn.execute(
                "SELECT id FROM topics WHERE name = ?",
                ("/gazebo/model_states",)
            ).fetchone()

            if row is None:
                continue

            for (data,) in conn.execute(
                "SELECT data FROM messages "
                "WHERE topic_id = ? ORDER BY timestamp",
                (row[0],)
            ):
                msg = deserialize_message(data, ModelStates)

                carts = {
                    name: pose
                    for name, pose in zip(msg.name, msg.pose)
                    if name in ("cart_1", "cart_2", "cart_3")
                }

                for a, b in combinations(carts, 2):
                    pa = carts[a].position
                    pb = carts[b].position

                    distance = math.hypot(
                        pa.x - pb.x,
                        pa.y - pb.y
                    )

                    if distance < minimum:
                        minimum = distance
                        closest_pair = f"{a}-{b}"

    if minimum < float("inf"):
        output.append([
            run.name,
            closest_pair,
            round(minimum, 3)
        ])

with open(results / "minimum_distances.csv", "w") as f:
    writer = csv.writer(f)
    writer.writerow([
        "trial", "closest_pair", "min_center_distance_m"
    ])
    writer.writerows(output)

for row in output:
    print(*row)

print("Saved minimum_distances.csv")
