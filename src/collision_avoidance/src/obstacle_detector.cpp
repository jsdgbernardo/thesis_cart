#include "collision_avoidance/obstacle_detector.hpp"
#include <algorithm>
#include <cmath>

using namespace std;

ObstacleDetector::ObstacleDetector(float front_angle_deg, float side_angle_deg)
: front_angle_(front_angle_deg * M_PI / 180.0),
  side_angle_(side_angle_deg * M_PI / 180.0),
  front_dist_(10.0),
  left_dist_(10.0),
  right_dist_(10.0)
{
}

void ObstacleDetector::update(const sensor_msgs::msg::LaserScan &scan)
{
  front_dist_ = left_dist_ = right_dist_ = scan.range_max;

  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    float angle = scan.angle_min + i * scan.angle_increment;
    float dist = scan.ranges[i];

    if (isnan(dist) || isinf(dist)) continue;

    if (abs(angle) < front_angle_) {
      front_dist_ = min(front_dist_, dist);
    } else if (angle > front_angle_ && angle < front_angle_ + side_angle_) {
      left_dist_ = min(left_dist_, dist);
    } else if (angle < -front_angle_ && angle > -front_angle_ - side_angle_) {
      right_dist_ = min(right_dist_, dist);
    }
  }
}

float ObstacleDetector::front_distance() const { return front_dist_; }
float ObstacleDetector::left_distance()  const { return left_dist_; }
float ObstacleDetector::right_distance() const { return right_dist_; }
