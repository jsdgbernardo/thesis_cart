
#include "collision_avoidance/obstacle_detector.hpp"

#include <algorithm>
#include <cmath>

ObstacleDetector::ObstacleDetector(
    float front_angle_deg,
    float side_angle_deg)
    : front_angle_(front_angle_deg * M_PI / 180.0f),
      side_angle_(side_angle_deg * M_PI / 180.0f),
      front_dist_(0.0f),
      left_dist_(0.0f),
      right_dist_(0.0f)
{
}

void ObstacleDetector::update(
    const sensor_msgs::msg::LaserScan &scan)
{
    front_valid_ = false;
    left_valid_ = false;
    right_valid_ = false;

    front_dist_ = scan.range_max;
    left_dist_ = 0.0f;
    right_dist_ = 0.0f;

    for (size_t i = 0; i < scan.ranges.size(); ++i) {

        float raw_angle =
            scan.angle_min + i * scan.angle_increment;

        // Normalize angle to [-pi, pi]
        float angle = std::atan2(
            std::sin(raw_angle),
            std::cos(raw_angle)
        );

        float dist = scan.ranges[i];

        // Reject invalid measurements
        if (!std::isfinite(dist) ||
            dist < scan.range_min ||
            dist > scan.range_max) {
            continue;
        }

        if (std::abs(angle) <= front_angle_) {

            front_dist_ = std::min(front_dist_, dist);
            front_valid_ = true;

        } else if (
            angle > front_angle_ &&
            angle <= front_angle_ + side_angle_) {

            if (!left_valid_) {
                left_dist_ = dist;
            } else {
                left_dist_ = std::min(left_dist_, dist);
            }

            left_valid_ = true;

        } else if (
            angle < -front_angle_ &&
            angle >= -front_angle_ - side_angle_) {

            if (!right_valid_) {
                right_dist_ = dist;
            } else {
                right_dist_ = std::min(right_dist_, dist);
            }

            right_valid_ = true;
        }
    }
}

float ObstacleDetector::front_distance() const
{
    return front_dist_;
}

float ObstacleDetector::left_distance() const
{
    return left_dist_;
}

float ObstacleDetector::right_distance() const
{
    return right_dist_;
}

bool ObstacleDetector::has_valid_front_data() const
{
    return front_valid_;
}
