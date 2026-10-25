
#pragma once

#include <sensor_msgs/msg/laser_scan.hpp>

class ObstacleDetector
{
public:
    ObstacleDetector(float front_angle_deg, float side_angle_deg);

    void update(const sensor_msgs::msg::LaserScan &scan);

    float front_distance() const;
    float left_distance() const;
    float right_distance() const;

    bool has_valid_front_data() const;

private:
    float front_angle_;
    float side_angle_;

    float front_dist_;
    float left_dist_;
    float right_dist_;

    bool front_valid_ = false;
    bool left_valid_ = false;
    bool right_valid_ = false;
};
