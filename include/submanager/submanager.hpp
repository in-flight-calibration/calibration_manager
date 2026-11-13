#pragma once

#include <ifc_msgs/msg/calibration_results.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>

namespace calibration_manager
{

class Submanager : public rclcpp::Node
{
public:
    explicit Submanager(const std::string& name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : rclcpp::Node(name, options)
    {
    }

    virtual void update(const rclcpp::Time& now) = 0;
    virtual void fill(ifc_msgs::msg::CalibrationResults& results) const = 0;
    virtual void debug() const = 0;
};

}
