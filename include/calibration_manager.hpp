#pragma once

#include <rclcpp/rclcpp.hpp>

namespace calibration_manager
{

class CalibrationManager : public rclcpp::Node
{
public:
    CalibrationManager() : rclcpp::Node("calibration_manager")
    {
    }
};

}
