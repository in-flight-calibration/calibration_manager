#pragma once

#include <submanager/delay_submanager.hpp>
#include <submanager/submanager.hpp>

#include <ifc_msgs/msg/calibration_results.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace calibration_manager
{

class CalibrationManager : public rclcpp::Node
{
public:
    CalibrationManager()
        : rclcpp::Node("calibration_manager"),
          _submanagers({std::make_shared<DelaySubmanager>()})
    {
        _publisher = create_publisher<ifc_msgs::msg::CalibrationResults>(
            declare_parameter<std::string>("calibration_manager.topic", "/calibration_manager/calibration_results"),
            rclcpp::ServicesQoS());

        _debug = declare_parameter<bool>("calibration_manager.debug", false);

        _timer = create_wall_timer(
            std::chrono::milliseconds(declare_parameter<int>("calibration_manager.update_ms", 500)),
            [this]() { update(); });
    }

    std::vector<rclcpp::Node::SharedPtr> children() const
    {
        return {_submanagers.begin(), _submanagers.end()};
    }

private:
    bool _debug = false;
    std::vector<std::shared_ptr<Submanager>> _submanagers;
    rclcpp::Publisher<ifc_msgs::msg::CalibrationResults>::SharedPtr _publisher;
    rclcpp::TimerBase::SharedPtr _timer;

    void update()
    {
        const rclcpp::Time time = now();
        ifc_msgs::msg::CalibrationResults results;
        results.header.stamp = time;

        for (const std::shared_ptr<Submanager>& submanager : _submanagers)
        {
            submanager->update(time);
            submanager->fill(results);

            if (_debug)
            {
                submanager->debug();
            }
        }

        _publisher->publish(results);
    }
};

}
