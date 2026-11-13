#pragma once

#include <ifc_tools/filter/kalman_filter_1d.hpp>
#include <submanager/submanager.hpp>

#include <ifc_msgs/msg/calibration_results.hpp>
#include <ifc_msgs/msg/delay.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace calibration_manager
{

struct DelayParameters
{
    double innov_gate_sd = 5.0;
    size_t max_rejections = 3;
    double process_noise_ms2_s = 0.25;
    double valid_sd_ms = 0.5;
    double invalid_sd_ms = 1.5;
};

class DelaySubmanager : public Submanager
{
public:
    explicit DelaySubmanager(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Submanager("calibration_manager_delay", options),
          _params(declare_delay_parameters()),
          _filter({.dQdt = _params.process_noise_ms2_s, .innov_gate_sd = _params.innov_gate_sd})
    {
        _subscription = create_subscription<ifc_msgs::msg::Delay>(
            declare_parameter<std::string>("delay_estimator.topic", "/delay_estimator/delay"),
            rclcpp::ServicesQoS(),
            [this](const ifc_msgs::msg::Delay& message) { receive(message); });
    }

    void update(const rclcpp::Time& now) override
    {
        if (!_filter.initialized())
        {
            return;
        }

        predict(now);

        const double sd = std::sqrt(_filter.state().P);

        if (sd < _params.valid_sd_ms)
        {
            _valid = true;
        }
        else if (sd > _params.invalid_sd_ms)
        {
            _valid = false;
        }
    }

    void fill(ifc_msgs::msg::CalibrationResults& results) const override
    {
        results.delay_status = _valid
            ? ifc_msgs::msg::CalibrationResults::DELAY_STATUS_VALID
            : ifc_msgs::msg::CalibrationResults::DELAY_STATUS_INVALID;
        results.delay = std::llround(_filter.state().x * NS_PER_MS);
        results.delay_sd = std::llround(std::sqrt(_filter.state().P) * NS_PER_MS);
    }

    void debug() const override
    {
        if (!_filter.initialized())
        {
            RCLCPP_WARN(get_logger(), "delay: filter not initialized");
            return;
        }

        RCLCPP_INFO(get_logger(), "delay: status: %s val:%.1f sd: %.2f ms", 
            _valid ? "valid" : "invalid", _filter.state().x, std::sqrt(_filter.state().P));
    }

private:
    static constexpr double NS_PER_MS = 1e6;

    const DelayParameters _params;
    ifc_tools::KalmanFilter1D<double> _filter;
    rclcpp::Subscription<ifc_msgs::msg::Delay>::SharedPtr _subscription;

    bool _valid = false;
    std::optional<rclcpp::Time> _last_time;
    size_t _rejections = 0;

    DelayParameters declare_delay_parameters()
    {
        DelayParameters params;
        params.innov_gate_sd = declare_parameter<double>("calibration_manager.delay.innov_gate_sd", params.innov_gate_sd);
        params.max_rejections = static_cast<size_t>(declare_parameter<int>("calibration_manager.delay.max_rejections", static_cast<int>(params.max_rejections)));
        params.process_noise_ms2_s = declare_parameter<double>("calibration_manager.delay.process_noise", params.process_noise_ms2_s);
        const std::vector<double> hysteresis = declare_parameter<std::vector<double>>(
            "calibration_manager.delay.validity_hysteresis",
            {params.valid_sd_ms, params.invalid_sd_ms});

        if (hysteresis.size() != 2 || hysteresis[0] > hysteresis[1])
        {
            throw std::invalid_argument("calibration_manager.delay.validity_hysteresis must be [valid_sd_ms, invalid_sd_ms] with valid <= invalid");
        }

        params.valid_sd_ms = hysteresis[0];
        params.invalid_sd_ms = hysteresis[1];
        return params;
    }

    void initialize(const rclcpp::Time& now, double delay, double delay_var)
    {
        RCLCPP_INFO(get_logger(), "delay: reset val:%.1f sd: %.2f ms", delay, std::sqrt(delay_var));
        _filter.initialize(delay, delay_var);
        _last_time = now;
        _rejections = 0;
        _valid = false;
    }

    void receive(const ifc_msgs::msg::Delay& message)
    {
        if (!message.valid)
        {
            return;
        }

        const rclcpp::Time timestamp = message.header.stamp;
        const double z = static_cast<double>(message.delay) / NS_PER_MS;
        const double sd = static_cast<double>(message.delay_sd) / NS_PER_MS;
        const double R = sd * sd;

        if (!_filter.initialized())
        {
            initialize(timestamp, z, R);
            return;
        }

        predict(timestamp);
        const bool sample_accepted = _filter.update(z, R);

        if (!sample_accepted)
        {
            _rejections++;

            if (_rejections >= _params.max_rejections)
            {
                initialize(timestamp, z, R);
            }

            return;
        }

        _rejections = 0;
    }

    void predict(const rclcpp::Time& now)
    {
        if (now > *_last_time)
        {
            _filter.predict((now - *_last_time).seconds());
            _last_time = now;
        }
    }
};

}
