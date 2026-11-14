# Copilot instructions: calibration_manager

Header-only C++20 ROS 2 (lyrical) node of the `ifc` colcon workspace (this repo lives at `<ws>/src/calibration_manager`). Build and test from the workspace root:

```bash
source /opt/ros/lyrical/setup.bash
colcon build --packages-up-to calibration_manager
source install/setup.bash
ros2 run calibration_manager calibration_manager --ros-args --params-file config.yaml
```

## Architecture
- `CalibrationManager` (`include/calibration_manager.hpp`) owns the submanagers, exposes them with `children()` (added to the same `SingleThreadedExecutor`; submanagers are nevertheless thread-safe) and every `calibration_manager.update_ms` calls each `update(now)` and `fill(results)`, then publishes `ifc_msgs/CalibrationResults` on `calibration_manager.topic`. With `calibration_manager.debug` (default `false`) it calls each `debug()` after `fill`.
- `Submanager` (`include/submanager/submanager.hpp`): abstract `rclcpp::Node` with `update(const rclcpp::Time&)`, `fill(CalibrationResults&) const` and `debug() const`, which logs the state in one line (`RCLCPP_INFO` when valid, `RCLCPP_WARN` otherwise). One submanager per calibrated quantity in `include/submanager/`.
- `DelaySubmanager` (`include/submanager/delay_submanager.hpp`, node `calibration_manager_delay`): subscribes `delay_estimator.topic`, skips `valid == false` and filters the delay with `ifc_tools::KalmanFilter1D<double>` in ms (`z = delay`, `R = delay_sd²` converted from ns). `receive(message)` predicts to `message.header.stamp`, `update(now)` to the node clock. Initialises on the first valid sample (`x = z`, `P = R`). Per sample: predict → `update`, which rejects when `|z − x|/√(P + R) > innov_gate_sd`; after `max_rejections` consecutive rejections it resets to the sample and becomes invalid (latency jump). `update(now)`: predict, then valid when `√P < validity_hysteresis[0]`, invalid only when `√P > validity_hysteresis[1]`. `fill` writes `delay_status`, `delay` and `delay_sd` (ns). `debug()` logs delay ± sd in ms (warns while uninitialized). A reset clears the rejections and the valid status. A `mutable std::mutex _mutex` guards the filter state: `update`, `fill`, `debug` and the subscription callback lock it; the private helpers `initialize` and `predict` expect it held.
- Parameters `calibration_manager.delay.{innov_gate_sd, max_rejections, process_noise (ms²/s), validity_hysteresis ([valid, invalid] sd in ms)}` are read into `DelayParameters`; the constructor throws `std::invalid_argument` unless the hysteresis has two values with valid ≤ invalid. Header defaults equal root `config.yaml`.
- There are no tests yet.
