// Copyright (c) 2019 Rafael Silva (gimbas)
//
// Licensed under the MIT license: https://opensource.org/licenses/MIT
// Permission is granted to use, copy, modify, and redistribute the work.
// Full license information available in the project LICENSE file.
//
// Modifications notice --

#ifndef FOCBOX_UNITY_DRIVER_H_
#define FOCBOX_UNITY_DRIVER_H_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/logging.hpp>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

// #include "hardware_interface/joint_command_interface.hpp" // ros1
#include "hardware_interface/handle.hpp"                                 // ros2
#include "hardware_interface/hardware_info.hpp"                          // ros2
#include "hardware_interface/system_interface.hpp"                       // ros2
#include "hardware_interface/types/hardware_interface_return_values.hpp" // ros2
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
// #include "hardware_interface/joint_state_interface.hpp" // ros1
// #include "hardware_interface/robot_hw.hpp" // ros1

#include <controller_manager/controller_manager.hpp>
#include <memory>
#include <sstream>

#include <std_msgs/msg/float64.hpp>

#include <boost/optional.hpp>
#include <boost/scoped_ptr.hpp>

#include "focbox_unity_interface.h"
#include "focbox_unity_packet.h"

namespace focbox_unity_driver
{

  class FocboxUnityDriver : public hardware_interface::SystemInterface //, public rclcpp::Node
  {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(FocboxUnityDriver);

    FocboxUnityDriver();
    // FocboxUnityDriver(rclcpp::Node::SharedPtr nh, rclcpp::Node::SharedPtr private_nh);

    // rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo &info) override;

    // Interfaces to and from hardware
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::return_type read(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;
    hardware_interface::return_type write(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    // interface to the FOCBOX
    FocboxUnityInterface focbox_;

    void controllerRoutine(const boost::shared_ptr<FocboxUnityPacketValues const> &values);
    void publishTopics(const boost::shared_ptr<FocboxUnityPacketValues const> &values);

    void focboxUnityPacketCB(const boost::shared_ptr<FocboxUnityPacket const> &packet);
    void focboxUnityErrorCB(const std::string &error);

    // limits on FOCBOX Unity commands
    struct CommandLimit
    {
      CommandLimit(const rclcpp::Node::SharedPtr &node, const std::string &str,
                   const boost::optional<double> &min_lower = boost::optional<double>(),
                   const boost::optional<double> &max_upper = boost::optional<double>());
      double clip(double value);
      std::string name;
      boost::optional<double> lower;
      boost::optional<double> upper;
    };
    CommandLimit duty_cycle_limit_;
    CommandLimit current_limit_;
    CommandLimit brake_limit_;
    CommandLimit speed_limit_;
    CommandLimit position_limit_;

    // ROS services
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor1_velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor1_current_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor1_position_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor1_temperature_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor2_velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor2_current_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor2_position_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor2_temperature_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr driver_temperature1_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr driver_temperature2_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr driver_current_in_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr driver_voltage_in_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr driver_fault_pub_;

    rclcpp::TimerBase::SharedPtr update_timer_;
    rclcpp::Time time_now_;
    rclcpp::Time time_last_;
    rclcpp::Duration elapsed_time_;
    std::shared_ptr<controller_manager::ControllerManager> controller_manager_;

    double _cmd[2];
    double _pos[2];
    double _vel[2];
    double _eff[2];

    double pole_pairs_;
    double counts_;

    // driver modes (possible states)
    typedef enum
    {
      MODE_INITIALIZING,
      MODE_OPERATING
    } driver_mode_t;

    // other variables
    driver_mode_t driver_mode_; ///< driver state machine mode (state)
    int fw_version_major_;      ///< firmware major version reported by focbox
    int fw_version_minor_;      ///< firmware minor version reported by focbox

    // ROS callbacks
    // void updateTimerCB(const ros::TimerEvent& event); // http://wiki.ros.org/roscpp/Overview/Timers // ros1
    void updateTimerCB(); // ros2

    // Store the command for the simulated robot
    std::vector<double> hw_commands_;
    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;
    std::vector<double> hw_effort_;
  };

  // class FocboxUnityDriverInterface : public hardware_interface::SystemInterface
  // {
  // public:
  //   // rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  //   hardware_interface::CallbackReturn on_init(
  //       const hardware_interface::HardwareInfo &info) override;

  //   // Interfaces to and from hardware
  //   std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  //   std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  //   hardware_interface::CallbackReturn on_activate(
  //       const rclcpp_lifecycle::State &previous_state) override;

  //   hardware_interface::CallbackReturn on_deactivate(
  //       const rclcpp_lifecycle::State &previous_state) override;

  //   hardware_interface::return_type read(
  //       const rclcpp::Time &time, const rclcpp::Duration &period) override;
  //   hardware_interface::return_type write(
  //       const rclcpp::Time &time, const rclcpp::Duration &period) override;

  // private:
  //   // Store the command for the simulated robot
  //   std::vector<double> hw_commands_;
  //   std::vector<double> hw_positions_;
  //   std::vector<double> hw_velocities_;
  // }

} // namespace focbox_unity_driver

#endif // FOCBOX_UNITY_DRIVER_H_
