// Copyright (c) 2019 Rafael Silva (gimbas)
//
// Licensed under the MIT license: https://opensource.org/licenses/MIT
// Permission is granted to use, copy, modify, and redistribute the work.
// Full license information available in the project LICENSE file.
//

#ifndef FOCBOX_UNITY_DRIVER_H_
#define FOCBOX_UNITY_DRIVER_H_

#include <string>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <controller_manager/controller_manager.h>

#include <sstream>

#include <std_msgs/Float64.h>

#include <boost/optional.hpp>
#include <boost/scoped_ptr.hpp>

#include "focbox_unity_driver/focbox_unity_interface.h"
#include "focbox_unity_driver/focbox_unity_packet.h"

namespace focbox_unity_driver
{

class FocboxUnityDriver : public hardware_interface::RobotHW
{
public:

  FocboxUnityDriver(ros::NodeHandle nh, ros::NodeHandle private_nh);

private:
  // interface to the FOCBOX
  FocboxUnityInterface focbox_;

  void controllerRoutine(const boost::shared_ptr<FocboxUnityPacketValues const>& values);
  void publishTopics(const boost::shared_ptr<FocboxUnityPacketValues const>& values);

  void focboxUnityPacketCB(const boost::shared_ptr<FocboxUnityPacket const>& packet);
  void focboxUnityErrorCB(const std::string& error);

  // limits onFOCBOX Unitycommands
  struct CommandLimit
  {
    CommandLimit(const ros::NodeHandle& nh, const std::string& str,
                 const boost::optional<double>& min_lower = boost::optional<double>(),
                 const boost::optional<double>& max_upper = boost::optional<double>());
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
  ros::Publisher motor1_velocity_pub_;
  ros::Publisher motor1_current_pub_;
  ros::Publisher motor1_position_pub_;
  ros::Publisher motor1_temperature_pub_;

  ros::Publisher motor2_velocity_pub_;
  ros::Publisher motor2_current_pub_;
  ros::Publisher motor2_position_pub_;
  ros::Publisher motor2_temperature_pub_;

  ros::Publisher driver_temperature1_pub_;
  ros::Publisher driver_temperature2_pub_;
  ros::Publisher driver_current_in_pub_;
  ros::Publisher driver_voltage_in_pub_;
  ros::Publisher driver_fault_pub_;

  ros::Timer update_timer_;
  ros::Time time_now_;
  ros::Time time_last_;
  ros::Duration elapsed_time_;
  boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
  hardware_interface::JointStateInterface _joint_state_interface;
  hardware_interface::VelocityJointInterface _effort_joint_interface;

  double _cmd[2];
  double _pos[2];
  double _vel[2];
  double _eff[2];

  double pole_pairs_;
  double counts_;

  // driver modes (possible states)
  typedef enum {
    MODE_INITIALIZING,
    MODE_OPERATING
  } driver_mode_t;

  // other variables
  driver_mode_t driver_mode_;           ///< driver state machine mode (state)
  int fw_version_major_;                ///< firmware major version reported by focbox
  int fw_version_minor_;                ///< firmware minor version reported by focbox

  // ROS callbacks
  void updateTimerCB(const ros::TimerEvent& event);
};

} // namespace focbox_unity_driver

#endif // FOCBOX_UNITY_DRIVER_H_
