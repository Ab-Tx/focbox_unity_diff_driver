// Copyright (c) 2019 Rafael Silva (gimbas)
//
// Licensed under the MIT license: https://opensource.org/licenses/MIT
// Permission is granted to use, copy, modify, and redistribute the work.
// Full license information available in the project LICENSE file.
//

#include "focbox_unity_driver/focbox_unity_driver.h"

#include <cassert>
#include <cmath>
#include <sstream>

#include <boost/bind.hpp>

namespace focbox_unity_driver
{

FocboxUnityDriver::FocboxUnityDriver(rclcpp::NodeHandle nh, rclcpp::NodeHandle private_nh) :
  focbox_(std::string(),
        boost::bind(&FocboxUnityDriver::focboxUnityPacketCB, this, _1),
        boost::bind(&FocboxUnityDriver::focboxUnityErrorCB, this, _1)),
  duty_cycle_limit_(private_nh, "duty_cycle", -1.0, 1.0), current_limit_(private_nh, "current"),
  brake_limit_(private_nh, "brake"), speed_limit_(private_nh, "speed"),
  position_limit_(private_nh, "position"),
  driver_mode_(MODE_INITIALIZING), fw_version_major_(-1), fw_version_minor_(-1)
{
  // get focbox serial port address
  std::string port;
  if (!private_nh.getParam("port", port))
  {
    RCLCPP_FATAL("FOCBOX communication port parameter required.");
    rclcpp::shutdown();
    return;
  }

  // attempt to connect to the serial port
  try
  {
    focbox_.connect(port);
  }
  catch(SerialException exception)
  {
    RCLCPP_FATAL("Failed to connect to the FOCBOX, %s.", exception.what());
    rclcpp::shutdown();
    return;
  }

  counts_ = 90.0f;
  pole_pairs_ = 14.0f;

  // connect and register the joint state interface
  hardware_interface::JointStateHandle state_handle_rw("motor1", &_pos[0], &_vel[0], &_eff[0]);
  _joint_state_interface.registerHandle(state_handle_rw);

  hardware_interface::JointStateHandle state_handle_lw("motor2", &_pos[1], &_vel[1], &_eff[1]);
  _joint_state_interface.registerHandle(state_handle_lw);

  registerInterface(&_joint_state_interface);

  // connect and register the joint velocity interface
  hardware_interface::JointHandle handle_rw(state_handle_rw, &_cmd[0]);
  _effort_joint_interface.registerHandle(handle_rw);

  hardware_interface::JointHandle handle_lw(state_handle_lw, &_cmd[1]);
  _effort_joint_interface.registerHandle(handle_lw);

  registerInterface(&_effort_joint_interface);

  controller_manager_.reset(new controller_manager::ControllerManager(this, nh));

  // create focbox state (telemetry) publisher
  //state_pub_ = nh.advertise<focbox_unity_msgs::FocboxUnityStateStamped>("focbox_unity/state", 10);
  motor1_velocity_pub_ = nh.advertise<std_msgs::Float64>("focbox_unity_driver/motor1/w_velocity", 10);
  motor1_current_pub_ = nh.advertise<std_msgs::Float64>("focbox_unity_driver/motor1/current", 10);
  motor1_position_pub_ = nh.advertise<std_msgs::Float64>("focbox_unity_driver/motor1/w_position", 10);
  motor1_temperature_pub_ = nh.advertise<std_msgs::Float64>("focbox_unity_driver/motor1/temperature", 10);

  motor2_velocity_pub_ = nh.advertise<std_msgs::Float64>("focbox_unity_driver/motor2/w_velocity", 10);
  motor2_current_pub_ = nh.advertise<std_msgs::Float64>("focbox_unity_driver/motor2/current", 10);
  motor2_position_pub_ = nh.advertise<std_msgs::Float64>("focbox_unity_driver/motor2/w_position", 10);
  motor2_temperature_pub_ = nh.advertise<std_msgs::Float64>("focbox_unity_driver/motor2/temperature", 10);

  driver_temperature1_pub_ = nh.advertise<std_msgs::Float64>("focbox_unity_driver/driver/temperature1", 10);
  driver_temperature2_pub_ = nh.advertise<std_msgs::Float64>("focbox_unity_driver/driver/temperature2", 10);
  driver_current_in_pub_ = nh.advertise<std_msgs::Float64>("focbox_unity_driver/driver/current", 10);
  driver_voltage_in_pub_ = nh.advertise<std_msgs::Float64>("focbox_unity_driver/driver/voltage", 10);
  driver_fault_pub_ = nh.advertise<std_msgs::Float64>("focbox_unity_driver/driver/fault", 10);

  // create a 50Hz timer, used for state machine & pollingFOCBOX Unitytelemetry
  update_timer_ = nh.createTimer(rclcpp::Duration(1.0/50.0), &FocboxUnityDriver::updateTimerCB, this);
}

void FocboxUnityDriver::updateTimerCB(const rclcpp::TimerEvent& event)
{
  //FOCBOX Unityinterface should not unexpectedly disconnect, but test for it anyway
  if(!focbox_.isConnected())
  {
    RCLCPP_FATAL("Unexpectedly disconnected from serial port.");
    update_timer_.stop();
    rclcpp::shutdown();
    return;
  }

  /*
   * Driver state machine, modes:
   *  INITIALIZING - request and wait for focbox version
   *  OPERATING - receiving commands from subscriber topics
   */
  if (driver_mode_ == MODE_INITIALIZING)
  {
    // request version number, return packet will update the internal version numbers
    focbox_.requestFWVersion();
    if (fw_version_major_ >= 0 && fw_version_minor_ >= 0)
    {
      RCLCPP_INFO("Connected to FOCBOX Unity with firmware version %d.%d",
               fw_version_major_, fw_version_minor_);

      driver_mode_ = MODE_OPERATING;

      focbox_.clrTach();
    }
  }
  else if (driver_mode_ == MODE_OPERATING)
  {
    // poll for focbox state (telemetry)
    focbox_.requestState();
  }
  else
  {
    // unknown mode, how did that happen?
    assert(false && "unknown driver mode");
  }
}

void FocboxUnityDriver::controllerRoutine(const boost::shared_ptr<FocboxUnityPacketValues const>& values)
{
  time_last_ = time_now_;
  time_now_ = rclcpp::Time::now();

  elapsed_time_ = rclcpp::Duration(time_now_.toSec() - time_last_.toSec());

  _eff[0] = values->motor_current1();
  _eff[1] = values->motor_current2();
  _vel[0] = ((values->rpm1() / pole_pairs_) / 60.0f) * (2.0f * M_PI); // angular velocity (rad/s)
  _vel[1] = ((values->rpm2() / pole_pairs_) / 60.0f) * (2.0f * M_PI); // angular velocity (rad/s)
  _pos[0] = values->tachometer1() * ((2.0f * M_PI) / counts_); // angular position (rads)
  _pos[1] = values->tachometer2() * ((2.0f * M_PI) / counts_); // angular position (rads)

  controller_manager_->update(time_now_, elapsed_time_);

  focbox_.setSpeed((_cmd[0] / (2.0f * M_PI)) * 60.0f * pole_pairs_, (_cmd[1] / (2.0f * M_PI)) * 60.0f * pole_pairs_);
}

void FocboxUnityDriver::publishTopics(const boost::shared_ptr<FocboxUnityPacketValues const>& values)
{
    // ------------------------------------------
    std_msgs::Float64 msg_holder;

    // ------------------------------------------
    msg_holder.data = ((values->rpm1() / pole_pairs_) / 60.0f) * (2.0f * M_PI);
    motor1_velocity_pub_.publish(msg_holder);

    msg_holder.data = values->motor_current1();
    motor1_current_pub_.publish(msg_holder);

    msg_holder.data = values->tachometer1() * ((2.0f * M_PI) / counts_);
    motor1_position_pub_.publish(msg_holder);

    msg_holder.data = values->temp_mot1();
    motor1_temperature_pub_.publish(msg_holder);

    // ------------------------------------------
    msg_holder.data = ((values->rpm2() / pole_pairs_) / 60.0f) * (2.0f * M_PI);
    motor2_velocity_pub_.publish(msg_holder);

    msg_holder.data = values->motor_current2();
    motor2_current_pub_.publish(msg_holder);

    msg_holder.data = values->tachometer2() * ((2.0f * M_PI) / counts_);
    motor2_position_pub_.publish(msg_holder);

    msg_holder.data = values->temp_mot2();
    motor2_temperature_pub_.publish(msg_holder);

    // ------------------------------------------
    msg_holder.data = values->temp_fet2();
    driver_temperature1_pub_.publish(msg_holder);

    msg_holder.data = values->temp_fet2();
    driver_temperature2_pub_.publish(msg_holder);

    msg_holder.data = values->current_in();
    driver_current_in_pub_.publish(msg_holder);

    msg_holder.data = values->v_in();
    driver_voltage_in_pub_.publish(msg_holder);

    msg_holder.data = values->fault_code();
    driver_fault_pub_.publish(msg_holder);
}

void FocboxUnityDriver::focboxUnityPacketCB(const boost::shared_ptr<FocboxUnityPacket const>& packet)
{
  if(packet->name() == "Values")
  {
    controllerRoutine(boost::dynamic_pointer_cast<FocboxUnityPacketValues const>(packet));

    publishTopics(boost::dynamic_pointer_cast<FocboxUnityPacketValues const>(packet));
  }
  else if (packet->name() == "FWVersion")
  {
    boost::shared_ptr<FocboxUnityPacketFWVersion const> fw_version =
      boost::dynamic_pointer_cast<FocboxUnityPacketFWVersion const>(packet);

    fw_version_major_ = fw_version->fwMajor();
    fw_version_minor_ = fw_version->fwMinor();
  }
}

void FocboxUnityDriver::focboxUnityErrorCB(const std::string& error)
{
  RCLCPP_ERROR("%s", error.c_str());
}

FocboxUnityDriver::CommandLimit::CommandLimit(const rclcpp::NodeHandle& nh, const std::string& str, const boost::optional<double>& min_lower, const boost::optional<double>& max_upper) :  name(str)
{
  // check if user's minimum value is outside of the range min_lower to max_upper
  double param_min;
  if (nh.getParam(name + "_min", param_min))
  {
    if (min_lower && param_min < *min_lower)
    {
      lower = *min_lower;
      RCLCPP_WARN_STREAM("Parameter " << name << "_min (" << param_min <<
                      ") is less than the feasible minimum (" << *min_lower << ").");
    }
    else if (max_upper && param_min > *max_upper)
    {
      lower = *max_upper;
      RCLCPP_WARN_STREAM("Parameter " << name << "_min (" << param_min <<
                      ") is greater than the feasible maximum (" << *max_upper << ").");
    }
    else
    {
      lower = param_min;
    }
  }
  else if (min_lower)
  {
    lower = *min_lower;
  }

  // check if the uers' maximum value is outside of the range min_lower to max_upper
  double param_max;
  if (nh.getParam(name + "_max", param_max))
  {
    if (min_lower && param_max < *min_lower)
    {
      upper = *min_lower;
      RCLCPP_WARN_STREAM("Parameter " << name << "_max (" << param_max <<
                      ") is less than the feasible minimum (" << *min_lower << ").");
    }
    else if (max_upper && param_max > *max_upper)
    {
      upper = *max_upper;
      RCLCPP_WARN_STREAM("Parameter " << name << "_max (" << param_max <<
                      ") is greater than the feasible maximum (" << *max_upper << ").");
    }
    else
    {
      upper = param_max;
    }
  }
  else if (max_upper)
  {
    upper = *max_upper;
  }

  // check for min > max
  if (upper && lower && *lower > *upper)
  {
    RCLCPP_WARN_STREAM("Parameter " << name << "_max (" << *upper
                    << ") is less than parameter " << name << "_min (" << *lower << ").");
    double temp(*lower);
    lower = *upper;
    upper = temp;
  }

  std::ostringstream oss;
  oss << "  " << name << " limit: ";
  if (lower) oss << *lower << " "; else oss << "(none) ";
  if (upper) oss << *upper; else oss << "(none)";
  RCLCPP_DEBUG_STREAM(oss.str());
}

double FocboxUnityDriver::CommandLimit::clip(double value)
{
  if (lower && value < lower)
  {
    RCLCPP_INFO_THROTTLE(10, "%s command value (%f) below minimum limit (%f), clipping.",
                      name.c_str(), value, *lower);
    return *lower;
  }
  if (upper && value > upper)
  {
    RCLCPP_INFO_THROTTLE(10, "%s command value (%f) above maximum limit (%f), clipping.",
                      name.c_str(), value, *upper);
    return *upper;
  }
  return value;
}

} // namespace focbox_unity_driver