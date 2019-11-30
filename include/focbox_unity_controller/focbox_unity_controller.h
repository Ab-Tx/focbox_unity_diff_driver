#ifndef FOCBOX_UNITY_CONTROLLER_H_
#define FOCBOX_UNITY_CONTROLLER_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <sstream>

namespace focbox_unity_controller
{
    class FocboxUnityController : public hardware_interface::RobotHW
    {
    public:
        FocboxUnityController(ros::NodeHandle &nh);
        ~FocboxUnityController();

        void init();
        void update(const ros::TimerEvent& e);
        void read();
        void write();

    private:
        ros::NodeHandle _nh;
        hardware_interface::JointStateInterface _joint_state_interface;
        hardware_interface::VelocityJointInterface _velocity_joint_interface;
        hardware_interface::EffortJointInterface _effort_joint_interface;
        ros::Timer non_realtime_loop_;
        ros::Duration control_period_;
        ros::Duration elapsed_time_;
        double _vel_cmd[2];
        double _cmd[2];
        double _pos[2];
        double _vel[2];
        double _eff[2];
        double loop_hz_;
        boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
    };
}

#endif // FOCBOX_UNITY_CONTROLLER_H_