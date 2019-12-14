// Copyright (c) 2019 Rafael Silva (gimbas)
//
// Licensed under the MIT license: https://opensource.org/licenses/MIT
// Permission is granted to use, copy, modify, and redistribute the work.
// Full license information available in the project LICENSE file.
//

#include "focbox_unity_driver/focbox_unity_driver.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "focbox_unity_driver_node");
  ros::CallbackQueue ros_queue;

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  nh.setCallbackQueue(&ros_queue);

  focbox_unity_driver::FocboxUnityDriver focbox_unity_driver(nh, private_nh);


  ros::MultiThreadedSpinner spinner(0);
  spinner.spin(&ros_queue);

  return 0;
}