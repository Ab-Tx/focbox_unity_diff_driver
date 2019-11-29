#include <focbox_unity_controller/focbox_unity_controller.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "focbox_unity_controller");
    ros::CallbackQueue ros_queue;

    ros::NodeHandle nh;
    nh.setCallbackQueue(&ros_queue);
    focbox_unity_controller::FocboxUnityController rhi(nh);

    ros::MultiThreadedSpinner spinner(0);
    spinner.spin(&ros_queue);
    return 0;
}