#include "imgProc/procCommon.h"

int main(int argc, char** argv)
{
    // node setup
    ros::init(argc, argv, "check");
    
    // define imgFbController Class as ifc
    procCommon pc;

    // run
    ros::Rate loop_rate(1000);
    while (ros::ok())
    {
        if(!pc.check())
        {
            ROS_WARN("Program stopped");
            break;
        }
        ros::spinOnce();
		loop_rate.sleep();
    }

    return 0;
}