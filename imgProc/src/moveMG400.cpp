#include "ros/ros.h"
#include "mg400_bringup/MovL.h"
#include <iostream>

double input_x = 285.0;
double input_y = -22.0;
double input_z = 91.0;
double input_r = 274.0;

bool move(mg400_bringup::MovL::Request  &req,
         mg400_bringup::MovL::Response &res)
{
    req.x = static_cast<float>(input_x);
    req.y = static_cast<float>(input_y);
    req.z = static_cast<float>(input_z);
    req.r = static_cast<float>(input_r);

    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_mg400_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("move_mg400", move);
  ros::spin();

  return 0;
}