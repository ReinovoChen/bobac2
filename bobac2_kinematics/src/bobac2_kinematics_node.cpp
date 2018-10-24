#include "bobac2_kinematics.h"
#include "ros/ros.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bobac_kinematics");
    BobacKinematics bk;
    ros::spin();
    return 0;
}
