#include "ros/ros.h"
#include <cstdlib>
#include <unistd.h>

int main(int argc, char **argv)
{
    ros::init(argc,argv,"start");
    ros::Duration(5);
    system("roslaunch mbot_gazebo view_mbot_with_laser_gazebo.launch");
 
    return 0;
}