#include "ros/ros.h"
#include "commander.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "Commander");
    tf::TransformListener l;

    commander d(&l);

    ros::Duration(1.0).sleep();

    ros::NodeHandle nh;
    ros::Timer timer = nh.createTimer(ros::Duration(1.0), &commander::state_tick, &d);

    ros::spin();
}