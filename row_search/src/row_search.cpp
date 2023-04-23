#include "ros/ros.h"
#include "row_search.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "Row_search");
    tf::TransformListener l;

    row_search d = row_search(&l);

    ros::Duration(1.0).sleep();

    ros::NodeHandle nh;
    ros::Timer timer = nh.createTimer(ros::Duration(0.01), &row_search::tick, &d);

    ros::spin();
}