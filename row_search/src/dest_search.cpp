#include "ros/ros.h"
#include "dest_search.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "Destination_Search");
    tf::TransformListener l;

    dest_search d = dest_search(&l);

    ros::Duration(1.0).sleep();

    ros::spin();
}