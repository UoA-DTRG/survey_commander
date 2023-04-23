#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>

laser_geometry::LaserProjection projector_;
ros::Publisher pointcloud_publisher_;
ros::Subscriber laser_subscriber_;

void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    printf("Got Pointcloud!\n");
    sensor_msgs::PointCloud2 cloud;
    projector_.projectLaser(*scan_in, cloud);
    cloud.header.stamp = scan_in->header.stamp;
    cloud.header.frame_id = scan_in->header.frame_id;

    pointcloud_publisher_.publish(cloud);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "Laser_to_PC2");
    ros::NodeHandle n;

    laser_subscriber_ = n.subscribe<sensor_msgs::LaserScan>("/laser/scan", 1, &scanCallback);
    pointcloud_publisher_ = n.advertise<sensor_msgs::PointCloud2>("/uav/d400/depth/points", 1);

    ros::spin();
}