#ifndef __dest_search_h
#define __dest_search_h
#pragma once

#include "ros/ros.h"
#include <string.h>
#include <vector>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "tf/transform_listener.h"
#include "visualization_msgs/MarkerArray.h"
#include "uav_messages/LeftRightRow.h"
#include "std_msgs/Float64MultiArray.h"

#include "pcl/filters/passthrough.h"
#include "outlier_filter.hpp"

// Constants
#define PI 3.14159

// Defines
#define RANSAC_MAX_GEN_RUNS 15

// Macros
#define DISTSQR(a,b) (((a.x - b.x) * (a.x - b.x)) + ((a.y - b.y) * (a.y - b.y)))
#define DISTSQR_TF(a,b) (((a.x - b.x()) * (a.x - b.x())) + ((a.y - b.y()) * (a.y - b.y())))
#define MORETHAN(x, y, a) ((abs(x - y)) > a)
#define SQR(a) ((a)*(a))

using namespace std;

struct dest_search_parameters{
    string laser_scan_topic;
    
    // Frames
    string world_frame;
    string uav_frame;
    
    // Point cloud filter
    string point_cloud_topic;
    double point_cloud_band;
    
    // DBSCAN paramters for clustering
    double dbscan_epsilon;
    double dbscan_epsilon_squared;
    double dbscan_min_neighbors;

    // RANSAC parameters for cluster rejection
    double ransac_max_iterations;
    double ransac_inlier_max_deviation;
    double ransac_min_compliance_ratio;
    double ransac_max_radius;
    double ransac_min_radius;

    // FOV restriction
    double y_fov_max_distance;
    double hough_angle_horizontal_tolerance;

    // Most likely row extraction
    double overlap_row_distance_threshold;
    double same_row_threshold;
    double row_outlier_gate_size;
    int row_outlier_filter_size;
    double stale_cull_time;
};

struct point2{
    double x;
    double y;
};

struct point2r{
    double x;
    double y;
    double r;
};

struct line_r_theta{
    double r;
    double theta;
    bool operator < (line_r_theta rhs){
        return(r < rhs.r);
    };
    line_r_theta operator += (line_r_theta rhs){
        line_r_theta ret;
        ret.r = r + rhs.r;
        ret.theta = theta + rhs.theta;

        return ret;
    };
    line_r_theta operator /= (double rhs){
        line_r_theta ret;
        ret.r = r / rhs;
        ret.theta = r/rhs;
        return ret;
    }
};

typedef vector<point2> cluster;

class dest_search{
public:
    dest_search(tf::TransformListener* l);

private:
    // Transform Listener
    tf::TransformListener* m_tf_listener;

    // Parameters for node
    struct dest_search_parameters m_params;

    // Subscribers
    ros::Subscriber m_laser_scan_subscriber;
    ros::Subscriber m_pointcloud_subscriber;

    // Publishers
    ros::Publisher m_visualization_pub;
    ros::Publisher m_out_publisher;
    ros::Publisher m_rowinfo_publisher;

    // Mutex
    bool m_process_mutex;

    // Class scope stuff for easy visualiszation
    visualization_msgs::MarkerArray m_vis_array;
    uav_messages::LeftRightRow m_out_msg;

    // Internal cache stuff for speed
    bool m_trig_cache_valid;
    double* m_cos_cache;
    double* m_sin_cache;

    // Scan stuff
    std::string m_frame_id;
    double m_laser_max_range;
    vector<point2> m_scan_cartesian;

    // Initialization
    void init_parameters();
    void inline init_subscribers(ros::NodeHandle n);
    void inline init_publishers(ros::NodeHandle n);

    // Subscriber
    void update_laser_scan(const sensor_msgs::LaserScan scan);
    void update_pointcloud(const sensor_msgs::PointCloudConstPtr cloud);

    // Processing
    void process_laser_scan();

    // DBSCAN stuff
    vector<cluster> dbscan();
    vector<int> dbscan_neighbors(point2 p);
    
    // RANSAC stuff
    bool ransac_circle_search(cluster c, point2r &center);

    // Compute centroids as an alternative to RANSAC. Seems to work better
    vector<point2r> compute_centroids(vector<cluster> clusters);
    void restrict_y_fov(vector<point2r> &centers);
    
    // Hough transform stuff
    vector<line_r_theta> hough_line_search(vector<point2r> points, unsigned int num_candidates);
    double bin_hough_lines(vector<line_r_theta> lines, int avg_bins);

    // Visualization
    void visualise_centers(vector<point2r> centers, vector<line_r_theta> lines, vector<line_r_theta> lines_complete);

    // Data out
    vector<line_r_theta> publish_likely_rows(vector<line_r_theta> lines);

    // List of likely rows
    vector<line_r_theta> m_likely_rows;
    vector<line_r_theta> collate_rows(vector<line_r_theta> row_list);
    void update_likely_rows(vector<line_r_theta> row_list);
    outlier_filter* m_outlier_filter;
    std_msgs::Float64MultiArray m_row_info;
    
};



#endif