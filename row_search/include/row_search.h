#pragma once

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "visualization_msgs/MarkerArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "uav_messages/GroundClothPoints.h"

#include "queue"

#include "Eigen/Eigen"
#include "pcl/filters/passthrough.h"

#include "dbscan.hpp"
#include "outlier_filter.hpp"

using namespace std;

typedef queue<vector<Eigen::Vector2d>> PCQueue;
typedef vector<Eigen::Vector2d> PCVec;

struct line_r_theta{
    double r;
    double theta;
};

struct row_search_parameters{
    // Input pointcloud
    string pointcloud_topic;
    string pointcloud2_topic;
    double pointcloud_band;
    
    // Frames
    string world_frame;
    string uav_frame;
    double z_offset;

    // DBSCAN paramters for clustering
    double dbscan_epsilon_centers;
    double dbscan_min_neighbors_centers;
    double dbscan_max_cluster_radius;
    double dbscan_min_cluster_radius;

    // DBSCAN parameters for line search
    double dbscan_epsilon_line;
    double dbscan_min_neighbors_line;
    double min_line_points;

    // Line search parameters
    double min_num_valid_cluster_ratio;
    double max_line_outlier_percent;
    double row_merge_threshold;
};

class row_search{
public:
    row_search(tf::TransformListener* l);

    // Main tick
    void tick(const ros::TimerEvent& event);

private:
    // Transform Listener
    tf::TransformListener* m_tf_listener;

    // Parameters
    struct row_search_parameters m_params;

    // Init code
    void init(ros::NodeHandle* n, ros::NodeHandle* pn);
    void init_subscribers(ros::NodeHandle* n, ros::NodeHandle* pn);
    void init_publishers(ros::NodeHandle* n, ros::NodeHandle* pn);

    // Subscriber callbacks
    void pointcloud_callback(const sensor_msgs::PointCloudConstPtr cloud);
    void pointcloud2_callback(const sensor_msgs::PointCloud2ConstPtr cloud);

    // Async stuff
    PCQueue m_pc_queue;

    // Ground cloth stuff
    double get_approximate_height(double x, double y);
    void ground_cloth_callback(const uav_messages::GroundClothPointsConstPtr cloth);
    bool m_cloth_valid;
    Eigen::Vector3d m_cloth_origin;
    int m_cloth_width;
    int m_cloth_length;
    double m_cloth_resolution;
    vector<double> m_cloth_heights;

    // Publish functions
    vector<line_r_theta> m_best_fit_lines;
    visualization_msgs::MarkerArray m_vis_array;
    std_msgs::Float64MultiArray m_row_info;
    void visualize_lines(vector<cluster> centers, vector<Eigen::Vector2d> pc, vector<line_r_theta> lines);
    void visualize_clusters(cluster centers);
    void publish_lines(vector<line_r_theta> lines);

    // Filters
    outlier_filter* m_minangle_filter;
    outlier_filter* m_rowcount_filter;

    // Subscribers
    ros::Subscriber m_pointcloud_subscriber;
    ros::Subscriber m_pointcloud2_subscriber;
    ros::Subscriber m_cloth_subscriber;

    // Publishers
    ros::Publisher m_visualization_pub;
    ros::Publisher m_cluster_pub;
    ros::Publisher m_rowinfo_publisher;
    ros::Publisher m_ground_corrected_pointcloud_pub;
};