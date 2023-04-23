#pragma once

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "trajectory_msgs/MultiDOFJointTrajectory.h"
#include "uav_messages/TrajectoryTarget.h"
#include "uav_messages/TrajectoryFollowerStatus.h"
#include "uav_messages/Height.h"
#include <Eigen/Eigen>

#include "row.h"
#include "helpers.hpp"

#include <vector>
#include <algorithm>
#include <cmath>
#include <string.h>

#define INIT_YAW_ANGLE_ARR_LENGTH 6

using namespace std;

enum STATE{
    STATE_START,
    STATE_SPIN,
    STATE_SETUP_ROW,
    STATE_TRAVERSING_ROW,
    STATE_ROW_SWITCH,
    STATE_RETURN,
    STATE_IDLE
};

struct params{
    // Frames
    string uav_frame;
    string world_frame;

    // Coverage parameters
    double exploration_distance;                    // How far to travel down each row
    double waypoint_distance;                       // How far apart waypoints are
    double target_speed;                            // Target speed to send to planner
    int num_survey_rows;                            // Number of rows to survey before returning

    // Traversal row filters
    double min_row_distance;                        // Minimum distance between traversal rows to consider the next row a row
    double row_deviation_threshold;                 // Maximum deviation from reference position (based on average distance) for the next row
    double target_height_above_ground;              // Target height to maintain above ground for waypoints 

    // Initialization parameters
    bool spin_at_start;                             // Toggles initialization yaw for easy map initialization
    double spin_pause_duration_seconds;             // Duration to pause at each step of the spin
    bool continuous_gen;                            // Test mode to continuously generate target waypoints
};

class commander{
public:
    commander(tf::TransformListener* l);

    void state_tick(const ros::TimerEvent& event);
private:
    // Parameters
    params m_params;

    // Tf stuff for position feedback on the mission
    tf::TransformListener* m_tf_listener;
    bool get_current_position(tf::StampedTransform &world_to_uav);

    // Subscribers
    ros::Subscriber m_row_info_subscriber;
    ros::Subscriber m_traj_splice_subscriber; // Need this to prevent rows from dying after a trajectory abort.

    // Subscriber callbacks
    void update_rows(const std_msgs::Float64MultiArray dat);
    void update_trajectory(const trajectory_msgs::MultiDOFJointTrajectory t);

    // Publishers
    ros::Publisher m_yaw_angle_target_publisher;
    ros::Publisher m_start_publisher;
    ros::Publisher m_finish_publisher;
    ros::Publisher m_return_publisher;
    ros::Publisher m_newrow_publisher;
    ros::Publisher m_rowdata_publisher;
    std_msgs::Bool m_true;

    // Service related stuff
    uav_messages::TrajectoryTarget m_trajectory_req;
    ros::ServiceClient m_trajectory_client;
    uav_messages::TrajectoryFollowerStatus m_trajectory_status;
    ros::ServiceClient m_trajectory_status_client;
    uav_messages::Height m_height_request;

    // Initialization
    void init();
    void init_subscribers(ros::NodeHandle &n);
    void init_publshers(ros::NodeHandle &n);
    void init_parameters(ros::NodeHandle &pn);

    // Row related storage and functions
    vector<row> m_row_info;
    vector<row> m_row_centers;
    vector<row> get_traverse_paths();
    row m_current_row;
    double m_average_row_distance;
    static double get_orthogonal_distance(row center, double x_location);
    bool m_first_row;

    // Initial spin parameters
    const double m_init_yaw_angles[INIT_YAW_ANGLE_ARR_LENGTH] = {-0.33 * M_PI, -0.67 * M_PI, -M_PI, 0.67 * M_PI, 0.33 * M_PI, 0};
    int m_init_yaw_arr_index;
    ros::Time m_spin_start_time;

    // State machine and related functions
    bool m_x_positive;
    STATE m_state;
    int m_num_traversed_rows;
    bool handle_state_start();
    bool handle_state_spin();
    bool handle_state_setup_row(bool change_row);
    bool handle_state_traverse_row();
    bool handle_state_return();
    Eigen::Vector3d m_return_target;
    bool handle_state_idle();
    ros::Time m_row_traverse_start_time;

    // Trajectory request populating functions
    void populate_trajectory_req(Eigen::Vector3d x0, Eigen::Vector3d x1);
};