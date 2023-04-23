#include "commander.h"

#define DEBUG true

using namespace std;

commander::commander(tf::TransformListener* l){
    // Setup internal data structure
    m_tf_listener = l;
    m_state = STATE_START;
    m_row_info.clear();
    m_x_positive = true;
    m_num_traversed_rows = 0;
    m_init_yaw_arr_index = -1;
    m_spin_start_time = ros::Time(0);

    // Make nodehandles
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    // Init rest of node
    init_parameters(pn);
    init_subscribers(n);
    init_publshers(n);

    m_true.data = true;
    // Flag for first waypoint skip on row 1
    m_first_row = true;

    // Publish start message

    // Override the start state if the spin flag is set
    if(m_params.spin_at_start){
        m_state = STATE_SPIN;
    }

}

void commander::init_subscribers(ros::NodeHandle &n){
    m_row_info_subscriber = n.subscribe("/uav/dat/rows", 1, &commander::update_rows, this);
    m_traj_splice_subscriber = n.subscribe("/uav/target/trajectory", 1, &commander::update_trajectory, this);
}

void commander::init_publshers(ros::NodeHandle &n){
    m_yaw_angle_target_publisher = n.advertise<std_msgs::Float64>("/uav/target/yaw_angle", 1);
    m_start_publisher = n.advertise<std_msgs::Bool>("/uav/status/start", 1);
    m_finish_publisher = n.advertise<std_msgs::Bool>("/uav/status/finish", 1);
    m_newrow_publisher = n.advertise<std_msgs::Bool>("/uav/status/newrow", 1);
    m_return_publisher = n.advertise<std_msgs::Bool>("/uav/status/return", 1);
    m_rowdata_publisher = n.advertise<geometry_msgs::PoseArray>("/uav/status/rowinfo", 1);

    m_trajectory_client = n.serviceClient<uav_messages::TrajectoryTarget>("planner/new_goal");
    m_trajectory_status_client = n.serviceClient<uav_messages::TrajectoryFollowerStatus>("traj_follower_status");
}

void commander::init_parameters(ros::NodeHandle &pn){
    // Populate m_params from roslaunch
    pn.param("world_frame", m_params.world_frame, string("map"));
    pn.param("uav_frame", m_params.uav_frame, string("fcu_link"));

    pn.param("exploration_distance", m_params.exploration_distance, 16.0);
    pn.param("waypoint_distance", m_params.waypoint_distance, 4.0);
    pn.param("num_survey_rows", m_params.num_survey_rows, 5);

    pn.param("target_speed", m_params.target_speed, 1.0);

    pn.param("min_row_distance", m_params.min_row_distance, 2.0);
    pn.param("row_deviation_threshold", m_params.row_deviation_threshold, 1.5);
    pn.param("target_height_above_ground", m_params.target_height_above_ground, 3.5);

    pn.param("spin_at_start", m_params.spin_at_start, false);
    pn.param("spin_pause_time", m_params.spin_pause_duration_seconds, 2.0);
    pn.param("continuous_gen", m_params.continuous_gen, false);

    ROS_INFO("Using target speed: %f", m_params.target_speed);
}

void commander::update_rows(const std_msgs::Float64MultiArray dat){
    m_row_info.clear();
    for(size_t i = 0; i < dat.data.size(); i+=2){
        row r;
        r.r = dat.data[i];
        r.theta = dat.data[i + 1];

        if(DEBUG){
            ROS_INFO("Adding row R: %f, T: %f", r.r, r.theta);
        }
        m_row_info.push_back(r);
    }
}

void commander::state_tick(const ros::TimerEvent& event){
    ROS_INFO("Current State: %i", m_state);
    switch(m_state){
        case STATE_START:
            handle_state_start();
            break;
        case STATE_SPIN:
            handle_state_spin();
            break;
        case STATE_SETUP_ROW:
            handle_state_setup_row(false);
            break;
        case STATE_TRAVERSING_ROW:
            handle_state_traverse_row();
            break;
        case STATE_ROW_SWITCH:
            handle_state_setup_row(true);
            break;
        case STATE_RETURN:
            handle_state_return();
            break;
        case STATE_IDLE:
            handle_state_idle();
            break;
        default:
            break;
    }
}

bool commander::handle_state_return(){
    // Get current position for the z location
    tf::StampedTransform here;
    Eigen::Vector3d here_v;
    // Fail if we don't get the tf correctly
    if(!get_current_position(here)){
        ROS_ERROR("Failed to get world to UAV tf");
        return false;
    }
    helpers::tf_to_eigen_vector(here_v, here);

    // Send origin as traversal target
    m_trajectory_req.request.start_velocity.x = 0;
    m_trajectory_req.request.start_velocity.y = 0;
    m_trajectory_req.request.start_velocity.z = 0;
    m_trajectory_req.request.end_velocity.x = 0;
    m_trajectory_req.request.end_velocity.y = 0;
    m_trajectory_req.request.end_velocity.z = 0;
    m_trajectory_req.request.target_velocity.x = m_params.target_speed;    

    m_trajectory_req.request.waypoints.clear();

    // Make a service call request to find the local height.
    m_height_request.request.points.clear();
    geometry_msgs::Vector3 origin_vec;
    origin_vec.x = 0;
    origin_vec.y = 0;
    origin_vec.z = 0;
    m_height_request.request.points.push_back(origin_vec);

    double height_target = here_v(2);

    if(ros::service::call("/ground_seg/height_request", m_height_request)){
        height_target = m_height_request.response.heights[0];
        height_target += m_params.target_height_above_ground;
    }else{
        ROS_WARN("Failed to call height request service!");
    }

    Eigen::Vector3d target;
    target << 0, 0, height_target;

    m_return_target = target;

    geometry_msgs::Point p;
    helpers::eigen_vector_to_msg(target, p);

    m_trajectory_req.request.waypoints.push_back(p);

    if(m_trajectory_client.call(m_trajectory_req)){
        // State switch to row following
        m_state = STATE_IDLE;
        m_return_publisher.publish(m_true);
    }else{
        ROS_ERROR("Failed to call trajectory client!");
    }

    return true;
}

bool commander::handle_state_idle(){
    // Get current position
    tf::StampedTransform here;
    Eigen::Vector3d here_v;
    // Fail if we don't get the tf correctly
    if(!get_current_position(here)){
        ROS_ERROR("Failed to get world to UAV tf");
        return false;
    }
    helpers::tf_to_eigen_vector(here_v, here);

    if((m_return_target - here_v).norm() < 0.5){
        m_finish_publisher.publish(m_true);
    }

    return true;
}

bool commander::handle_state_start(){
    // Wait until row information is populated
    if(m_row_info.size() < 2) return false;

    // Send start message
    m_start_publisher.publish(m_true);

    m_state = STATE_SETUP_ROW;    

    return true;
}

bool commander::handle_state_spin(){
    ROS_INFO("State Spin: ind: %i, time: %f", m_init_yaw_arr_index, (ros::Time::now() - m_spin_start_time).toSec());
    if(ros::Time::now() - m_spin_start_time > ros::Duration(m_params.spin_pause_duration_seconds)){
        // Update start time
        m_spin_start_time = ros::Time::now();
        
        // Send new yaw setpoint
        m_init_yaw_arr_index++;
        if(m_init_yaw_arr_index >= INIT_YAW_ANGLE_ARR_LENGTH){
            m_state = STATE_START;
            return true;
        }

        static std_msgs::Float64 yawdat;
        yawdat.data = m_init_yaw_angles[m_init_yaw_arr_index];

        m_yaw_angle_target_publisher.publish(yawdat);
    }

    return true;
}

bool commander::handle_state_setup_row(bool change_row){
    // Switch to return mode if we've surveyed the required number of rows
    if(m_num_traversed_rows >= m_params.num_survey_rows){
        m_state = STATE_RETURN;
        return true;
    }

    ROS_INFO("Setting up row, change flag: %i", change_row);
    // Set up required temp storage
    vector<row> row_estimates;
    tf::StampedTransform here;
    Eigen::Vector3d here_v;
    // Fail if we don't get the tf correctly
    if(!get_current_position(here)){
        ROS_ERROR("Failed to get world to UAV tf");
        return false;
    }
    helpers::tf_to_eigen_vector(here_v, here);
    
    ROS_INFO("Got tf data");
    // Fail if less than two rows are detected
    if(m_row_info.size() < 2){
        return false;
    }

    // Row estimates are now good. Compile into a set of lines
    row_estimates = get_traverse_paths();

    // Fail if no traversable paths are detected
    if(row_estimates.size() < 1){
        return false;
    }

    // Find the closest one
    double min_dist = 9999999999;
    row min_row;
    ROS_INFO("Got Estimates");

    // If we're requesting a row change mode, find the smallest r1.r > r.r
    if(change_row){
        min_dist = 9999999999;
        row min_row_next;

        row estimated_row = m_current_row;
        estimated_row.r += m_average_row_distance;

        if(DEBUG){
            ROS_INFO("Using estimated row R: %f, T: %f", estimated_row.r, estimated_row.theta);
        }

        for(auto r : row_estimates){
            if((r.r > (m_current_row.r + m_params.min_row_distance)) && r.r < min_dist){
                // Do row validity tests
                if(fabs(estimated_row.r - r.r) < m_params.row_deviation_threshold){
                    min_dist = r.r;
                    min_row_next = r;
                }
            }
        }

        // Seed the next row
        min_row = min_row_next;

        // Check if we've succeeded in identifying a row
        if(min_dist > 99999999){
            ROS_WARN("Failed to locate valid row, using estimate at r: %f t:%f", estimated_row.r, estimated_row.theta);
            min_row = estimated_row;
        }
    }else{
        for(auto r : row_estimates){
            auto dist = fabs(here_v(1) - get_orthogonal_distance(r, here_v(0)));
            if(dist < min_dist){
                min_dist = dist;
                min_row = r;
            }
        }
    }

    m_current_row = min_row;

    // Setup the line to find the required waypoints
    Eigen::Vector3d x0, x1;
    x0 << 0, get_orthogonal_distance(min_row, 0), here_v(2);
    x1 << m_params.exploration_distance, get_orthogonal_distance(min_row, m_params.exploration_distance), here_v(2);
    
    static geometry_msgs::PoseArray rowinfo_msg;
    geometry_msgs::Pose p;
    helpers::eigen_vector_to_msg(x0, p.position);
    rowinfo_msg.poses.push_back(p);
    helpers::eigen_vector_to_msg(x1, p.position);
    rowinfo_msg.poses.push_back(p);

    m_rowdata_publisher.publish(rowinfo_msg);

    // Swap x0 and x1 if we're doing a return stroke
    if(!m_x_positive){
        Eigen::Vector3d temp = x0;
        x0 = x1;
        x1 = temp;
    }

    // Invert x positive flag
    m_x_positive = !m_x_positive;

    populate_trajectory_req(x0, x1);

    // Do ground estimation to correct for altitude
    // Make request to perform height correction
    uav_messages::Height height_request;
    height_request.request.points.clear();

    // Get current position
    geometry_msgs::Vector3 here_vec3;
    here_vec3.x = here.getOrigin().x();
    here_vec3.y = here.getOrigin().y();
    here_vec3.z = here.getOrigin().z();

    height_request.request.points.push_back(here_vec3);

    ROS_INFO("Generated Waypoints!");

    for(auto w : m_trajectory_req.request.waypoints){
        geometry_msgs::Vector3 v;
        v.x = w.x;
        v.y = w.y;
        // We don't need Z

        height_request.request.points.push_back(v);
    }

    if(ros::service::call("/ground_seg/height_request", height_request)){
        // Update waypoint z with relative heights
        // Note response 0 is our current height;
        //double current_relative_height = height_request.response.heights[0];
        // Replace the estimated height with current estimated altitude if we fail to interpolate the current position
        //if(abs(current_relative_height) < 0.001) current_relative_height = height_request.response.mean;
        for(int i = 1; i < height_request.response.heights.size(); ++i){
            if(height_request.response.valid[i] == true){
                // Add the offset to the z height, which is initialized to our current altitude
                m_trajectory_req.request.waypoints[i-1].z = height_request.response.heights[i] + m_params.target_height_above_ground;
            }
        }
    }else{
        ROS_WARN("Failed to call height request service!");
    }

    for(auto p : m_trajectory_req.request.waypoints){
        ROS_INFO("Waypoint: %f, %f, %f", p.x, p.y, p.z);
    }

    if(m_trajectory_client.call(m_trajectory_req)){
        // State switch to row following
        m_state = STATE_TRAVERSING_ROW;
    }else{
        ROS_ERROR("Failed to call trajectory client!");
    }

    //if(!m_params.continuous_gen) 
    m_num_traversed_rows++;
    m_newrow_publisher.publish(m_true);
    m_row_traverse_start_time = ros::Time::now();
    return true;
}

bool commander::handle_state_traverse_row(){
    // Wait at least a second before checking this
    if((ros::Time::now() - m_row_traverse_start_time) < ros::Duration(5.0)){
        return true;
    }

    // Make a service call to the trajectory follower
    m_trajectory_status.request.request = true;

    if(!m_trajectory_status_client.call(m_trajectory_status)){
        ROS_ERROR("Failed to call trajectory status service, is the trajectory follower node running?");
        return false;
    }

    // Wait for state to become idle
    if(m_trajectory_status.response.status == uav_messages::TrajectoryFollowerStatus::Request::IDLE || m_params.continuous_gen){
        m_state = STATE_ROW_SWITCH;
    }

    return true;
}

vector<row> commander::get_traverse_paths(){
    vector<row> ret;

    // Sort the list of available rows by radius
    // The search node should already cull this for us

    sort(m_row_info.begin(), m_row_info.end());

    double count = 0;
    m_average_row_distance = 0;

    for(size_t i = 0; i < m_row_info.size() - 1; ++i){
        // Take the average location of two neighboring rows
        const row r1 = m_row_info[i], r2 = m_row_info[i + 1];
        row out;
        out.r = (r1.r + r2.r) / 2.0;
        out.theta = (r1.theta + r2.theta) / 2.0;

        m_average_row_distance += fabs(r1.r - r2.r);
        count++;

        if(DEBUG){
            ROS_INFO("Path R1: %f, %f, R2: %f, %f, OUT: %f, %f", r1.r, r1.theta, r2.r, r2.theta, out.r, out.theta);
        }

        ret.push_back(out);
    }
    
    if(DEBUG){
        for(auto r : ret){
            ROS_INFO("Traverse path at R: %f, T: %f", r.r, r.theta);
        }
    }

    m_average_row_distance /= count;

    return ret;
}

double commander::get_orthogonal_distance(row center, double x_location){
    double y, m, c;
    // Compute gradient and y intercept
    m = tan(center.theta - M_PI_2);
    c = center.r / sin(center.theta);

    // Find y value at current x location
    y = m * x_location + c;

    return y;
}

bool commander::get_current_position(tf::StampedTransform &world_to_uav){
    auto now = ros::Time::now();
    try{
        m_tf_listener->waitForTransform(m_params.world_frame, m_params.uav_frame, now, ros::Duration(0.5));
        m_tf_listener->lookupTransform(m_params.world_frame, m_params.uav_frame, now, world_to_uav);

        return true;
    }catch(tf::TransformException ex){
        ROS_INFO("Failed to lookup tf: %s", ex.what());
        return false;
    }
}

void commander::populate_trajectory_req(Eigen::Vector3d x0, Eigen::Vector3d x1){
    // Setup the request
    m_trajectory_req.request.start_velocity.x = 0;
    m_trajectory_req.request.start_velocity.y = 0;
    m_trajectory_req.request.start_velocity.z = 0;
    m_trajectory_req.request.end_velocity.x = 0;
    m_trajectory_req.request.end_velocity.y = 0;
    m_trajectory_req.request.end_velocity.z = 0;
    m_trajectory_req.request.target_velocity.x = m_params.target_speed;    

    m_trajectory_req.request.waypoints.clear();

    Eigen::Vector3d v = x1 - x0;
    double dist = (x1 - x0).norm();
    v.normalize();

    double i = 0;
    // Skip first waypoint on row 1
    if(m_first_row) {
        i += m_params.waypoint_distance;
        m_first_row = false;
    }
    
    for(; i < dist; i += m_params.waypoint_distance){
        Eigen::Vector3d pos = x0 + v * i;
        geometry_msgs::Point p;
        
        helpers::eigen_vector_to_msg(pos, p);

        m_trajectory_req.request.waypoints.push_back(p);
    }

    if((dist - i) > 1.0){
        Eigen::Vector3d pos = x1;
        geometry_msgs::Point p;
        
        helpers::eigen_vector_to_msg(pos, p);

        m_trajectory_req.request.waypoints.push_back(p);
    }
}

void commander::update_trajectory(const trajectory_msgs::MultiDOFJointTrajectory t){
    // Reset row traversal start time
    m_row_traverse_start_time = ros::Time::now();
}