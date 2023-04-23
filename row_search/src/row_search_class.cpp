#include "ros/ros.h"
#include "row_search.h"
#include "helpers.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/conversions.h"

#include "stdlib.h"

using namespace std;

row_search::row_search(tf::TransformListener* l){
    // Setup listeners
    m_tf_listener = l;
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    // Invalidate cloth
    m_cloth_valid = false;

    // Init nodes and subscribers
    init(&n, &pn);
    init_subscribers(&n, &pn);
    init_publishers(&n, &pn);
}

void row_search::init(ros::NodeHandle* n, ros::NodeHandle* pn){
    // Pointcloud stuff
    pn->param("pointcloud_topic", m_params.pointcloud_topic, string("/path_search/ESDFMap/occ_pc"));
    pn->param("pointcloud2_topic", m_params.pointcloud2_topic, string("/path_search/ESDFMap/occ_pc2"));
    pn->param("pointcloud_band", m_params.pointcloud_band, 0.5);
    pn->param("world_frame", m_params.world_frame, string("map"));
    pn->param("uav_frame", m_params.uav_frame, string("fcu_link"));
    pn->param("z_offset", m_params.z_offset, 0.0);

    pn->param("dbscan_epsilon_centers", m_params.dbscan_epsilon_centers, 1.0);
    pn->param("dbscan_min_neighbors_centers", m_params.dbscan_min_neighbors_centers, 5.0);
    pn->param("dbscan_max_cluster_radius", m_params.dbscan_max_cluster_radius, 6.0);
    pn->param("dbscan_min_cluster_radius", m_params.dbscan_min_cluster_radius, 0.0);

    pn->param("dbscan_epsilon_line", m_params.dbscan_epsilon_line, 1.0);
    pn->param("dbscan_min_neighbors_line", m_params.dbscan_min_neighbors_line, 1.0);

    pn->param("min_line_points", m_params.min_line_points, 4.0);

    pn->param("max_line_outlier_percent", m_params.max_line_outlier_percent, 0.3);
    pn->param("row_merge_threshold", m_params.row_merge_threshold, 2.0);
    pn->param("min_num_valid_cluster_ratio", m_params.min_num_valid_cluster_ratio, 0.5);

    m_minangle_filter = new outlier_filter(20, 1.0);
    m_rowcount_filter = new outlier_filter(20, 2.0);

}
void row_search::init_subscribers(ros::NodeHandle* n, ros::NodeHandle* pn){
    //m_pointcloud_subscriber = n->subscribe(m_params.pointcloud_topic, 1, &row_search::pointcloud_callback, this);
    m_pointcloud_subscriber = n->subscribe(m_params.pointcloud2_topic, 1, &row_search::pointcloud2_callback, this);
    m_cloth_subscriber = n->subscribe("/ground_seg/points", 1, &row_search::ground_cloth_callback, this);
}

void row_search::init_publishers(ros::NodeHandle* n, ros::NodeHandle* pn){
    m_visualization_pub = n->advertise<visualization_msgs::MarkerArray>("/uav/vis/centers", 10);
    m_cluster_pub = n->advertise<visualization_msgs::MarkerArray>("/uav/vis/clusters", 1);
    m_rowinfo_publisher = n->advertise<std_msgs::Float64MultiArray>("/uav/dat/rows", 5);
    m_ground_corrected_pointcloud_pub = n->advertise<sensor_msgs::PointCloud2>("/uav/vis/ground_corrected_pcloud", 1);
}

void row_search::pointcloud_callback(const sensor_msgs::PointCloudConstPtr cloud){
    // Get current z position
    tf::StampedTransform odom_to_body;
    helpers::get_current_position(m_tf_listener, m_params.world_frame, m_params.uav_frame, odom_to_body, ros::Time::now());
    double x = odom_to_body.getOrigin().x();
    double y = odom_to_body.getOrigin().y();
    double z = odom_to_body.getOrigin().z();

    if(m_cloth_valid){
        z -= get_approximate_height(x, y);
    }else{
        ROS_WARN("Cloth points are invalid, is the ground segmentation node running?");
    }

    ROS_INFO("Z band: %f %f", z - m_params.pointcloud_band + m_params.z_offset, z + m_params.pointcloud_band + m_params.z_offset);

    // Passthrough filter
    sensor_msgs::PointCloud2 pc2;
    sensor_msgs::convertPointCloudToPointCloud2(*cloud, pc2);
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
    pcl::fromROSMsg(pc2, pcl_cloud);

    // if valid, untransform the point cloud by the ground height
    if(m_cloth_valid){
        pcl::PointCloud<pcl::PointXYZ>::iterator it;
        for(it = pcl_cloud.points.begin(); it < pcl_cloud.points.end(); it++){
            it->z -= get_approximate_height(it->x, it->y);
        }
    }

    // Lazy visualization
    pcl::toROSMsg(pcl_cloud, pc2);
    m_ground_corrected_pointcloud_pub.publish(pc2);
    
    pcl::PassThrough<pcl::PointXYZ> pass;
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_ptr;
    source_ptr = pcl_cloud.makeShared();
    pass.setInputCloud(source_ptr);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z - m_params.pointcloud_band + m_params.z_offset, z + m_params.pointcloud_band + m_params.z_offset);
    pass.filter(cloud_filtered);

    // Add to queue
    PCVec pc_vec;
    pcl::PointCloud<pcl::PointXYZ>::iterator it;

    for(it = cloud_filtered.points.begin(); it < cloud_filtered.points.end(); it++){
        Eigen::Vector2d p;
        p << it->x, it->y;

        pc_vec.push_back(p);
    }

    m_pc_queue.emplace(pc_vec);
}

void row_search::pointcloud2_callback(const sensor_msgs::PointCloud2ConstPtr cloud){
    // Get current z position
    tf::StampedTransform odom_to_body;
    helpers::get_current_position(m_tf_listener, m_params.world_frame, m_params.uav_frame, odom_to_body, ros::Time::now());
    double x = odom_to_body.getOrigin().x();
    double y = odom_to_body.getOrigin().y();
    double z = odom_to_body.getOrigin().z();

    if(m_cloth_valid){
        z -= get_approximate_height(x, y);
    }else{
        ROS_WARN("Cloth points are invalid, is the ground segmentation node running?");
    }

    ROS_INFO("Z band: %f %f", z - m_params.pointcloud_band + m_params.z_offset, z + m_params.pointcloud_band + m_params.z_offset);

    // Passthrough filter
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
    pcl::fromROSMsg(*cloud, pcl_cloud);

    // if valid, untransform the point cloud by the ground height
    if(m_cloth_valid){
        pcl::PointCloud<pcl::PointXYZ>::iterator it;
        for(it = pcl_cloud.points.begin(); it < pcl_cloud.points.end(); it++){
            it->z -= get_approximate_height(it->x, it->y);
        }
    }

    // Lazy visualization
    sensor_msgs::PointCloud2 pc2;
    pcl::toROSMsg(pcl_cloud, pc2);
    m_ground_corrected_pointcloud_pub.publish(pc2);
    
    pcl::PassThrough<pcl::PointXYZ> pass;
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_ptr;
    source_ptr = pcl_cloud.makeShared();
    pass.setInputCloud(source_ptr);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z - m_params.pointcloud_band + m_params.z_offset, z + m_params.pointcloud_band + m_params.z_offset);
    pass.filter(cloud_filtered);

    // Add to queue
    PCVec pc_vec;
    pcl::PointCloud<pcl::PointXYZ>::iterator it;

    for(it = cloud_filtered.points.begin(); it < cloud_filtered.points.end(); it++){
        Eigen::Vector2d p;
        p << it->x, it->y;

        pc_vec.push_back(p);
    }

    m_pc_queue.emplace(pc_vec);
}

void row_search::ground_cloth_callback(const uav_messages::GroundClothPointsConstPtr cloth){
    // Populate internal cloth struct
    m_cloth_resolution = cloth->resolution;
    m_cloth_width = cloth->width;
    m_cloth_length = cloth->length;
    m_cloth_origin << cloth->origin.x, cloth->origin.y, cloth->origin.z;
    m_cloth_heights = cloth->heights;

    // Make cloth valid
    m_cloth_valid = true;
}

void row_search::tick(const ros::TimerEvent& event){
    // Just spin if queue is empty
    if(m_pc_queue.empty()) return;

    // Remove all but one element from the queue
    while(m_pc_queue.size() > 1){
        m_pc_queue.pop();
    }

    // Get pointcloud and do first DBSCAN cluter for centers
    auto pc = m_pc_queue.front();
    m_pc_queue.pop();
    double outliers = 0;
    auto clusters = dbscan(pc, m_params.dbscan_epsilon_centers, m_params.dbscan_min_neighbors_centers, outliers);
    ROS_INFO("Found %lu clusters", clusters.size());

    // Get list of centers
    cluster centers;
    for(auto c : clusters){
        double minx = 999999, maxx = -999999, miny = 999999, maxy = -999999;
        for(auto p : c){
            if(p(0) > maxx) maxx = p(0);
            if(p(0) < minx) minx = p(0);
            if(p(1) > maxy) maxy = p(1);
            if(p(1) < miny) miny = p(1);
        }

        const double xdiff = maxx - minx;
        const double ydiff = maxy - miny;
        const double sizesq = xdiff * xdiff + ydiff * ydiff;
        if ((sizesq > m_params.dbscan_max_cluster_radius * m_params.dbscan_max_cluster_radius) ||
            (sizesq < m_params.dbscan_min_cluster_radius * m_params.dbscan_min_cluster_radius))
            continue;

        // TODO: Perhaps perform more filtering
        auto center = cluster_mean(c);
        centers.push_back(center);
    }

    visualize_clusters(centers);

    double minresidual = 99999999;
    double minangle = -999;
    
    // We directly use radians here because the trig functions do
    //for(double a = 0; a < M_PI; a += (M_PI / 180)){
    for(double a = -(M_PI/6); a < (M_PI/6); a += (M_PI / 180)){
        // Copy centers so we can mess with it
        cluster centers_rot = centers;
        Eigen::Matrix2d rotm;
        rotm << cos(a), -sin(a),
                sin(a), cos(a);

        // Rotate by rotm to test for best direction
        for(auto& c : centers_rot){
            c = rotm * c;
            // Remove effects of x axis
            c(0) = 0;
        }


        // Do a DBSCAN cluster with no influence of x axis
        double outliers = 0;
        auto rowcluster = dbscan(centers_rot, m_params.dbscan_epsilon_line, m_params.dbscan_min_neighbors_line, outliers);
        
        // Get smallest residual
        double residuals = 0;
        int numvalid = 0;
        for(auto c : rowcluster){
            if(c.size() < m_params.min_line_points) continue;
            numvalid ++;
            residuals += (get_cluster_range(c).norm() / c.size()) * (get_cluster_range(c).norm() / c.size());
        }
        residuals *= ((double)rowcluster.size() / (double)numvalid);
        // if((double) numvalid / (double) rowcluster.size() < m_params.min_num_valid_cluster_ratio){
        //     residuals = 0;
        // }

        // Only add if we have a small enough percentage of outliers
        ROS_INFO("Angle %f, residual: %f, min: %f, outliers: %f, size: %lu", a, residuals, minresidual, outliers, centers_rot.size());
        if(residuals != 0 && (minresidual > residuals) && ((outliers / (double) centers_rot.size()) < m_params.max_line_outlier_percent)){
            minresidual = residuals;
            minangle = a;
        }
    }

    if(minangle < -10){
        // Something has gone REALLY wrong. Just abort
        return;
    }

    // Do a statistical outlier filter to remove spontaneous bad estimates
    if(!m_minangle_filter->add_sample(minangle)){
        ROS_INFO("Outlier detected, skipping");
        return;
    }

    ROS_INFO("Got best direction, rotangle = %f", minangle);

    // Transform main reference by rotm to get the best direction horizontal in world frame
    cluster centers_rot = centers;
    Eigen::Matrix2d rotm;
    rotm << cos(minangle), -sin(minangle),
            sin(minangle), cos(minangle);
    // Rotate by rotm to for best direction
    for(auto& c : centers_rot){
        c = rotm * c;
        // Remove effects of x axis
        c(0) = 0;
    }
    double residuals = 0;
    auto labels = dbscan_labels(centers_rot, m_params.dbscan_epsilon_line, m_params.dbscan_min_neighbors_line, residuals);
    
    
    vector<cluster> lines;
    vector<cluster> lines2;
    vector<double> line_y;
    // Find the number of clusters and prepopulate the line array. This allows us to traverse labels in O(n) time.
    double num_clusters = 0;
    for(int i = 0; i < labels.size(); ++i) num_clusters = (labels[i] > num_clusters) ? labels[i] : num_clusters;
    ROS_INFO("Found %f rows", num_clusters);
    for(int i = 0; i < num_clusters; ++i){
        cluster c;
        cluster c2;
        double meany = 0;
        for(int j = 0; j < labels.size(); ++j){
            if(labels[j] == i){
                c.push_back(centers[j]);
                c2.push_back(centers_rot[j]);
                meany += centers_rot[j](1);
            }
        }
        if(c.size() >= m_params.min_line_points){
            meany /= (double) c.size();
            bool merge = false;
            for(int i = 0; i < line_y.size(); ++i){
                if(fabs(meany - line_y[i]) < m_params.row_merge_threshold){
                    // Don't make a new row, merge into existing instead
                    merge = true;
                    for(auto cc : c){
                        lines[i].push_back(cc);
                    }
                    break;
                }
            }

            if(!merge){
                ROS_INFO("Pushing line of size %lu", c.size());
                line_y.push_back(meany);
                lines.push_back(c);
            }
        }
        lines2.push_back(c2);
    }

    // Populate lines for publishing
    m_best_fit_lines.clear();

    for(auto l : lines){
        // Setup least squares problem
        const int size = l.size();
        Eigen::MatrixXd A(size, 2);
        Eigen::MatrixXd B(size, 1);
        for(int i = 0; i < l.size(); ++i){
            A(i, 0) = l[i](0);
            A(i, 1) = 1;
            B(i, 0) = l[i](1);
        }

        // Solve for least squares fit
        Eigen::MatrixXd lsf = A.colPivHouseholderQr().solve(B);

        double m = lsf(0, 0);
        double c = lsf(1, 0);

        line_r_theta o;
        o.theta = atan2(1, -m);
        o.r = c * sin(o.theta);

        m_best_fit_lines.push_back(o);

        ROS_INFO("Fit line T: %f, R: %f", o.theta, o.r);
    }

    // Reject sample if the number changes suddenly
    if(!m_rowcount_filter->add_sample((double) m_best_fit_lines.size())){
        ROS_WARN("Bad row count detected! skipping");
        return;
    }

    // Visualize and publish
    visualize_lines(lines, pc, m_best_fit_lines);
    publish_lines(m_best_fit_lines);
}

void row_search::visualize_lines(vector<cluster> centers, vector<Eigen::Vector2d> pc, vector<line_r_theta> lines){
    visualization_msgs::Marker m;
    
    m.header.frame_id = m_params.world_frame;
    m.header.stamp = ros::Time();
    m.type = visualization_msgs::Marker::CYLINDER;
    m.action = visualization_msgs::Marker::ADD;
    m.lifetime = ros::Duration(1.0);
    m.pose.position.z = 0;
    m.pose.orientation.x = 0;
    m.pose.orientation.y = 0;
    m.pose.orientation.z = 0;
    m.pose.orientation.w = 1.0;
    m.scale.z = 1;
    m.color.a = 1.0;
    m.color.r = 1.0;
    m.color.g = 0;
    m.color.b = 0;

    m_vis_array.markers.clear();

    int id = 0;
    for(auto c : centers){
        m.color.r = (double) (rand() % 20) / 20;
        m.color.g = (double) (rand() % 20) / 20;
        m.color.b = (double) (rand() % 20) / 20;
        for(auto p : c){
            m.pose.position.x = p(0);
            m.pose.position.y = p(1);
            m.id = id++;
            m.scale.x = 1.0;
            m.scale.y = 1.0;
            m_vis_array.markers.push_back(m);
        }
    }

    m.color.a = 1.0;
    m.color.r = 1.0;
    m.color.g = 1.0;
    m.color.b = 0;

    for(auto p : pc){
        m.pose.position.x = p(0);
        m.pose.position.y = p(1);
        m.id = id++;
        m.scale.x = 0.2;
        m.scale.y = 0.2;
        m_vis_array.markers.push_back(m);
    }

    // Make line_list to visualise found lines
    m.type = visualization_msgs::Marker::LINE_LIST;
    m.color.b = 1.0;
    m.color.r = 0.0;
    m.scale.x = 0.1;
    m.scale.y = 0;
    m.scale.z = 0;
    m.pose.position.x = 0;
    m.pose.position.y = 0;
    m.id = id++;

    constexpr double projection_length = 30;
    m.points.clear();

    for(auto l : lines){
        double x_pos_1 = cos(l.theta) * l.r - sin(l.theta) * projection_length;
        double y_pos_1 = sin(l.theta) * l.r + cos(l.theta) * projection_length;

        // Negative projection length
        double x_pos_2 = cos(l.theta) * l.r + sin(l.theta) * projection_length;
        double y_pos_2 = sin(l.theta) * l.r - cos(l.theta) * projection_length;

        geometry_msgs::Point p1, p2;
        p1.x = x_pos_1;
        p1.y = y_pos_1;
        p1.z = 0;

        p2.x = x_pos_2;
        p2.y = y_pos_2;
        p2.z = 0;

        m.points.push_back(p1);
        m.points.push_back(p2);
    }

    m_vis_array.markers.push_back(m);
    m_visualization_pub.publish(m_vis_array);
}

void row_search::publish_lines(vector<line_r_theta> lines){
    m_row_info.data.clear();
    for(auto r : lines){
        m_row_info.data.push_back(r.r);
        m_row_info.data.push_back(r.theta);
    }

    m_rowinfo_publisher.publish(m_row_info);
}

void row_search::visualize_clusters(cluster centers){
    visualization_msgs::Marker m;
    
    m.header.frame_id = m_params.world_frame;
    m.header.stamp = ros::Time();
    m.type = visualization_msgs::Marker::CYLINDER;
    m.action = visualization_msgs::Marker::ADD;
    m.lifetime = ros::Duration(2.0);
    m.pose.position.z = 0;
    m.pose.orientation.x = 0;
    m.pose.orientation.y = 0;
    m.pose.orientation.z = 0;
    m.pose.orientation.w = 1.0;
    m.scale.z = 1.5;
    m.color.a = 1.0;
    m.color.r = 1.0;
    m.color.g = 0;
    m.color.b = 0;

    m_vis_array.markers.clear();

    int id = 0;
    printf("Detected clusters\n");
    for(auto c : centers){
        m.color.r = (double) (rand() % 20) / 20;
        m.color.g = (double) (rand() % 20) / 20;
        m.color.b = (double) (rand() % 20) / 20;
        m.pose.position.x = c(0);
        m.pose.position.y = c(1);
        m.id = id++;
        m.scale.x = 0.5;
        m.scale.y = 0.5;
        m_vis_array.markers.push_back(m);
        // Dump data for generation
        printf("%f, %f\n", c(0), c(1));
    }
    printf("\n\n");

    m_vis_array.markers.push_back(m);
    m_cluster_pub.publish(m_vis_array);
}

inline double row_search::get_approximate_height(double x, double y){
    // Note, this function will return 0 if the cloth is not initialized!
    // Make sure you check before using it
    // Fail if no cloth data has been received yet
    if(!m_cloth_valid){
        return 0;
    }

    const int xind = (x - m_cloth_origin(0)) / m_cloth_resolution;
    const int yind = (y - m_cloth_origin(1)) / m_cloth_resolution;
    
    const int height_ind = yind * m_cloth_width + xind;

    // Fail if out of bounds
    // NOTE: We *should* never fail this check as long as the cloth data is up to date
    if(height_ind < 0 || height_ind >= m_cloth_heights.size()) return 0;

    return m_cloth_heights[height_ind];
}
