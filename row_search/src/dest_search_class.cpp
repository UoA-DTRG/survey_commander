#include "ros/ros.h"
#include <cmath>
#include "dest_search.h"
#include "timer.hpp"
#include "pcl_conversions/pcl_conversions.h"

dest_search::dest_search(tf::TransformListener* l){
    // Invalidate our trig cache so we can compute it when the first laser scan arrives
    m_trig_cache_valid = false;
    
    m_tf_listener = l;
    m_process_mutex = false;

    init_parameters();

    m_outlier_filter = new outlier_filter(m_params.row_outlier_filter_size, m_params.row_outlier_gate_size);

    ros::NodeHandle n;
    init_subscribers(n);
    init_publishers(n);
}

void dest_search::init_parameters(){
    ros::NodeHandle pn("~");

    pn.param("laser_scan_topic", m_params.laser_scan_topic, string("/uav/laser/scan"));

    // Pointcloud stuff
    pn.param("point_cloud_topic", m_params.point_cloud_topic, string("/path_search/ESDFMap/occ_pc"));
    pn.param("point_cloud_band", m_params.point_cloud_band, 0.2);
    pn.param("world_frame", m_params.world_frame, string("odom_frame"));
    pn.param("uav_frame", m_params.uav_frame, string("fcu_link"));

    pn.param("search_range", m_laser_max_range, 80.0);

    pn.param("dbscan_epsilon", m_params.dbscan_epsilon, 0.4);
    // We square epsilon here so we don't have to compute squareroots later. This speeds things up when we compute neighbors for DBSCAN
    m_params.dbscan_epsilon_squared = m_params.dbscan_epsilon * m_params.dbscan_epsilon;
    pn.param("dbscan_min_neighbors", m_params.dbscan_min_neighbors, 4.0);

    pn.param("ransac_max_iterations", m_params.ransac_max_iterations, 100.0);
    pn.param("ransac_inlier_max_deviation", m_params.ransac_inlier_max_deviation, 0.06);
    pn.param("ransac_min_compliance_ratio", m_params.ransac_min_compliance_ratio, 0.9);
    pn.param("ransac_max_radius", m_params.ransac_max_radius, 0.8);
    pn.param("ransac_min_radius", m_params.ransac_min_radius, 0.05);

    pn.param("y_fov_max_distance", m_params.y_fov_max_distance, 8.0);
    pn.param("hough_angle_horizontal_tolerance", m_params.hough_angle_horizontal_tolerance, 0.1745329);

    pn.param("overlap_row_distance_threshold", m_params.overlap_row_distance_threshold, 1.5);
    pn.param("same_row_threshold", m_params.same_row_threshold, 1.5);
    pn.param("row_outlier_gate_size", m_params.row_outlier_gate_size, 0.3);
    pn.param("row_outlier_filter_size", m_params.row_outlier_filter_size, 20);
    pn.param("stale_cull_time", m_params.stale_cull_time, 5.0);
}

void inline dest_search::init_subscribers(ros::NodeHandle n){
    //m_laser_scan_subscriber = n.subscribe(m_params.laser_scan_topic, 1, &dest_search::update_laser_scan, this);
    m_pointcloud_subscriber = n.subscribe(m_params.point_cloud_topic, 1, &dest_search::update_pointcloud, this);
}

void inline dest_search::init_publishers(ros::NodeHandle n){
    m_visualization_pub = n.advertise<visualization_msgs::MarkerArray>("/uav/vis/centers", 10);
    m_out_publisher = n.advertise<uav_messages::LeftRightRow>("/uav/dat/left_right_rows", 10);
    m_rowinfo_publisher = n.advertise<std_msgs::Float64MultiArray>("/uav/dat/rows", 5);
}

void dest_search::update_pointcloud(const sensor_msgs::PointCloudConstPtr cloud){
    if(m_process_mutex) return;
    sensor_msgs::PointCloud2 pc2;
    sensor_msgs::convertPointCloudToPointCloud2(*cloud, pc2);
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
    pcl::fromROSMsg(pc2, pcl_cloud);

    // Get current location
    tf::StampedTransform odom_to_fcu;
    try{
        auto now = ros::Time::now();
        m_tf_listener->waitForTransform(m_params.world_frame, m_params.uav_frame, now, ros::Duration(0.5));
        m_tf_listener->lookupTransform(m_params.world_frame, m_params.uav_frame, now, odom_to_fcu);
    }catch(tf::TransformException ex){
        ROS_ERROR("Failed to get world to uav tf: %s", ex.what());
        return;
    }    

    double z = odom_to_fcu.getOrigin().z();
    tf::Quaternion q = odom_to_fcu.getRotation();
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // Passthrough filter
    pcl::PassThrough<pcl::PointXYZ> pass;
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_ptr;
    source_ptr = pcl_cloud.makeShared();
    pass.setInputCloud(source_ptr);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z - m_params.point_cloud_band, z + m_params.point_cloud_band);
    pass.filter(cloud_filtered);

    m_scan_cartesian.clear();

    pcl::PointCloud<pcl::PointXYZ>::iterator it;

    for(it = cloud_filtered.points.begin(); it < cloud_filtered.points.end(); it++){
        point2 p;

        p.x = it->x;
        p.y = it->y;

        //p.x -= odom_to_fcu.getOrigin().x();
        //p.y -= odom_to_fcu.getOrigin().y();

        //double temp = p.x;
        //p.x = p.x * cos(-yaw) - p.y * sin(-yaw);
        //p.y = temp * sin(-yaw) + p.y * cos(-yaw);

        m_scan_cartesian.push_back(p);
    }

    process_laser_scan();
}

void dest_search::update_laser_scan(const sensor_msgs::LaserScan scan){
    m_frame_id = scan.header.frame_id;
    m_laser_max_range = scan.range_max;

    if(!m_trig_cache_valid){
        tick();
        // We haven't computed our trig cache yet. Compute it now for speed
        m_trig_cache_valid = true;

        // malloc memory for our cache
        m_cos_cache = (double*) malloc(sizeof(double) * scan.ranges.size());
        m_sin_cache = (double*) malloc(sizeof(double) * scan.ranges.size());
        
        for(double i = 0; i < scan.ranges.size(); ++i){
            double angle = i * scan.angle_increment + scan.angle_min;
            m_cos_cache[(int) i] = cos(angle);
            m_sin_cache[(int) i] = sin(angle);
        }
        printf("Finished generating trig cache in %lu microseconds\n", tock());
    }

    m_scan_cartesian.clear();
    
    tick();
    for(size_t i = 0; i < scan.ranges.size(); ++i){
        double r = scan.ranges[i];
        // Check to make sure our values are within the allowed range
        if(r < scan.range_max && r > scan.range_min){
            point2 p;

            // This automatically puts us into standard ROS coordinate frame
            p.x = m_cos_cache[i] * r;
            p.y = m_sin_cache[i] * r;

            m_scan_cartesian.push_back(p);
        }
    }

    printf("Got laser scan of size: %ld\n", scan.ranges.size());
    printf("Generated cartesian form of size: %ld in %lu microseconds\n", m_scan_cartesian.size(), tock());

    process_laser_scan();
}

void dest_search::process_laser_scan(){
    m_process_mutex = true;
    tick();
    vector<cluster> clusters = dbscan();

    printf("Found %lu DBSCAN Clusters in %lu microseconds\n", clusters.size(), tock());

    tick();
    /*vector<point2r> centers;
    point2r p;
    for(cluster c : clusters){
        if(ransac_circle_search(c, p)){
            centers.push_back(p);
        }
    }*/

    vector<point2r> centers_no_filter = compute_centroids(clusters);

    printf("Found %lu stem candidates in %lu microseconds\n", centers_no_filter.size(), tock());

    tick();
    vector<line_r_theta> rows = hough_line_search(centers_no_filter, 1000);
    vector<line_r_theta> rows_complete = rows;
    printf("Found %lu row candidates in %lu microseconds\n", rows.size(), tock());

    tick();
    int bin_size = 3;
    double angle = bin_hough_lines(rows, bin_size);
    const double deg_to_rad_factor = 0.0174533;
    // Precompute hi and lo boundaries
    double hi = ((double)(bin_size / 2)) * deg_to_rad_factor + angle;
    double lo = -((double)(bin_size / 2)) * deg_to_rad_factor + angle;

    for(size_t i = 0; i < rows.size(); ++i){
        if(rows[i].theta < lo || rows[i].theta > hi || fabs(rows[i].r) < 0.2){
            rows.erase(rows.begin() + i);
            i--;
        }
    }

    // Add angle to outlier filter and stop if the sample is an outlier
    if(!m_outlier_filter->add_sample(angle)) {
        m_process_mutex = false;
        return;
    }

    sort(rows.begin(), rows.end());
    auto rows_collated = collate_rows(rows);
    update_likely_rows(rows_collated);

    for(auto r : m_likely_rows){
        printf("Row at R: %f, T: %f\n", r.r, r.theta);
    }

    printf("Updated global to %lu row candidates in %lu microseconds\n", m_likely_rows.size(), tock());

    // Hack to visualise the left and right rows
    auto likely_rows = publish_likely_rows(rows_collated);

    m_process_mutex = false;

    visualise_centers(centers_no_filter, likely_rows, rows_collated);
}

vector<line_r_theta> dest_search::collate_rows(vector<line_r_theta> row_list){
    double dist_accum, theta_accum, count = 0;
    bool start = false;
    vector<line_r_theta> ret;

    for(int i = 0; i < row_list.size(); ++i){
        auto r = row_list[i];
        // Start a new cluster of lines if we haven't already
        if(!start){
            // Push to output if this is not the first sample
            if(i != 0){
                dist_accum /= count;
                theta_accum /= count;
                line_r_theta t;
                t.r = dist_accum;
                t.theta = theta_accum;
                ret.push_back(t);
            }
            // Start a new cluster
            count = 0;
            dist_accum = r.r;
            theta_accum = r.theta;
            count++;
            start = true;
            continue;
        }
        
        // Stop the cluster if it is sufficiently far from the initial cluster
        if(fabs(row_list[i].r - row_list[i-1].r) > m_params.overlap_row_distance_threshold){
            start = false;
            i--;
            continue;
        }

        dist_accum += r.r;
        theta_accum += r.theta;
        count++;
    }

    return ret;
}

void dest_search::update_likely_rows(vector<line_r_theta> row_list){
    for(auto r : row_list){
        bool found = false;
        for(int i = 0; i < m_likely_rows.size(); ++i){
            // Check if the row is sufficiently similar to an already detected row
            if(fabs(r.r - m_likely_rows[i].r) < m_params.same_row_threshold){
                found = true;
                m_likely_rows[i] += r;
                m_likely_rows[i] /= 2;
            }
        }
        // If not found then add to the list of rows
        if(!found){
            m_likely_rows.push_back(r);
        }
    }

    // Filter through the list of rows
    for(int i = m_likely_rows.size() - 1; i >= 0; --i){
        if(!m_outlier_filter->check_sample(m_likely_rows[i].theta))
            m_likely_rows.erase(m_likely_rows.begin() + i);
    }
}

vector<cluster> dest_search::dbscan(){
    // Get the size of our input data
    int size = m_scan_cartesian.size();

    // Define our label vector
    vector<int> label(size, -1);
    // Labels are as follows:
    // -1 : Undefined
    // -2 : Noise
    //  1+: Cluster number

    int c = 0;
    for(size_t i=0; i<size; ++i){
        if(label[i] != -1) continue;

        vector<int> neighbors = dbscan_neighbors(m_scan_cartesian[i]);
        // Label point as noise if it doesn't have the required number of neighbors
        if(neighbors.size() < m_params.dbscan_min_neighbors){
            label[i] = -2;
            continue;
        }

        c++;
        label[i] = c;

        // Check through all the neighbors
        for(size_t j=0; j<neighbors.size(); ++j){
            int ind = neighbors[j];
            // If the pointis labeled as noise then add it as part of this set
            if(label[ind] == -2) label[ind] = c;
            // Continue if the point is not undefined
            if(label[ind] != -1) continue;

            // The point is undefined, so we classify it as part of this set
            label[ind] = c;

            vector<int> sub_neighbors = dbscan_neighbors(m_scan_cartesian[ind]);
            for(auto a : sub_neighbors){
                neighbors.push_back(a);
            }
        }
    }

    // Collate into clusters
    vector<cluster> ret;
    cluster cl;
    for(size_t i=0; i<=c; ++i){
        cl.clear();
        for(size_t j=0; j<size; ++j){
            if(label[j] == i){
                cl.push_back(m_scan_cartesian[j]);
            }
        }
        if(cl.size() > 0) ret.push_back(cl);
    }

    return ret;

}

vector<int> dest_search::dbscan_neighbors(point2 p){
    int size = m_scan_cartesian.size();
    vector<int> ret;

    // Create vector of neighbor points based on the epsilon
    for(size_t i=0; i<size; ++i){
        if(DISTSQR(p, m_scan_cartesian[i]) < m_params.dbscan_epsilon_squared){
            ret.push_back(i);
        }
    }
    
    return ret;
}

bool dest_search::ransac_circle_search(cluster c, point2r &center){
    // Seed RNG for our RANSAC search
	srand(time(NULL));

	// We don't check for size here because to form a cluster from DBSCAN we need at least N nearest neighbors
    // As long as N is larger than 3 we should have no problems
    int size = c.size();

	for(size_t i=0; i<100; ++i){
		float x1, x2, x3;
		float y1, y2, y3;
		int i1, i2, i3;

		// Pick 3 random points
		i1 = rand() % size;
		i2 = rand() % size;
		int count = 0;
		while((i1 == i2) && ++count < RANSAC_MAX_GEN_RUNS) i2 = rand() % size;
		i3 = rand() % size;
		count = 0;
		while((i1 == i3 || i2 == i3) && ++count < RANSAC_MAX_GEN_RUNS) i3 = rand() % size;

		x1 = c[i1].x;
		x2 = c[i2].x;
		x3 = c[i3].x;
		
		y1 = c[i1].y;
		y2 = c[i2].y;
		y3 = c[i3].y;

		// Calculate circle center from three points
		float A, B, C, D;
		A = x1 * (y2 - y3) - y1 * (x2 - x3) + (x2 * y3) - (x3 * y2);
		B = (x1 * x1 + y1 * y1) * (y3 - y2) + (x2 * x2 + y2 * y2) * (y1 - y3) + (x3 * x3 + y3 * y3) * (y2 - y1);
		C = (x1 * x1 + y1 * y1) * (x2 - x3) + (x2 * x2 + y2 * y2) * (x3 - x1) + (x3 * x3 + y3 * y3) * (x1 - x2);
		D = (x1 * x1 + y1 * y1) * (x3 * y2 - x2 * y3) + (x2 * x2 + y2 * y2) * (x1 * y3 - x3 * y1) + (x3 * x3 + y3 * y3) * (x2 * y1 - x1 * y2);

		float x, y, r;
		x = -B / (2 * A);
		y = -C / (2 * A);
		r = sqrt((B * B + C * C - 4 * A * D) / (4 * A * A));

		// Test every point for compliance
		int numCompliant = 0;
		for(size_t j=0; j<size; ++j){
			double xp = c[j].x, yp = c[j].y;
			double diffX = xp - x, diffY = yp - y;
			double dist = sqrt(diffX * diffX + diffY * diffY);
			if(abs(dist - r) <= m_params.ransac_inlier_max_deviation) numCompliant++;
		}

		double complianceRatio = (double) numCompliant / (double) size;

        // Return 
		if(complianceRatio > m_params.ransac_min_compliance_ratio &&
            r < m_params.ransac_max_radius &&
            r > m_params.ransac_min_radius){
			center.x = x;
            center.y = y;
            center.r = r;
            return true;
		}
	}

	// We've exhausted RANSAC attempts and not found a fitting circle. Just mark it as incomplete and move on
	return false;
}

vector<line_r_theta> dest_search::hough_line_search(vector<point2r> points, unsigned int num_candidates){
    constexpr double hough_resolution = 0.3;
    const double offset = m_laser_max_range / hough_resolution;
    constexpr double deg_to_rad_factor = 0.0174533;
    constexpr int angle_num = 90;
    constexpr double angle_resolution = 360 / (angle_num * 2);

    const double round_scale = (1.0 / hough_resolution);
    #define ROUND(x) (int) (x * round_scale)
    const int accumulator_r_size = (m_laser_max_range / hough_resolution) * 2 + 1;

    // Define our hough transform accumulator
    vector<vector<int>> accumulator;
    // Index 1 is angle, so we reserve between -90 deg to 90 deg, in 1 deg increments. We don't need 180 deg because that's identical to 0 deg
    accumulator.reserve(angle_num);
    // Preallocate accumulator size
    for(int i = 0; i < 180; ++i){
        vector<int> r_row(accumulator_r_size, 0);
        accumulator.push_back(r_row);
    }

    point2 zero;
    zero.x = 0;
    zero.y = 0;
    for(auto p : points){
        int weight = m_laser_max_range - sqrt(DISTSQR(p, zero));
        if(weight < 0) weight = 0;
        weight = 1;
        for(double angle = 45; angle < 135; angle+=1){
            double r = p.x * cos(angle * deg_to_rad_factor) + p.y * sin(angle * deg_to_rad_factor);
            int ind = ((int) (ROUND(r) + offset));
            if(ind >= accumulator_r_size || ind < 0) continue;
            accumulator[(int)angle][(int) (ROUND(r) + offset)]++;
            //accumulator[(int)angle][(int) (ROUND(r) + offset)]+= weight;
        }
    }

    vector<line_r_theta> ret;
    // Get top n line candidates
    for(size_t count = 0; count < num_candidates; ++count){
        // Get max indexes 
        int max = 0;
        int angleMax = -1;
        int rMax = -1;
        for(size_t i = 0; i < 180; i++){
            for(size_t j = 0; j < accumulator_r_size; ++j){
                if(accumulator[i][j] > max){
                    max = accumulator[i][j];
                    angleMax = i;
                    rMax = j;
                }
            }
        }
        // If max is < point threshold then end, we don't have any more good lines
        if(max < 3) break;
        // Push max index to ret vector
        line_r_theta out;
        out.r = (((double) rMax) - offset) * hough_resolution;
        out.theta = ((double) angleMax) * deg_to_rad_factor;
        accumulator[angleMax][rMax] = 0;
        ret.push_back(out);
    }

    return ret;
}

double dest_search::bin_hough_lines(vector<line_r_theta> lines, int avg_bins){
    const double deg_to_rad_factor = 0.0174533;
    vector<int> bins(180, 0);

    // Filter out duplicates by R and theta
    /*const double min_r_tolerance = 0.7;
    const double min_theta_tolerance = ((double) avg_bins / 2) * deg_to_rad_factor;
    for(size_t i = 0; i < lines.size() - 1; ++i){
        for(size_t j = i + 1; j < lines.size(); ++j){
            // If the radius is sufficiently similar
            if(fabs(lines[i].r - lines[j].r) < min_r_tolerance){
                // If the theta is also similar
                if(fabs(lines[i].theta - lines[j].theta) < min_theta_tolerance){
                    // Remove element j
                    lines.erase(lines.begin() + j);
                    j--;
                }
            }
        }
    }*/

    // Filter out any near horizontal lines
    for(size_t i = 0; i < lines.size(); ++i){
        // Remove from list if angle of the line is near horizontal (+- hough_angle_horizontal_tolerance)
        if(fabs(lines[i].theta - (PI/2)) > m_params.hough_angle_horizontal_tolerance){
            lines.erase(lines.begin() + i);
            i--;
        }
    }

    // Bin all the data so we have a histogram of each angle
    for(auto l : lines){
        double deg = l.theta / deg_to_rad_factor;
        bins[(int) deg]++;
    }

    vector<int> count_bins(180, 0);
    // Find the cumulative sum over n bins to emphasise biggest peak
    for(int i = 0; i < 180; ++i){
        int sum = 0;
        for(int j = -(avg_bins / 2); j <= (avg_bins / 2); ++j){
            int ind = j + i;
            // Fix cases where ind is out of bounds by wrapping the index
            if(ind < 0){
                ind += 180;
            }
            if(ind > 179){
                ind -= 180;
            }
            sum += bins[ind];
        }
        count_bins[i] = sum;
    }

    // Return the most likely angle
    int max_ind = -1;
    int max = -999;
    for(int i = 0; i < 180; ++i){
        if(count_bins[i] > max){
            max = count_bins[i];
            max_ind = i;
        }
    }

    return ((double) max_ind) * deg_to_rad_factor;
}

vector<point2r> dest_search::compute_centroids(vector<cluster> clusters){
    vector<point2r> ret;
    
    point2r p;
    for(cluster c : clusters){
        // Compute our non ransac filtered cluster
        p.x = 0;
        p.y = 0;
        for(point2 point : c){
            p.x += point.x;
            p.y += point.y;
        }
        p.x /= c.size();
        p.y /= c.size();

        p.r = c.size() * 0.05;

        ret.push_back(p);
    }

    return ret;
}

void dest_search::restrict_y_fov(vector<point2r> &centers){
    for(size_t i=0; i < centers.size(); ++i){
        if(fabs(centers[i].y) > m_params.y_fov_max_distance){
            centers.erase(centers.begin() + i);
            i--;
        }
    }
}

vector<line_r_theta> dest_search::publish_likely_rows(vector<line_r_theta> lines){
    // Publish global row list first
    m_row_info.data.clear();
    for(auto r : m_likely_rows){
        m_row_info.data.push_back(r.r);
        m_row_info.data.push_back(r.theta);
    }

    m_rowinfo_publisher.publish(m_row_info);

    vector<line_r_theta> ret;
    // We have to have at least 2 lines to do this
    if(lines.size() < 2) return ret;

    double min_neg_r = 0;
    double min_neg_theta = M_PI / 2;
    double min_pos_r = 0;
    double min_pos_theta = M_PI / 2;

    double min_l = 9999;
    double min_r = -9999;

    for(auto l : lines){
        // Find y intercept
        //double y_int = l.r * tan(l.theta);
        double y_int = l.r / sin(l.theta);
        if(y_int > 0){
            if(y_int < min_l){
                min_l = y_int;
                min_neg_r = l.r;
                min_neg_theta = l.theta;
            }
        }else{
            if(y_int > min_r){
                min_r = y_int;
                min_pos_r = l.r;
                min_pos_theta = l.theta;
            }
        }
    }

    vector<line_r_theta> left_cluster;
    vector<line_r_theta> right_cluster;

    constexpr double r_tolerance = 0.7;
    for(auto l : lines){
        if(fabs(min_neg_r - l.r) < r_tolerance) left_cluster.push_back(l);
        if(fabs(min_pos_r - l.r) < r_tolerance) right_cluster.push_back(l);
    }

    min_neg_r = 0;
    min_neg_theta = 0;
    for(auto l : left_cluster){
        min_neg_r += l.r;
        min_neg_theta += l.theta;
    }
    min_neg_r /= left_cluster.size();
    min_neg_theta /= left_cluster.size();

    min_pos_r = 0;
    min_pos_theta = 0;
    for(auto l : right_cluster){
        min_pos_r += l.r;
        min_pos_theta += l.theta;
    }
    min_pos_r /= right_cluster.size();
    min_pos_theta /= right_cluster.size();

    m_out_msg.left_r = min_neg_r;
    m_out_msg.left_theta = min_neg_theta;
    m_out_msg.right_r = min_pos_r;
    m_out_msg.right_theta = min_pos_theta;

    line_r_theta l1, l2;
    
    l1.r = min_neg_r;
    l1.theta = min_neg_theta;
    l2.r = min_pos_r;
    l2.theta = min_pos_theta;
    ret.push_back(l1);
    ret.push_back(l2);

    m_out_publisher.publish(m_out_msg);

    return ret;
}

void dest_search::visualise_centers(vector<point2r> centers, vector<line_r_theta> lines,  vector<line_r_theta> lines_complete){
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
    m.scale.z = 1;
    m.color.a = 1.0;
    m.color.r = 1.0;
    m.color.g = 0;
    m.color.b = 0;

    m_vis_array.markers.clear();

    int id = 0;
    for(auto p : centers){
        if(p.r > 1.0){
            m.pose.position.x = p.x;
            m.pose.position.y = p.y;
            m.id = id++;
            m.scale.x = p.r * 0.2;
            m.scale.y = p.r * 0.2;
            m_vis_array.markers.push_back(m);
        }
    }

    // Make line_list to visualise found lines
    m.type = visualization_msgs::Marker::LINE_LIST;
    m.color.b = 1.0;
    m.color.r = 0.0;
    m.scale.x = 0.1;
    m.pose.position.x = 0;
    m.pose.position.y = 0;
    m.id = id++;

    const double projection_length = 30;
    
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

    // Make line_list to visualise found lines
    m.type = visualization_msgs::Marker::LINE_LIST;
    m.color.b = 0.0;
    m.color.g = 1.0;
    m.scale.x = 0.02;
    m.pose.position.x = 0;
    m.pose.position.y = 0;
    m.id = id++;
    
    for(auto l : lines_complete){
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

