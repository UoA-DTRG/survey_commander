#pragma once

#include <vector>

#include "Eigen/Eigen"

using namespace std;

typedef vector<Eigen::Vector2d> cluster;

// Function prototypes
static vector<int> dbscan_neighbors(vector<Eigen::Vector2d> points, Eigen::Vector2d p, double epsilon);
static vector<cluster> dbscan(vector<Eigen::Vector2d> points, double epsilon, double min_neighbors);
static vector<cluster> dbscan(vector<Eigen::Vector2d> points, double epsilon, double min_neighbors, double& outliers);
static vector<int> dbscan_labels(vector<Eigen::Vector2d> points, double epsilon, double min_neighbors, double& outliers);
static Eigen::Vector2d cluster_mean(cluster c);
static Eigen::Vector2d get_cluster_range(cluster c);


static vector<int> dbscan_neighbors(vector<Eigen::Vector2d> points, Eigen::Vector2d p, double epsilon){
    int size = points.size();
    vector<int> ret;

    // Create vector of neighbor points based on the epsilon
    for(size_t i=0; i<size; ++i){
        if((points[i] - p).norm() < epsilon){
            ret.push_back(i);
        }
    }
    
    return ret;
}

static vector<cluster> dbscan(vector<Eigen::Vector2d> points, double epsilon, double min_neighbors){
    double outliers = 0;
    return dbscan(points, epsilon, min_neighbors, outliers);
}

static vector<cluster> dbscan(vector<Eigen::Vector2d> points, double epsilon, double min_neighbors, double& outliers){
    // Get the size of our input data
    int size = points.size();

    auto label = dbscan_labels(points, epsilon, min_neighbors, outliers);
    int c = 0;
    for(auto l : label) c = (l > c) ? l : c;

    // Collate into clusters
    vector<cluster> ret;
    for(size_t i=0; i<=c; ++i){
        cluster cl;
        for(size_t j=0; j<size; ++j){
            if(label[j] == i){
                cl.push_back(points[j]);
            }
        }
        if(cl.size() > 0) ret.push_back(cl);
    }

    return ret;

}

static vector<int> dbscan_labels(vector<Eigen::Vector2d> points, double epsilon, double min_neighbors, double& outliers){
    // Get the size of our input data
    int size = points.size();

    // Define our label vector
    vector<int> label(size, -1);
    // Labels are as follows:
    // -1 : Undefined
    // -2 : Noise
    //  1+: Cluster number

    int c = 0;
    for(size_t i=0; i<size; ++i){
        if(label[i] != -1) continue;

        vector<int> neighbors = dbscan_neighbors(points, points[i], epsilon);
        // Label point as noise if it doesn't have the required number of neighbors
        // Note, dbscan_neighbors will always find itself as a neighbor, so the actual number of neighbors is one less
        if(neighbors.size() <= (min_neighbors - 1)){
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

            vector<int> sub_neighbors = dbscan_neighbors(points, points[ind], epsilon);
            for(auto a : sub_neighbors){
                neighbors.push_back(a);
            }
        }
    }

    outliers = 0;
    for(auto l : label){
        if(l < 0) outliers++;
    }

    return label;
}

static Eigen::Vector2d cluster_mean(cluster c){
    double x = 0;
    double y = 0;
    double count = 0;
    for(auto p : c){
        x += p(0);
        y += p(1);
        count++;
    }    

    Eigen::Vector2d ret;
    ret(0) = (x / count);
    ret(1) = (y / count);

    return ret;
}

static Eigen::Vector2d get_cluster_range(cluster c){
    Eigen::Vector2d min_d;
    Eigen::Vector2d max_d;

    min_d << 99999, 99999;
    max_d << -99999, -99999;

    for(auto p : c){
        if(p(0) > max_d(0)) max_d(0) = p(0);
        if(p(0) < min_d(0)) min_d(0) = p(0);

        if(p(1) > max_d(1)) max_d(1) = p(1);
        if(p(1) < min_d(1)) min_d(1) = p(1);
    }

    return max_d - min_d;
}

static double get_cluster_residual(cluster c){
    double residual = 0;
    auto range = get_cluster_range(c);
    for(auto p : c){
        residual += (p - range).norm();
    }

    return residual;
}