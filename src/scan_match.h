#ifndef SCAN_MATCH_H_
#define SCAN_MATCH_H_

#include "utils.h"

#include <ros/ros.h>
#include <algorithm>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

//some ros massage for easy implementation
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>

class ScanMatch {
public:
    ScanMatch(double linear_search_window_, double angular_search_window_);

    geometry_msgs::Pose2D MatchBruteForce(const geometry_msgs::Pose2D& initial_pose,
                                          const PointCloud& point_cloud);
    
    
    void updateMap(const nav_msgs::OccupancyGrid& map_) {
        map = map_;
    }

    nav_msgs::OccupancyGrid getMap() {
        return map;
    }

    PointCloud RotatePointCloud(const PointCloud& in_cloud, const Eigen::Quaterniond& rotation);

    std::vector<PointCloud> GenerateSans(const PointCloud& point_cloud, const SearchParameters& search_parameters);

    std::vector<std::vector<Eigen::Array2i>> GenerateDiscreteScans(const std::vector<PointCloud>& scans, const geometry_msgs::Pose2D& pose);

    Eigen::Array2i getGrid2D(const Eigen::Vector2d& point);

    std::vector<Candidate> GenerateCandidates(const SearchParameters& search_parameters);

    void ScoreCandidates(const std::vector<std::vector<Eigen::Array2i>>& scans,
                         const SearchParameters& search_parameters, 
                         std::vector<Candidate>* candidates);

    float ComputeScore(const std::vector<Eigen::Array2i>& scan, int x_offset, int y_offset);

    float getProbability(int cx, int cy);
    
    PointCloud ScanToCloud(const sensor_msgs::LaserScan& scan);

private:
    double linear_search_window;
    double angular_search_window;
    nav_msgs::OccupancyGrid map;
};

#endif
