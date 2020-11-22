#ifndef SCAN_MATCH_H_
#define SCAN_MATCH_H_

#include "utils.h"

#include <ros/ros.h>
#include <algorithm>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <unordered_map>

#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>

class ScanMatch {
public:
    
    ScanMatch(double linear_search_window_, 
              double angular_search_window_, 
              int branch_and_bound_depth_);

    geometry_msgs::Pose2D MatchBruteForce(const nav_msgs::OccupancyGrid& map,
                                          const geometry_msgs::Pose2D& initial_pose,
                                          const PointCloud& point_cloud);
    
    geometry_msgs::Pose2D MatchMultiResolution(const nav_msgs::OccupancyGrid& map,
                                               const geometry_msgs::Pose2D& initial_pose, 
                                               const PointCloud& point_cloud, 
                                               float min_score);


    PointCloud RotatePointCloud(const PointCloud& in_cloud, const Eigen::Quaterniond& rotation);

    std::vector<PointCloud> GenerateSans(const PointCloud& point_cloud, const SearchParameters& search_parameters);

    std::vector<std::vector<Eigen::Array2i>>
    GenerateDiscreteScans(const nav_msgs::OccupancyGrid& map, 
                          const std::vector<PointCloud>& scans, 
                          const geometry_msgs::Pose2D& pose);

    Eigen::Array2i getGrid2D(const nav_msgs::OccupancyGrid& map, const Eigen::Vector2d& point);

    std::vector<Candidate> GenerateCandidates(const SearchParameters& search_parameters);

    void ScoreCandidates(const nav_msgs::OccupancyGrid& map,
                         const std::vector<std::vector<Eigen::Array2i>>& scans,
                         const SearchParameters& search_parameters, 
                         std::vector<Candidate>& candidates);

    float ComputeScore(const nav_msgs::OccupancyGrid& map,
                       const std::vector<Eigen::Array2i>& scan,
                       int x_offset, int y_offset);

    float getProbability(const nav_msgs::OccupancyGrid& map, int cx, int cy);
    
    PointCloud ScanToCloud(const sensor_msgs::LaserScan& scan);
    
    std::vector<nav_msgs::OccupancyGrid> GenerateLookUpTables(const nav_msgs::OccupancyGrid& map);
    
    nav_msgs::OccupancyGrid CompressMap(const nav_msgs::OccupancyGrid& map, const int& size);
    
    Candidate BranchAndBound(const std::vector<nav_msgs::OccupancyGrid>& maps,
                             const std::vector<std::vector<std::vector<Eigen::Array2i>>>& discrete_scans,
                             const std::vector<SearchParameters>& search_parameters,
                             const std::vector<Candidate>& candidates, int candidate_depth,
                             float min_score);

private:
    double linear_search_window;
    double angular_search_window;
    int branch_and_bound_depth;
};

#endif
