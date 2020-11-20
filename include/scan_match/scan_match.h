#ifndef SCAN_MATCH_H_
#define SCAN_MATCH_H_

#include <scan_match/utils.h>

class ScanMatch {
public:
    ScanMatch(double linear_search_window_, double angular_search_window_,
              double resolution_, int width_, int height_);
    
    void updateMap(const sensor_msgs::LaserScan::ConstPtr& laser);

    double MatchBruteForce(const geometry_msgs::Pose2D& initial_pose,
                         const PointCloud& point_cloud,
                         geometry_msgs::Pose2D& pose_estimated);

private:
    PointCloud RotatePointCloud(const PointCloud& in_cloud, const Eigen::Quaterniond& rotation);

    std::vector<PointCloud> GenerateSans(const PointCloud& point_cloud, const SearchParameters& search_parameters);

    std::vector<std::vector<Eigen::Array2i>> GenerateDiscreteScans(const std::vector<PointCloud>& scans, const geometry_msgs::Pose2D& pose);

    Eigen::Array2i getGrid2D(const Eigen::Vector2d& point);

    std::vector<Candidate> GenerateCandidates(const SearchParameters& search_parameters);

    void ScoreCandidates(const std::vector<std::vector<Eigen::Array2i>>& scans, const SearchParameters& search_parameters, std::vector<Candidate>* candidates);

    float ComputeScore(const std::vector<Eigen::Array2i>& scan, int x_offset, int y_offset);

    float getProbability(int cx, int cy);

private:
    double linear_search_window;
    double angular_search_window;
    nav_msgs::OccupancyGrid map;
};

#endif
