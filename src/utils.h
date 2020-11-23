#ifndef UTILS_H_
#define UTILS_H_

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

//define point cloud

class PointCloud {
public:

    PointCloud() {}

    PointCloud(std::vector<Eigen::Vector3d> points)
        : points_(std::move(points)) {}
    
    const std::vector<Eigen::Vector3d>& points() const {
        return points_;
    }

    size_t size() const {
        return points_.size();
    }

    void push_back(Eigen::Vector3d value) {
        points_.push_back(std::move(value));
    }

    using ConstIterator = std::vector<Eigen::Vector3d>::const_iterator;

    ConstIterator begin() const {
        return points_.begin();
    }

    ConstIterator end() const {
        return points_.end();
    }

private:
    std::vector<Eigen::Vector3d> points_;
};

//define search window

struct SearchParameters {

  struct LinearBounds {
    int min_x;
    int max_x;
    int min_y;
    int max_y;
  };

  SearchParameters(double linear_search_window, double angular_search_window,
                   const PointCloud& point_cloud, double resolution_);
  
  SearchParameters(double linear_search_window, double angular_search_window,
                   const double angular_step_size_, double resolution_);

  int num_angular_steps;
  double angular_step_size;
  double resolution;
  int num_scans;
  std::vector<LinearBounds> linear_bounds;
};

struct Candidate {
  Candidate(const int scan_index_, const int x_offset_,
            const int y_offset_,
            const SearchParameters& search_parameters)
      : scan_index(scan_index_),
        x_offset(x_offset_),
        y_offset(y_offset_),
        x(x_offset * search_parameters.resolution),
        y(y_offset * search_parameters.resolution),
        orientation((scan_index - search_parameters.num_angular_steps) *
                    search_parameters.angular_step_size) {}

  // Index into the rotated scans vector.
  int scan_index = 0;

  // Linear offset from the initial pose.
  int x_offset = 0;
  int y_offset = 0;

  // Pose of this Candidate relative to the initial pose.
  double x = 0.;
  double y = 0.;
  double orientation = 0.;

  // Score, higher is better.
  float score = 0.f;

  bool operator<(const Candidate& other) const { return score < other.score; }
  bool operator>(const Candidate& other) const { return score > other.score; }
};

#endif
