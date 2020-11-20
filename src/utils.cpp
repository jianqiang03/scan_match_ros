#include <scan_match/utils.h>

SearchParameters::SearchParameters(const double linear_search_window,
                                   const double angular_search_window,
                                   const PointCloud& point_cloud,
                                   const double resolution)
    : resolution(resolution)
{
    float max_scan_range = 3.f * resolution;

    for (const Eigen::Vector3d& point : point_cloud) {
        const float range = point.head<2>().norm();
        max_scan_range = std::max(range, max_scan_range);
    }
    const double kSafetyMargin = 1. - 1e-3;
    angular_step_size = kSafetyMargin * std::acos(1. - resolution * resolution /
                        (2. * max_scan_range * max_scan_range));
    num_angular_steps = std::ceil(angular_search_window / angular_step_size);
    num_scans = 2 * num_angular_steps + 1;

    const int num_linear_steps = std::ceil(linear_search_window / resolution);
    
    linear_bounds.reserve(num_scans);
    
    for (int i = 0; i < num_scans; ++i) {
        linear_bounds.push_back(
        LinearBounds{-num_linear_steps, num_linear_steps,
                     -num_linear_steps, num_linear_steps});
    }
}