#include <scan_match/scan_match.h>

ScanMatch::ScanMatch(double linear_search_window_, double angular_search_window_,
                     double resolution_, int width_, int height_)
            :linear_search_window(linear_search_window_),
             angular_search_window(angular_search_window_)
{
    map.info.resolution = resolution_;
    map.info.width = width_;
    map.info.height = height_;
    map.data.resize(width_ * height_);
}

double ScanMatch::MatchBruteForce(const geometry_msgs::Pose2D& initial_pose,
                       const PointCloud& point_cloud,
                       geometry_msgs::Pose2D& pose_estimated)
{
    Eigen::AngleAxisd init_r_vector(initial_pose.theta, Eigen::Vector3d(0, 0, 1)); //rotate theta around z
    Eigen::Quaterniond initial_rotation(init_r_vector);

    PointCloud rotated_cloud = RotatePointCloud(point_cloud, initial_rotation);

    SearchParameters search_parameters(linear_search_window, angular_search_window, point_cloud, map.info.resolution);

    std::vector<PointCloud> scans = GenerateSans(rotated_cloud, search_parameters);

    std::vector<std::vector<Eigen::Array2i>> discrete_scans = GenerateDiscreteScans(scans, initial_pose);

    std::vector<Candidate> candidates = GenerateCandidates(search_parameters);

    ScoreCandidates(discrete_scans, search_parameters, &candidates);

    const Candidate& best_candidate = *std::max_element(candidates.begin(), candidates.end());

    pose_estimated.x = initial_pose.x + best_candidate.x;
    pose_estimated.y = initial_pose.y + best_candidate.y;
    pose_estimated.theta = initial_pose.theta + best_candidate.orientation;

    return best_candidate.score;
}

PointCloud ScanMatch::RotatePointCloud(const PointCloud& in_cloud, const Eigen::Quaterniond& rotation)
{
    PointCloud transformed_cloud;
    for(const Eigen::Vector3d& point : in_cloud.points()) {
        transformed_cloud.push_back(rotation * point);
    }
    return transformed_cloud;
}

std::vector<PointCloud> ScanMatch::GenerateSans(const PointCloud& point_cloud, const SearchParameters& search_parameters)
{
    std::vector<PointCloud> scans;
    scans.reserve(search_parameters.num_scans);
    double delta_theta = -search_parameters.num_angular_steps * search_parameters.angular_step_size;
    for(int i=0; i < scans.size(); ++i, delta_theta += search_parameters.angular_step_size) {
        scans.push_back(RotatePointCloud(point_cloud, Eigen::Quaterniond(Eigen::AngleAxisd(delta_theta, Eigen::Vector3d(0, 0, 1)))));
    }
    return scans;
}

std::vector<std::vector<Eigen::Array2i>> ScanMatch::GenerateDiscreteScans(const std::vector<PointCloud>& scans, const geometry_msgs::Pose2D& pose)
{
    std::vector<std::vector<Eigen::Array2i>> discrete_scans;
    discrete_scans.reserve(scans.size());
    for(const PointCloud& scan : scans) {
        discrete_scans.emplace_back();
        discrete_scans.back().reserve(scan.size());
        for(const Eigen::Vector3d& point : scan) {
            Eigen::Vector2d translated_point = Eigen::Affine2d(Eigen::Translation2d(pose.x, pose.y)) * point.head<2>();
            discrete_scans.back().push_back(getGrid2D(translated_point));
        }
    }
    return discrete_scans;
}

Eigen::Array2i ScanMatch::getGrid2D(const Eigen::Vector2d& point)
{
    int cx = ceil((point.x() - map.info.origin.position.x) / map.info.resolution);
    int cy = ceil((point.y() - map.info.origin.position.y) / map.info.resolution);

    return Eigen::Array2i(cx, cy);
}

std::vector<Candidate> ScanMatch::GenerateCandidates(const SearchParameters& search_parameters)
{
    int num_candidates = 0;
    for(int k = 0; k<search_parameters.num_scans; ++k) {
        int num_x = (search_parameters.linear_bounds[k].max_x - search_parameters.linear_bounds[k].min_x + 1);
        int num_y = (search_parameters.linear_bounds[k].max_y - search_parameters.linear_bounds[k].min_y + 1);
        num_candidates += num_x * num_y;
    }

    std::vector<Candidate> candidates;
    candidates.reserve(num_candidates);

    for(int scan_index = 0; scan_index < search_parameters.num_scans; ++scan_index) {
        for(int x_offset = search_parameters.linear_bounds[scan_index].min_x; x_offset <= search_parameters.linear_bounds[scan_index].max_x; ++x_offset) {
            for(int y_offset = search_parameters.linear_bounds[scan_index].min_y; y_offset <= search_parameters.linear_bounds[scan_index].max_y; ++y_offset) {
                candidates.emplace_back(scan_index, x_offset, y_offset, search_parameters);
            }
        }
    }

    return candidates;
}

void ScanMatch::ScoreCandidates(const std::vector<std::vector<Eigen::Array2i>>& scans, const SearchParameters& search_parameters, std::vector<Candidate>* candidates)
{
    for(Candidate& candidate : *candidates) {
        candidate.score = ComputeScore(scans[candidate.scan_index], candidate.x_offset, candidate.y_offset);
        candidate.score *= std::exp(-std::pow((std::hypot(candidate.x, candidate.y)*0.1 + std::abs(candidate.orientation)*0.1),2));
    }
}

float ScanMatch::ComputeScore(const std::vector<Eigen::Array2i>& scan, int x_offset, int y_offset)
{
    float candidate_score = 0.f;
    for(const Eigen::Array2i& xy : scan) {
        Eigen::Array2i xy_offset(xy.x() + x_offset, xy.y() + y_offset);
        float probability = getProbability(xy_offset.x(), xy_offset.y());
        candidate_score += probability;
    }
    candidate_score /= static_cast<float>(scan.size());
    return candidate_score;
}

float ScanMatch::getProbability(int cx, int cy)
{
    if(cx < 0 || cx >= map.info.width || cy < 0 || cy >= map.info.height)
        return -1;
    return static_cast<float>(map.data[cx + cy * map.info.width]) / 100;
}