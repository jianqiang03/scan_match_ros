#include "scan_match.h"

ScanMatch::ScanMatch(double linear_search_window_,
                     double angular_search_window_,
                     int branch_and_bound_depth_)
            :linear_search_window(linear_search_window_),
             angular_search_window(angular_search_window_),
             branch_and_bound_depth(branch_and_bound_depth_) {}

geometry_msgs::Pose2D ScanMatch::MatchBruteForce(const nav_msgs::OccupancyGrid& map,
                                                 const geometry_msgs::Pose2D& initial_pose,
                                                 const PointCloud& point_cloud)
{
    
    Eigen::AngleAxisd init_r_vector(initial_pose.theta, Eigen::Vector3d(0, 0, 1));
    
    Eigen::Quaterniond initial_rotation(init_r_vector);

    PointCloud rotated_cloud = RotatePointCloud(point_cloud, initial_rotation);

    SearchParameters search_parameters(linear_search_window, angular_search_window, point_cloud, map.info.resolution);

    std::vector<PointCloud> scans = GenerateSans(rotated_cloud, search_parameters);

    std::vector<std::vector<Eigen::Array2i>> discrete_scans = GenerateDiscreteScans(map, scans, initial_pose);

    std::vector<Candidate> candidates = GenerateCandidates(search_parameters);

    ScoreCandidates(map, discrete_scans, search_parameters, candidates);

    const Candidate& best_candidate = *candidates.begin();
    
    geometry_msgs::Pose2D pose_estimated;

    pose_estimated.x = initial_pose.x + best_candidate.x;
    pose_estimated.y = initial_pose.y + best_candidate.y;
    pose_estimated.theta = initial_pose.theta + best_candidate.orientation;

    return pose_estimated;
}

geometry_msgs::Pose2D ScanMatch::MatchMultiResolution(const nav_msgs::OccupancyGrid& map,
                                                      const geometry_msgs::Pose2D& initial_pose,
                                                      const PointCloud& point_cloud, 
                                                      float min_score)
{        
    Eigen::AngleAxisd init_r_vector(initial_pose.theta, Eigen::Vector3d(0, 0, 1));
    
    Eigen::Quaterniond initial_rotation(init_r_vector);

    PointCloud rotated_cloud = RotatePointCloud(point_cloud, initial_rotation);
    
    std::vector<nav_msgs::OccupancyGrid> maps = GenerateLookUpTables(map);
    
    SearchParameters s1(linear_search_window, angular_search_window, point_cloud, maps[0].info.resolution);
    
    double angular_step_size = s1.angular_step_size;
    
    std::vector<SearchParameters> search_parameters;
    search_parameters.reserve(branch_and_bound_depth);
    
    for(int i=0; i < branch_and_bound_depth; ++i) {
        search_parameters.emplace_back(linear_search_window, angular_search_window,
                                       angular_step_size, maps[i].info.resolution);
    }

    std::vector<PointCloud> scans = GenerateSans(rotated_cloud, search_parameters.back());

    std::vector<std::vector<std::vector<Eigen::Array2i>>> discrete_scans;
    discrete_scans.reserve(branch_and_bound_depth);
    
    for(int i=0; i < branch_and_bound_depth; ++i) {
        discrete_scans.push_back(GenerateDiscreteScans(maps[i], scans, initial_pose));
    }
    
    std::vector<Candidate> lowest_resolution_candidates = GenerateCandidates(search_parameters.back());
    
    ScoreCandidates(maps.back(), discrete_scans.back(), search_parameters.back(), lowest_resolution_candidates);
    
    Candidate best_candidate = BranchAndBound(maps, discrete_scans, search_parameters,
                                              lowest_resolution_candidates,
                                              branch_and_bound_depth-1, min_score);
    
    geometry_msgs::Pose2D pose_estimated;

    pose_estimated.x = initial_pose.x + best_candidate.x;
    pose_estimated.y = initial_pose.y + best_candidate.y;
    pose_estimated.theta = initial_pose.theta + best_candidate.orientation;

    return pose_estimated;
    
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
    for(int i=0; i < search_parameters.num_scans; ++i, delta_theta += search_parameters.angular_step_size) {
        scans.push_back(RotatePointCloud(point_cloud, Eigen::Quaterniond(Eigen::AngleAxisd(delta_theta, Eigen::Vector3d(0, 0, 1)))));
    }
    return scans;
}

std::vector<std::vector<Eigen::Array2i>>
ScanMatch::GenerateDiscreteScans(const nav_msgs::OccupancyGrid& map,
                                 const std::vector<PointCloud>& scans,
                                 const geometry_msgs::Pose2D& pose)
{
    std::vector<std::vector<Eigen::Array2i>> discrete_scans;
    discrete_scans.reserve(scans.size());
    for(const PointCloud& scan : scans) {
        discrete_scans.emplace_back();
        discrete_scans.back().reserve(scan.size());
        for(const Eigen::Vector3d& point : scan) {
            Eigen::Vector2d translated_point = Eigen::Affine2d(Eigen::Translation2d(pose.x, pose.y)) * point.head<2>();
            discrete_scans.back().push_back(getGrid2D(map, translated_point));
        }
    }
    return discrete_scans;
}

Eigen::Array2i ScanMatch::getGrid2D(const nav_msgs::OccupancyGrid& map, const Eigen::Vector2d& point)
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

void ScanMatch::ScoreCandidates(const nav_msgs::OccupancyGrid& map,
                                const std::vector<std::vector<Eigen::Array2i>>& scans,
                                const SearchParameters& search_parameters,
                                std::vector<Candidate>& candidates)
{
    for(Candidate& candidate : candidates) {
        candidate.score = ComputeScore(map, scans[candidate.scan_index], candidate.x_offset, candidate.y_offset);
        candidate.score *= std::exp(-std::pow((std::hypot(candidate.x, candidate.y)*0.1 + std::abs(candidate.orientation)*0.1),2));
    }
    std::sort(candidates.begin(), candidates.end(), std::greater<Candidate>());
}


float ScanMatch::ComputeScore(const nav_msgs::OccupancyGrid& map,
                              const std::vector<Eigen::Array2i>& scan, 
                              int x_offset, int y_offset)
{
    float candidate_score = 0.f;
    for(const Eigen::Array2i& xy : scan) {
        Eigen::Array2i xy_offset(xy.x() + x_offset, xy.y() + y_offset);
        float probability = getProbability(map, xy_offset.x(), xy_offset.y());
        candidate_score += probability;
    }
    candidate_score /= static_cast<float>(scan.size());
    return candidate_score;
}

float ScanMatch::getProbability(const nav_msgs::OccupancyGrid& map, int cx, int cy)
{
    if(cx < 0 || cx >= map.info.width || cy < 0 || cy >= map.info.height)
        return -1;
    return static_cast<float>(map.data[cx + cy * map.info.width]) / 100;
}

std::vector<nav_msgs::OccupancyGrid> ScanMatch::GenerateLookUpTables(const nav_msgs::OccupancyGrid& map)
{
    std::vector<nav_msgs::OccupancyGrid> tables;
    tables.reserve(branch_and_bound_depth);
    for(int i=0; i < branch_and_bound_depth; ++i) {
        int size = 1 << i;
        nav_msgs::OccupancyGrid low_resolution_map = CompressMap(map, size);
        tables.push_back(low_resolution_map);
    }
    return tables;
}

nav_msgs::OccupancyGrid ScanMatch::CompressMap(const nav_msgs::OccupancyGrid& map, const int& size)
{
    nav_msgs::OccupancyGrid low_resolution_map;
    
    low_resolution_map.info.height = ceil(map.info.height / size);
    low_resolution_map.info.width = ceil(map.info.width / size);
    low_resolution_map.info.resolution = map.info.resolution * size;
    low_resolution_map.info.origin.position.x = -(low_resolution_map.info.width * 
                                            low_resolution_map.info.resolution) / 2;
    low_resolution_map.info.origin.position.y = -(low_resolution_map.info.height * 
                                            low_resolution_map.info.resolution) / 2;
    low_resolution_map.data.resize(low_resolution_map.info.width * low_resolution_map.info.height);
        
    std::unordered_map<int, std::vector<int>> id_x;
    std::unordered_map<int, std::vector<int>> id_y;
    
    for(int x = 0; x < map.info.width; ++x) {
        int x_index = x / size;
        id_x[x_index].push_back(x);
    }
    
    for(int y = 0; y < map.info.height; ++y) {
        int y_index = y / size;
        id_y[y_index].push_back(y);
    }
//    ROS_INFO("idx size is %f", (float)id_x[1][0]);
    
    for(int cx = 0; cx < low_resolution_map.info.width; ++cx) {
        for(int cy = 0; cy < low_resolution_map.info.height; ++cy) {
            int id = cx + cy * low_resolution_map.info.width;
            low_resolution_map.data[id] = -1;
            for(int x : id_x[cx]) {
                for(int y : id_y[cy]) {
                    if(map.data[x + y * map.info.width] > low_resolution_map.data[id]) {
                        low_resolution_map.data[id] = map.data[x + y * map.info.width];
                    }
                }
            }
        }
    }
    
    return low_resolution_map;
}


Candidate ScanMatch::BranchAndBound(const std::vector<nav_msgs::OccupancyGrid>& maps,
                                    const std::vector<std::vector<std::vector<Eigen::Array2i>>>& discrete_scans, 
                                    const std::vector<SearchParameters>& search_parameters,
                                    const std::vector<Candidate>& candidates,
                                    int candidate_depth, float min_score)
{
    if(candidate_depth == 0) {
        return *candidates.begin();
    }
    
    Candidate best_candidate(0, 0, 0, search_parameters[0]);
    best_candidate.score = min_score;
    
    for(const Candidate& candidate : candidates) {
        if(candidate.score <= min_score) {
            break;
        }
        
        std::vector<Candidate> higher_resolution_candidates;
        for(int x : {0, 1}) {
            for(int y : {0, 1}) {
                higher_resolution_candidates.emplace_back(candidate.scan_index, 
                                                          2 * candidate.x_offset + x,
                                                          2 * candidate.y_offset + y,
                                                          search_parameters[candidate_depth-1]);
            }
        }
        
        ScoreCandidates(maps[candidate_depth-1], discrete_scans[candidate_depth-1], search_parameters[candidate_depth-1], higher_resolution_candidates);
        
        best_candidate = std::max(best_candidate, BranchAndBound(maps, discrete_scans, search_parameters, higher_resolution_candidates, candidate_depth-1, best_candidate.score));
    }
    
    return best_candidate;
    
}
