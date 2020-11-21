#include "scan_match.h"

#include <ros/ros.h>
#include <algorithm>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

//some ros massage for easy implementation
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>

ScanMatch* pcsm;
nav_msgs::OccupancyGrid map;

const double linear_search_window = 1.0;
const double angular_search_window = 0.5;
const double resolution = 0.05; //m
const double width = 30;  //m
const double height = 30; //m

PointCloud point_cloud_1;
PointCloud point_cloud_2;


void InitMap()
{
    int num_x_cells = ceil(width / resolution);
    map.info.width = num_x_cells;
    int num_y_cells = ceil(height / resolution);
    map.info.height = num_y_cells;
    map.info.resolution = resolution;
    map.info.origin.position.x = -width / 2.0;
    map.info.origin.position.y = -height / 2.0;
    map.data.resize(num_x_cells * num_y_cells);

    for(int i=0; i < map.data.size(); ++i) {
        map.data[i] = 49;
    }
}

void AddPointToMap(const Eigen::Vector3d& point)
{
    int cx = ceil((point.x() - map.info.origin.position.x) / map.info.resolution);
    int cy = ceil((point.y() - map.info.origin.position.y) / map.info.resolution);
    int id = cx + cy*map.info.width;
    map.data[id] = 100;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scan_match_node_test");
    ros::NodeHandle nh;
    
    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("scan_map", 1000);
    
    InitMap();
    
    ros::Rate loop_rate(10);
    
    while(ros::ok()) {
        
        for(double x = -0.95; x < 1; x += 0.05) {
            Eigen::Vector3d point(x, 1, 0);
            AddPointToMap(point);
            point_cloud_1.push_back(point);
        }
        
        for(double y = 0; y <= 1; y += 0.05) {
            Eigen::Vector3d point(-1, y, 0);
            AddPointToMap(point);
            point_cloud_1.push_back(point);
        }
        
        for(double y = 0; y <= 1; y += 0.05) {
            Eigen::Vector3d point(1, y, 0);
            AddPointToMap(point);
            point_cloud_1.push_back(point);
        }
        
        for(double x = -0.95; x < 1; x += 0.05) {
            Eigen::Vector3d point(x, 0.5, 0);
            point_cloud_2.push_back(point);
        }
        
        for(double y = 0; y <= 0.5; y += 0.05) {
            Eigen::Vector3d point(-1, y, 0);
            point_cloud_2.push_back(point);
        }
        
        for(double y = 0; y <= 0.5; y += 0.05) {
            Eigen::Vector3d point(1, y, 0);
            point_cloud_2.push_back(point);
        }
        
        pcsm = new ScanMatch(linear_search_window, angular_search_window);
        
        pcsm->updateMap(map);
        
        geometry_msgs::Pose2D pose_1;
        pose_1.x = 0;
        pose_1.y = 0;
        pose_1.theta = 0;
        
        //geometry_msgs::Pose2D pose_2 = pcsm->MatchBruteForce(pose_1, point_cloud_2);
        Eigen::AngleAxisd init_r_vector(pose_1.theta, Eigen::Vector3d(0, 0, 1));
        Eigen::Quaterniond initial_rotation(init_r_vector);
        
        PointCloud cloud = pcsm->RotatePointCloud(point_cloud_2, initial_rotation);
        
        SearchParameters search_parameters(linear_search_window, angular_search_window, point_cloud_2, map.info.resolution);
        
        std::vector<PointCloud> scans = pcsm->GenerateSans(cloud, search_parameters);
        
        std::vector<std::vector<Eigen::Array2i>> discrete_scans = pcsm->GenerateDiscreteScans(scans, pose_1);
        
        std::vector<Candidate> candidates = pcsm->GenerateCandidates(search_parameters);
        
        pcsm->ScoreCandidates(discrete_scans, search_parameters, &candidates);
        
        const Candidate& best_candidate = *std::max_element(candidates.begin(), candidates.end());
        
        nav_msgs::OccupancyGrid new_map = pcsm->getMap();

        map.header.frame_id = "map";

        map_pub.publish(new_map);
        
        
        ROS_INFO("the value is %2.2f", (float)best_candidate.orientation);

        ros::spinOnce();

        loop_rate.sleep();
    }
}
