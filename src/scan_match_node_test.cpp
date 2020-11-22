#include "scan_match.h"

#include <ros/ros.h>
#include <algorithm>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

ScanMatch* pcsm;
nav_msgs::OccupancyGrid map;

const double linear_search_window = 1.0;
const double angular_search_window = 0.5;
const double resolution = 0.05; //m
const double width = 30;  //m
const double height = 30; //m
const int branch_and_bound_depth = 4;

PointCloud point_cloud_1;
PointCloud point_cloud_2;
pcl::PointCloud<pcl::PointXYZ> cloud;


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

void GenerateSamplePointCloud()
{
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
    
    cloud.width = point_cloud_2.size();
    cloud.height = 1;
    cloud.points.reserve(cloud.width * cloud.height);
    
    for(const Eigen::Vector3d& point : point_cloud_2) {
        cloud.points.push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scan_match_node_test");
    ros::NodeHandle nh;
    
    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("scan_map", 1000);
    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud", 1000);
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 1000);
    
    InitMap();
    
    GenerateSamplePointCloud();
    
    ros::Rate loop_rate(10);
    
    while(ros::ok()) {
        
        pcsm = new ScanMatch(linear_search_window, angular_search_window, branch_and_bound_depth);
        
        geometry_msgs::Pose2D pose_1;
        pose_1.x = 0;
        pose_1.y = 0;
        pose_1.theta = 0;
        
        geometry_msgs::Pose2D pose_2 = pcsm->MatchBruteForce(map, pose_1, point_cloud_2);
        
        Eigen::Affine3d transform = Eigen::Affine3d::Identity();
        transform.translation() << pose_2.x, pose_2.y, 0.0;
        transform.rotate(Eigen::AngleAxisd(pose_2.theta, Eigen::Vector3d::UnitZ()));
        
        
        pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
        pcl::transformPointCloud(cloud, transformed_cloud, transform.matrix());
        
        pcl::PCLPointCloud2 PCL_cloud;
        pcl::toPCLPointCloud2(transformed_cloud, PCL_cloud);
        sensor_msgs::PointCloud2 pub_points;
        pcl_conversions::moveFromPCL(PCL_cloud, pub_points);
        pub_points.header.frame_id = "map";
        
        geometry_msgs::PoseStamped pub_pose;
        pub_pose.header.frame_id = "map";
        pub_pose.pose.position.x = pose_2.x;
        pub_pose.pose.position.y = pose_2.y;
        pub_pose.pose.position.z = 0;
        
        //geometry_msgs::Pose2D pose_3 = pcsm->MatchMultiResolution(map, pose_1, point_cloud_2, 0.1);

        map.header.frame_id = "map";

        map_pub.publish(map);
        cloud_pub.publish(pub_points);
        pose_pub.publish(pub_pose);
        
        //std::vector<nav_msgs::OccupancyGrid> maps = pcsm->GenerateLookUpTables(map);
        //nav_msgs::OccupancyGrid new_map = pcsm->CompressMap(map, 4);
        
        //ROS_INFO("pose 3 is %2.1f", (float)pose_);
        
       ROS_INFO("The estimated pose is (x: %2.1f y: %2.1f theta: %2.1f)", (float)pose_2.x, (float)pose_2.y, (float)pose_2.theta);

        ros::spinOnce();

        loop_rate.sleep();
    }
}
