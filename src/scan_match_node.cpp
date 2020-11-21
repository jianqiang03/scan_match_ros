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

#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>

ScanMatch* pcsm;
nav_msgs::OccupancyGrid map;

std::string base_link_name;     // Base link name (frame)
std::string lidar_link_name;    // Lidar link name (frame)
nav_msgs::Path path;            // the path (trajectory) of the robot
geometry_msgs::Pose2D pose;     // the current pose (according to the current LiDAR scan and current map)
geometry_msgs::Pose2D old_pose; // the last pose

int cpt_no_change_val; // number of iteration where the pose does not change
int watchdog;          // maximal number of iterations
int map_publishing_rate;
int path_publishing_rate;

bool map_started = false;           // flag to identify the first iteration
bool stop_mapping = false;          // flag to start/stop the mapping process
bool publish_lidar_pose_tf = true;  // to enable/disable the publishing of the tf to display LiDAR

double linear_search_window = 1.0;
double angular_search_window = 0.5;
double resolution = 0.05; //m
double width = 150;  //m
double height = 150; //m

int max_belief = 100;
int min_belief = 0;
int add_belief = 25;
int rem_belief = 10;
int threshold_belief = 50;

int nb_iterations; 

ros::Publisher path_pub;
ros::Publisher map_pub;

void InitMap(double width, double height, double resolution)
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

void InitParameters(ros::NodeHandle& node)
{
    if(node.getParam("cpt_no_change_val", cpt_no_change_val)){
        ROS_INFO("scan_match_node::cpt_no_change_val parameter (nb iterations): %d", cpt_no_change_val);
    }else{
        cpt_no_change_val = 10;
        ROS_WARN("scan_match_node::Could not get the cpt_no_change_val parameter, default value (nb iterations): %d", cpt_no_change_val);
    }
    // maximal number of iterations
    if(node.getParam("watchdog", watchdog)){
        ROS_INFO("scan_match_node::watchdog parameter (nb iterations): %d", watchdog);
    }else{
        watchdog = 40;
        ROS_WARN("scan_match_node::Could not get the watchdog parameter, default value (nb iterations): %d", watchdog);
    }
    // number of iteration (processed LiDAR data) before publishing the probability map
    map_publishing_rate = 100;
    if(node.getParam("map_publishing_rate", map_publishing_rate)){
        ROS_INFO("scan_match_node::map_publishing_rate parameter (nb iteration): %d", map_publishing_rate);
    }else{
        ROS_WARN("scan_match_node::Could not get the map_publishing_rate parameter, default value (nb iteration): %d", map_publishing_rate);
    }
    // number of iteration (processed LiDAR data) before publishing the path
    path_publishing_rate = 10;
    if(node.getParam("path_publishing_rate", path_publishing_rate)){
        ROS_INFO("scan_match_node::path_publishing_rate parameter (nb iteration): %d", path_publishing_rate);
    }else{
        ROS_WARN("scan_match_node::Could not get the path_publishing_rate parameter, default value (nb iteration): %d", path_publishing_rate);
    }
    // to enable/disable the publishing of the tf to display the LiDAR data
    publish_lidar_pose_tf = true;
    if(node.getParam("publish_lidar_pose_tf", publish_lidar_pose_tf)){
        ROS_INFO("scan_match_node::publish_lidar_pose_tf parameter (boolean): %d", publish_lidar_pose_tf);
    }else{
        ROS_WARN("scan_match_node::Could not get the publish_lidar_pose_tf parameter, default value (boolean): %d", publish_lidar_pose_tf);
    }
    // Base Link name (frame)
    base_link_name = "base_link_default";
    if(node.getParam("base_link_name", base_link_name)){
        ROS_INFO("scan_match_node::base link name: %s", base_link_name.c_str());
    }else{
        ROS_WARN("scan_match_node::Could not get the base link name parameter, default value: %s", base_link_name.c_str());
    }
    // LiDAR Link name (frame)
    lidar_link_name = "laser_default";
    if(node.getParam("lidar_link_name", lidar_link_name)){
        ROS_INFO("scan_match_node::lidar link name: %s", lidar_link_name.c_str());
    }else{
        ROS_WARN("scan_match_node::Could not get the lidar link name parameter, default value: %s", lidar_link_name.c_str());
    }
    // to publish the path of the robot
    std::string path_topic_name = "/scan_match/path";
    if(node.getParam("path_topic_name", path_topic_name)){
        ROS_INFO("scan_match_node::path topic name: %s", path_topic_name.c_str());
    }else{
        ROS_WARN("scan_match_node::Could not get the path topic name parameter, default value: %s", path_topic_name.c_str());
    }

    // the topic name to publish the probability map
    std::string probmap_topic_name = "/scan_match/probmap";
    if(node.getParam("probmap_topic_name", probmap_topic_name)){
        ROS_INFO("scan_match_node::probmap topic name: %s", probmap_topic_name.c_str());
    }else{
        ROS_WARN("scan_match_node::Could not get the probmap topic name parameter, default value: %s", probmap_topic_name.c_str());
    }

    // Map cell resolution (meters)
    if(node.getParam("resolution", resolution)){
        ROS_INFO("scan_match_node::resolution parameter (meters): %2.3f", resolution);
    }else{
        resolution = 0.05;
        ROS_WARN("scan_match_node::Could not get the resolution parameter, default value (meters): %2.3f", resolution);
    }
    // Map maximal height (meters)
    if(node.getParam("height", height)){
        ROS_INFO("scan_match_node::height parameter (meters): %2.2f", height);
    }else{
        height = 150;
        ROS_WARN("scan_match_node::Could not get the height parameter, default value (meters): %2.2f", height);
    }
    // Map maximal width (meters)
    if(node.getParam("width", width)){
        ROS_INFO("scan_match_node::width parameter (meters): %2.2f", width);
    }else{
        width = 150;
        ROS_WARN("scan_match_node::Could not get the width parameter, default value (meters): %2.2f", width);
    }
    
    //initial map
    InitMap(width, height, resolution);
    
    //Linear Search Window
    if(node.getParam("linear_search_window", linear_search_window)){
        ROS_INFO("scan_match_node::linear_search_window parameter (meters): %2.2f", linear_search_window);
    }else{
        linear_search_window = 1.0;
        ROS_WARN("scan_match_node::Could not get the linear_search_window parameter, default value (meters): %2.2f", linear_search_window);
    }
    
    //Angular Search Window
    if(node.getParam("angular_search_window", angular_search_window)){
        ROS_INFO("scan_match_node::angular_search_window (radianss): %2.2f", angular_search_window);
    }else{
        angular_search_window = 0.5;
        ROS_WARN("scan_match_node::Could not get the angular_search_window parameter, default value (radians): %2.2f", angular_search_window);
    }
    
    //intial scan matching
    pcsm = new ScanMatch(linear_search_window, angular_search_window);
    
    // maximal value of a cell in the probability map
    if(node.getParam("max_belief", max_belief)){
        ROS_INFO("scan_match_node::max_belief parameter (cost): %d", max_belief);
    }else{
        max_belief = 100;
        ROS_WARN("scan_match_node::Could not get the max_belief parameter, default value (cost): %d", max_belief);
    }
    
    // minimal value of a cell in the probability map
    if(node.getParam("min_belief", min_belief)){
        ROS_INFO("scan_match_node::min_belief parameter (cost): %d", min_belief);
    }else{
        min_belief = 0;
        ROS_WARN("scan_match_node::Could not get the min_belief parameter, default value (cost): %d", min_belief);
    }
    // value to add in a cell when adding an obstacle
    if(node.getParam("add_belief", add_belief)){
        ROS_INFO("scan_match_node::add_belief parameter (cost): %d", add_belief);
    }else{
        add_belief = 25;
        ROS_WARN("scan_match_node::Could not get the add_belief parameter, default value (cost): %d", add_belief);
    }
    // value to remove in a cell when removing an obstacle
    if(node.getParam("rem_belief", rem_belief)){
        ROS_INFO("scan_match_node::rem_belief parameter (cost): %d", rem_belief);
    }else{
        rem_belief = 10;
        ROS_WARN("scan_match_node::Could not get the rem_belief parameter, default value (cost): %d", rem_belief);
    }
    // Limit value for a cell, under this threshold the cell is free, above, the cell is an obstacle
    if(node.getParam("threshold_belief", threshold_belief)){
        ROS_INFO("scan_match_node::threshold_belief parameter (cost): %d", threshold_belief);
    }else{
        threshold_belief = 50;
        ROS_WARN("scan_match_node::Could not get the threshold_belief parameter, default value (cost): %d", threshold_belief);
    }
    // Link name of the map (frame)
    std::string map_link_name = base_link_name;
    if(node.getParam("map_link_name", map_link_name)){
        ROS_INFO("pa_slam_node::map link name: %s", map_link_name.c_str());
    }else{
        ROS_WARN("pa_slam_node::Could not get the map link name parameter, default value: %s", map_link_name.c_str());
    }
    map.header.frame_id = map_link_name;

    // init the map started flag
    map_started = false;
    
    //init path publisher
    path_pub = node.advertise<nav_msgs::Path>(path_topic_name, 1000);
    
    //init map publisher
    map_pub = node.advertise<nav_msgs::OccupancyGrid>(probmap_topic_name, 1000);
}

// to transform a pitch/roll/yaw value to a quaternion
geometry_msgs::Quaternion toQuaternion(double pitch, double roll, double yaw){
    geometry_msgs::Quaternion q;
    double t0 = std::cos(yaw * 0.5);
    double t1 = std::sin(yaw * 0.5);
    double t2 = std::cos(roll * 0.5);
    double t3 = std::sin(roll * 0.5);
    double t4 = std::cos(pitch * 0.5);
    double t5 = std::sin(pitch * 0.5);

    q.w = t0 * t2 * t4 + t1 * t3 * t5;
    q.x=(t0 * t3 * t4 - t1 * t2 * t5);
    q.y=(t0 * t2 * t5 + t1 * t3 * t4);
    q.z=(t1 * t2 * t4 - t0 * t3 * t5);
    return q;
}

// To save the path of the robot in order to be able to display it
void updatepath(){
    path.header.frame_id = base_link_name.c_str();
    // the 2D pose needs to be converted into a posestamped
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = base_link_name.c_str();
    pose_stamped.pose.position.x = pose.x;
    pose_stamped.pose.position.y = pose.y;
    pose_stamped.pose.orientation = toQuaternion(0.0f,0.0f,pose.theta);
    // adding the new pose to the path
    path.poses.push_back(pose_stamped);
}

// To publish the LiDAR transform based on the pose computed
void publisLiDARPoseTf(){
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(pose.x, pose.y, 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, pose.theta);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), base_link_name.c_str(), lidar_link_name.c_str()));
}

// function to get the value of a given cell cx, cy
int GetProbability(int cx, int cy){
    // we first check if the cell is inside the map and then return the value
    if(cx >= 0 && cx < map.info.width && cy >= 0 && cy < map.info.height)
        return map.data[cx+cy*map.info.width];
    else{
        // -1 is returned otherwise
        ROS_ERROR("prob_val_at() - %d %d is not a correct cell", cx, cy);
        return -1;
    }
}

// function to add a value to a given cell
void AddProbabilityValue(int cx, int cy, int val){
    // first we check if the cells belongs to the map
    if(cx >= 0 && cx < map.info.width && cy >= 0 && cy < map.info.height){
        // we compute the index of the cell according the cell coordinates
        int id = cx+cy*map.info.width;
        // the cost is increased beyound a max value
        if( map.data[id] + val <= max_belief) map.data[id] += val;
        else map.data[id] = max_belief;
    }// if the cell does not belong to the map, nothing to do but warn the user
    else ROS_ERROR("prob_add_val_at() - %d (%d) %d (%d) is not a correct cell", cx, map.info.width, cy, map.info.height);
}

// function to add a value to a given cell
void RemoveProbabilityValue(int cx, int cy, int val){
    // first we check if the cells belongs to the map
    if(cx >= 0 && cx < map.info.width && cy >= 0 && cy < map.info.height){
        // we compute the index of the cell according the cell coordinates
        int id = cx+cy*map.info.width;
        // the cost is decreased over a min value
        if( map.data[id] > val) map.data[id] -= val;
        else map.data[id] = 0;
    }// if the cell does not belong to the map, nothing to do but warn the user
    else ROS_ERROR("prob_rem_val_at() - %d %d is not a correct cell", cx, cy);
}

void FreePath(int x1, int y1, int x2, int y2){

    bool steep = abs(y2-y1) > abs(x2-x1);

    if(steep) {
        std::swap<int>(x1,y1);
        std::swap<int>(x2,y2);
    }

    if(x1 > x2) {
        std::swap<int>(x1,x2);
        std::swap<int>(y1,y2);
    }

    int dx = x2 - x1;
    int dy = abs(y2 - y1);

    double error = dx/2;
    const int ystep = (y1 < y2) ? 1 : -1;

    int y = y1;
    const int max_x = x2;
    int x_tmp=0, y_tmp=0;

    for(int x = x1 ; x < max_x; x++){
        if(steep){
            x_tmp = y;
            y_tmp = x;
        }
        else{
            x_tmp = x;
            y_tmp = y;
        }

        RemoveProbabilityValue(x_tmp, y_tmp, rem_belief);

        error -= dy;
        if(error < 0){
            y += ystep;
            error += dx;
        }
    }
}

int get_x_cell_from_world(double x){
    int cx = (int)((x - map.info.origin.position.x)/map.info.resolution);
    return cx;
}

// this function convert the y world value into y cell index
int get_y_cell_from_world(double y){
   int cy = (int)((y-map.info.origin.position.y)/map.info.resolution);
   return cy;
}


// to update the map when an obstacle is detected (inputs are cells coordinates)
void AddObstacle(int x_cell_robot, int y_cell_robot, int x_cell_obs, int y_cell_obs){
    // first we check if the cells belongs to the map
    if(x_cell_obs >= 0 && x_cell_obs <map.info.width && y_cell_obs >= 0 && y_cell_obs < map.info.height){
        // we add the obstacle in the probability map
        AddProbabilityValue(x_cell_obs, y_cell_obs, add_belief);
        // from the robot to this obstacle, the belives of a free path is updated
        FreePath(x_cell_robot, y_cell_robot, x_cell_obs, y_cell_obs);
    } // if not, nothing to but but warn the user
    else ROS_ERROR("obstacle_detected() - (%d %d) or (%d %d) is not a correct cell", x_cell_robot, y_cell_robot, x_cell_obs, y_cell_obs);
}

// to update the map when an obstacle is detected (inputs are world coordinates in m)
void AddObstacle(double x_robot, double y_robot, double x_obs, double y_obs){
    // first we convert the world coordinates into cell indexes
    int x_cell_r = get_x_cell_from_world(x_robot);
    int y_cell_r = get_y_cell_from_world(y_robot);
    int x_cell_o = get_x_cell_from_world(x_obs);
    int y_cell_o = get_y_cell_from_world(y_obs);

    // then we check if the obstacle and the robot belongs to the map
    if(x_cell_r >= 0 && y_cell_r >= 0 && x_cell_o >= 0 && y_cell_o >= 0 &&
       x_cell_r < map.info.width && y_cell_r < map.info.height && x_cell_o < map.info.width && y_cell_o  < map.info.height){
        // and we use the obstacle_detected function that needs cell coordinates
        AddObstacle(x_cell_r, y_cell_r, x_cell_o, y_cell_o);
    } // if not, nothing to but but warn the user
    else ROS_ERROR("obstacle_detected() - (%d %d) or (%d %d) is not a correct cell", x_cell_r, y_cell_r, x_cell_o, y_cell_o);
}

void AddScanToMap(const sensor_msgs::LaserScan &scan, const geometry_msgs::Pose2D& pose)
{
    unsigned int i, size = scan.ranges.size();
    double angle = scan.angle_min;
    double obs_x, obs_y;
    std::vector<float>::const_iterator it_msr; //using an iterator for faster loops
    // loop over the measurements
    for(it_msr=scan.ranges.begin(); it_msr!=scan.ranges.end(); ++it_msr){
        if(*it_msr == 0 || *it_msr >= INFINITY || *it_msr < 0.2f || *it_msr >= 100.0f){
            continue;
        }
        else{
            // convert the scan into x y coordinates (according to the pose)
            obs_x = (*it_msr) * cos(angle + pose.theta) + pose.x;
            obs_y = (*it_msr) * sin(angle + pose.theta) + pose.y;

            // add the detected obstacle to the map
            // the pose coordinates are also needed to "free" the line between the pose and the obstacle
            AddObstacle(pose.x, pose.y, obs_x, obs_y);
        }
        // needed to convert a scan into x,y coordinates
        angle += scan.angle_increment;
    }    
}

void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    if(!map_started) {
        ROS_INFO("start mapping ...");
        pose.x = 0;
        pose.y = 0;
        pose.theta = 0;
        AddScanToMap(*msg, pose);
        map_started = true;
        nb_iterations = 0;
        map_pub.publish(map);
        if(publish_lidar_pose_tf) publisLiDARPoseTf();
    }
    else if (!stop_mapping) {
        int watch_dog = 0;
        do{
            int cpt_no_change = 0; 
            while(cpt_no_change < cpt_no_change_val){
                old_pose = pose;
                pcsm->updateMap(map);
                pcsm->MatchBruteForce(old_pose, *msg, pose);
                if(old_pose.x == pose.x && old_pose.y == pose.y && old_pose.theta == pose.theta){
                    cpt_no_change++;
                }else{
                    cpt_no_change=0;
                }
            }
            watch_dog++; // update the watch dog not to loop too long
        }while(watch_dog < watchdog);

        // if we did not break the loop because of the watchdog
        if(watch_dog < watchdog){
            // we add the new scan to the map
            AddScanToMap(*msg, pose);
            // we update the path
            updatepath();
            // if enabled, the corresponding tf is published
            if(publish_lidar_pose_tf) publisLiDARPoseTf();
        }else{
            // we break the loop because of the watch dog, meaning that the pose/lidar cost is to high...
            // we do not add the LiDAR scan, we stop the map building
            stop_mapping = true;
        }
        nb_iterations ++;
        if(nb_iterations%map_publishing_rate == 0){
            map_pub.publish(map);
        }
        if(nb_iterations%path_publishing_rate == 0){
            path_pub.publish(path);
        }
    }
}


int main(int argc, char** argv) {

    ros::init(argc, argv, "scan_match_node");
    ros::NodeHandle nh;
    
    InitParameters(nh);
    
    std::string scan_topic_name = "/scan";
    ros::Subscriber sub_lidar = nh.subscribe(scan_topic_name.c_str(), 1000, ScanCallback);
/**    
    while(ros::ok()) {

        pcsm = new ScanMatch(linear_search_window, angular_search_window, resolution, num_x_cells, num_y_cells);

        nav_msgs::OccupancyGrid map = pcsm->getMap();

        map.header.frame_id = "map";

        map_pub.publish(map);
        
        geometry_msgs::PoseStamped map_origin;
        map_origin.header.frame_id = "map";
        map_origin.pose = map.info.origin;
        
        pose_pub.publish(map_origin);
        
        ROS_INFO("The map origin is %2.2f", map_origin.pose.position.x);

        ros::spinOnce();

        loop_rate.sleep();
    }
**/
    return 0;
}
