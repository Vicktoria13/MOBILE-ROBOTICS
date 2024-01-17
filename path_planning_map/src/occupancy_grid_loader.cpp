#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_server/image_loader.h>
#include <iostream>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include "path_planning_map/occupancy_grid_loader.hpp"


/**
 * @brief Creation du noeud permettant de publier une occupancy grid (nav_msgs::OccupancyGrid) sur un topic
 * La carte doit etre complete, pour ensuite faire l'exploration dedans
 * 
 */

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_publisher");
    ros::NodeHandle nh("~");

    // On recupere le YAML
    std::string param_file_path;
    nh.getParam("param_file", param_file_path);
    if (param_file_path.empty()) {
        ROS_ERROR("Aucun fichier de paramètres spécifié.");
        return -1;
    }



    YAML::Node config = YAML::LoadFile(param_file_path);

    std::string path_yaml_map = config["map_yaml_path"].as<std::string>();
    std::string path_pgm_map = config["map_pgm_path"].as<std::string>();

    ROS_INFO("YAML MAP PATH: %s", path_yaml_map.c_str());
    ROS_INFO("PGM MAP PATH: %s", path_pgm_map.c_str());


    /** PARAM */
    double resolution;
    double* origin = new double[3];
    double free_thresh;
    double occupied_thresh;


    read_yaml_param(path_yaml_map, &resolution, origin, &free_thresh, &occupied_thresh);
  
    nav_msgs::GetMap::Response map_msg;

    char* path = new char[path_pgm_map.length() + 1];
    strcpy(path, path_pgm_map.c_str());

    // on normalise les valeurs entre 0 et 1

    map_server::loadMapFromFile(&map_msg, path, resolution, false, free_thresh, occupied_thresh, origin, MapMode::TRINARY);

    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    ros::Rate rate(1);  // Fréquence de publication (1 Hz, par exemple)

    while (ros::ok()) {
        map_pub.publish(map_msg.map);
        rate.sleep();
    }

    



    return 0;
}
