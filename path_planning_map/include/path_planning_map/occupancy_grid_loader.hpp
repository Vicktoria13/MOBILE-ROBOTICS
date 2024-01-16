#ifndef OCCUPANCY_GRID_LOADER_HPP
#define OCCUPANCY_GRID_LOADER_HPP

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_server/image_loader.h>
#include <iostream>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>



void read_yaml_param(std::string param_file_path, double* resolution, double* origin, double* free_thresh, double* occupied_thresh) {
    // YAML
    YAML::Node config = YAML::LoadFile(param_file_path);
    
    //resolution
    *resolution = config["resolution"].as<double>();


    
    //origin
    origin[0] = config["origin"][0].as<double>();
    origin[1] = config["origin"][1].as<double>();
    origin[2] = config["origin"][2].as<double>();

    //free_thresh
    *free_thresh = config["free_thresh"].as<double>();

    //occupied_thresh
    *occupied_thresh = config["occupied_thresh"].as<double>();

}


#endif