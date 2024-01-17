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
    /**
     * @brief Lit le fichier .yaml representant une occupancy grid 
     * @param param_file_path Chemin vers le fichier .yaml
     * @param resolution Pointeur vers la variable qui va contenir la resolution de la map
     * @param origin Pointeur vers le tableau qui va contenir l'origine de la map
     * @param free_thresh Pointeur vers la variable qui va contenir la valeur de free_thresh
     * @param occupied_thresh Pointeur vers la variable qui va contenir la valeur de occupied_thresh
     * @return void
     */

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