#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_server/image_loader.h>
#include <iostream>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>


#define PATH_YAML "/home/spi-2019/robmob_ws/src/minilab_simulation/map/buvette.yaml"
#define PATH_MAP "/home/spi-2019/robmob_ws/src/minilab_simulation/map/buvette.pgm"

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





    






int main(int argc, char** argv) {
    ros::init(argc, argv, "map_publisher");
    ros::NodeHandle nh;


    /** PARAM */
    double resolution;
    double* origin = new double[3];
    double free_thresh;
    double occupied_thresh;


    read_yaml_param(PATH_YAML, &resolution, origin, &free_thresh, &occupied_thresh);
    

    // Charger l'image PGM via path et map_server
    nav_msgs::GetMap::Response map_msg;

    map_server::loadMapFromFile(&map_msg, PATH_MAP, resolution, false, free_thresh, occupied_thresh, origin);

    // Créer un publisher pour publier la carte
    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    ros::Rate rate(1);  // Fréquence de publication (1 Hz, par exemple)

    while (ros::ok()) {
        map_pub.publish(map_msg.map);
        rate.sleep();
    }

    



    return 0;
}
