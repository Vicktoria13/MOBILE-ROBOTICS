#include <opencv4/opencv2/highgui.hpp>
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <yaml-cpp/yaml.h>
#include "nav_msgs/Path.h"
#include <tf/tf.h>

#include "path_planning_map/getUnicyclePos.h"
#include "path_planning_map/Divide.hpp"
#include "path_planning_map/Dijsktra.hpp"
#include "path_planning_map/occupancy_grid_loader.hpp"


using namespace std;


cv::Mat from_pgm_to_binary(std::string path){
  
    //load the image in grayscale mode
    cv::Mat image = cv::imread(path, cv::IMREAD_GRAYSCALE);
    cv::Mat image_binary = cv::Mat::zeros(image.rows, image.cols, CV_8UC1);

    for (int i = 0; i < image.rows; i++){
        for (int j = 0; j < image.cols; j++){

            if (image.at<uchar>(i,j) > 245){
                image_binary.at<uchar>(i,j) = 255;
            }
            else{
                image_binary.at<uchar>(i,j) = 0;
            }
        }
    }
    return image_binary;
}



int main(int argc, char** argv)
{
    /************************************* Initialisation du nœud ROS ******************************************/

    ros::init(argc, argv, "PathPlanning");
    ros::NodeHandle nh("~");



    /************************************* Lecture des paramères donnés en argument ****************************/

    /*********************   YAML contenant les chemins ************************/

    std::string param_file_path ;
    nh.getParam("param_file", param_file_path);
    
    if (param_file_path.empty()) {
        ROS_ERROR("Aucun fichier de paramètres spécifié.");
        return -1;
    }

    
    YAML::Node config = YAML::LoadFile(param_file_path);

    int pas_divide = config["pas"].as<int>();
    int marge = config["marge"].as<int>();
    std::string folder_where_to_save_im = config["folder_image_saver"].as<std::string>();
    std::string path_pgm_map = config["map_pgm_path"].as<std::string>();
    std::string path_yaml_map = config["map_yaml_path"].as<std::string>();


    std::string extension_path = path_pgm_map;
    std::string extension = extension_path.substr(extension_path.size()-3,extension_path.size());



    
    /*********************   YAML contenant les metadonnes de la map  ************************/

    double resolution;
    double* origin = new double[3];

    YAML::Node config_map = YAML::LoadFile(path_yaml_map);
    resolution = config_map["resolution"].as<double>();
    origin[0] = config_map["origin"][0].as<double>();
    origin[1] = config_map["origin"][1].as<double>();
    origin[2] = config_map["origin"][2].as<double>();

    ROS_INFO("================== INFO PARAMETRES MAP ========================");
    ROS_INFO("Resolution: %f", resolution);
    ROS_INFO("Origin: %f, %f, %f", origin[0], origin[1], origin[2]);
    ROS_INFO("================================================================");
    


    /************************************* Lecture de la MAP PGM ******************************************/

    cv::Mat image;

    if (extension == "pgm"){
        ROS_INFO("PGM");
        image = from_pgm_to_binary(path_pgm_map);
    }
    else{
        ROS_ERROR("Extension non supportée");
        return -1;
    }

    if (image.empty()) {
        ROS_ERROR("Impossible de charger l'image.");
        return -1;
    }

    

    /************************************* Discretisation MAP ******************************************/

    Divide div = Divide(image,pas_divide);
    int nb_celles = div.divide_map(marge);
    div.build_graph_free_subcells();
    
    /* =================================== FOR DEBUG  ===================================*/
    div.display_subcells(folder_where_to_save_im);
    /* =================================== FOR DEBUG  ===================================*/




    /************************************* Récupération de la position initiale du robot ******************************************/


    ros::ServiceClient client = nh.serviceClient<path_planning_map::getUnicyclePos>("/get_unicycle_pos");
    path_planning_map::getUnicyclePos srv;


    float x_metres, y_metres;
    if (client.call(srv)) {
        nav_msgs::Odometry unicycle_position = srv.response.unicycle_position;
        x_metres = unicycle_position.pose.pose.position.x;
        y_metres = unicycle_position.pose.pose.position.y;


    } else {
        ROS_ERROR("Impossible de récupérer la position du robot.");
        return -1;
    }

    /** Convertir en subcell **/
    int lignes_subcell;
    int colonnes_subcell;
    int a = div.convert_from_meters_to_free_subcells(x_metres,y_metres,&lignes_subcell,&colonnes_subcell,resolution,origin,true,false);
    
    if (a == -1){
        ROS_ERROR("SUR UN MUR");
        return -1;
    }


    Subcell* subcell_depart = div.get_one_subcell_with_index(lignes_subcell,colonnes_subcell);
    int id_start = subcell_depart->get_id();
    

    /************************************* DIJKSTRA******************************************/

    std::vector<Subcell*> tableau_pointurs_free_nodes = div.get_subcells_free();
    int nb_free_nodes = div.get_nb_free_nodes();
    Dijsktra dij = Dijsktra(&tableau_pointurs_free_nodes, nb_free_nodes);
    std::vector<int> path = dij.launch_dijsktra(id_start,nb_free_nodes-345);


    ROS_INFO("Enregistrement du trace chemin dans un fichier PNG ..");
    //div.display_subcell_state(path, folder_where_to_save_im);


    

   



    /************************************* Remplissage et consitution Du PATH  ******************************************/

    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = ros::Time::now();

    for (Subcell* ptr_sub : dij.get_sub_path()){
        geometry_msgs::PoseStamped pose;

        float x = origin[0] + 0 + (ptr_sub->get_x()*resolution);
        float y = origin[1] + div.get_rows()*resolution - (ptr_sub->get_y()*resolution);

        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;

        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 0.0;

        path_msg.poses.push_back(pose);
    }


  


    /************************************* Publication du PATH ******************************************/

    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/trajectory", 1);
    ros::Rate rate(1);  // Fréquence de publication (1 Hz, par exemple)



    ROS_INFO("PUBLICATION DE LA TRAJECTOIRE ...");

    while (ros::ok()) {
        path_pub.publish(path_msg);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
