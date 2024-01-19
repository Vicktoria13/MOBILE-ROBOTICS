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
    // Initialisation du nœud ROS
    ros::init(argc, argv, "PathPlanning");
    ros::NodeHandle nh("~");


    /**************************** YAML ******************************/
    std::string param_file_path ;
    
    // Récupérer le paramètre spécifié lors du lancement du nœud
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

    cv::Mat image;

    if (extension == "pgm"){
        ROS_INFO("PGM");
        image = from_pgm_to_binary(path_pgm_map);
    }
    else{
        ROS_ERROR("Extension non supportée");
    }

    if (image.empty()) {
        ROS_ERROR("Impossible de charger l'image.");
        return -1;
    }

    

    Divide div = Divide(image,pas_divide);

    // Necessaire si jamais valeurs incohérentes en entrée / systeme différent
    //div.get_rid_inconsitencies();

    int nb_celles = div.divide_map(marge);

    ROS_INFO("============= INFO =======================");
    ROS_INFO("Nombre lignes: %d", div.get_rows());
    ROS_INFO("Nombre colonnes: %d", div.get_cols());
    ROS_INFO("Nombre de subcells: %d", nb_celles);
    ROS_INFO("==========================================");

    //permet de creer this->subcells_free : a faire apres avoir init + ajouté les voisins adjacents
    div.build_graph_free_subcells();
    
    /* =================================== FOR DEBUG  ===================================*/
    div.display_subcells(folder_where_to_save_im);
    /* =================================== FOR DEBUG  ===================================*/

    //////////////// DIJKSTRA ///////////////////////
    
    std::vector<Subcell*> tableau_pointurs_free_nodes = div.get_subcells_free();
    int nb_free_nodes = div.get_nb_free_nodes();
    Dijsktra dij = Dijsktra(&tableau_pointurs_free_nodes, nb_free_nodes);

    // faire une fonction qui convertir une consigne en mètres en une consigne en subcells (verifier que c une subcell libre)
    std::vector<int> path = dij.launch_dijsktra(3,nb_free_nodes-1);

    //////////////// DIJKSTRA ///////////////////////

    ROS_INFO("Enregistrement du trace chemin dans un fichier PNG ...");
    div.display_subcell_state(path, folder_where_to_save_im);
    ROS_INFO("===================> SAVING DONE  ");

    // on charge les parametres de path_yaml_map
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
    

    // Subcell bas_droite coordonness
    double coordonnes_subcell_haut_droite[2];
    Subcell *haut_droite = div.get_one_subcell_with_index(0,div.get_nb_cols_discrete()-1);

    coordonnes_subcell_haut_droite[0] = origin[0] + 0 + (haut_droite->get_x()*resolution);
    coordonnes_subcell_haut_droite[1] = origin[1] + div.get_rows()*resolution - (haut_droite->get_y()*resolution);

    ROS_INFO("coordonnes_subcell_haut_droite: %f, %f", coordonnes_subcell_haut_droite[0], coordonnes_subcell_haut_droite[1]);

    // test debug
    // subcell en bas a gauche
    int nb_lignes_subcells = div.get_nb_rows_discrete();
    int nb_colonnes_subcells = div.get_nb_cols_discrete();

    ROS_INFO("nb_lignes_subcells: %d", nb_lignes_subcells);
    ROS_INFO("nb_colonnes_subcells: %d", nb_colonnes_subcells);
    
    Subcell* bas_gauche = div.get_one_subcell_with_index(nb_lignes_subcells-1,0);

    float x_meters;
    float y_meters;

    x_meters = origin[0] + 0 + (bas_gauche->get_x()*resolution);
    y_meters = origin[1] + div.get_rows()*resolution - (bas_gauche->get_y()*resolution);

    ROS_INFO("aFFIChage de la subcell[%d][%d]", nb_lignes_subcells-1,0);
    ROS_INFO("x_meters: %f", x_meters);
    ROS_INFO("y_meters: %f", y_meters);

    // on fait l'inverse : étant donné des coordonnées en mètres, on veut les coordonnées en subcells
    int x_subcells;
    int y_subcells;

    // sachant que la subcell en bas a gauche a pour coordonnées (origin[0], origin[1])

    x_subcells = (x_meters*nb_lignes_subcells/ origin[0]) - 1;
    y_subcells = (x_meters*(nb_colonnes_subcells-1)) / 99.5;

    ROS_INFO("x_subcells: %d", x_subcells);
    ROS_INFO("y_subcells: %d", y_subcells);






    //message de type path
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = ros::Time::now();

    // on repmpli le message path
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

    //maintenant, on fait l'inerse : étant donné des coordonnées en mètres, on veut les coordonnées en subcells

  


    /************************** PUBLICATION ***************************/

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
