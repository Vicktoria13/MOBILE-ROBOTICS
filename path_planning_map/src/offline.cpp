#include <opencv4/opencv2/highgui.hpp>
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/PoseArray.h>


#include "path_planning_map/Divide.hpp"
#include "path_planning_map/Dijsktra.hpp"

#include "path_planning_map/occupancy_grid_loader.hpp"


using namespace std;


cv::Mat from_pgm_to_binary(std::string path){
  
    //load the image in grayscale mode
    cv::Mat image = cv::imread(path, cv::IMREAD_GRAYSCALE);
    ROS_INFO("Image size: %d x %d", image.rows, image.cols);

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


    // YAML
    std::string param_file_path ;

    
    
    // Récupérer le paramètre spécifié lors du lancement du nœud
    nh.getParam("param_file", param_file_path);
    
    if (param_file_path.empty()) {
        ROS_ERROR("Aucun fichier de paramètres spécifié.");
        return -1;
    }

    

   

    YAML::Node config = YAML::LoadFile(param_file_path);
    //on recupere les parametres du yaml
    int pas_divide = config["pas"].as<int>();
    int marge = config["marge"].as<int>();
    std::string folder_where_to_save_im = config["folder_image_saver"].as<std::string>();
    std::string path_pgm_map = config["map_pgm_path"].as<std::string>();
    std::string path_yaml_map = config["map_yaml_path"].as<std::string>();

    ROS_INFO("PATH_MAP: %s", path_pgm_map.c_str());

    std::string extension_path = path_pgm_map;
    std::string extension = extension_path.substr(extension_path.size()-3,extension_path.size());

    cv::Mat image;

    if (extension == "pgm"){
        ROS_INFO("PGM");
        image = from_pgm_to_binary(path_pgm_map);
    }
    else{
        ROS_INFO("PNG");
        ROS_ERROR("Extension non supportée");
    }

    // Vérifier si l'image est chargée avec succès
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
    //Dijsktra::Dijsktra(std::vector<Subcell*>* only_free_node_from_grid,int nb_free_cells){
    std::vector<Subcell*> tableau_pointurs_free_nodes = div.get_subcells_free();

    int nb_free_nodes = div.get_nb_free_nodes();
    Dijsktra dij = Dijsktra(&tableau_pointurs_free_nodes, nb_free_nodes);

    //on recupere les coordonnees de depart et d'arrivee
    //Subcell* start = div.get_one_subcell_free_with_index(3);
    //Subcell* end = div.get_one_subcell_free_with_index(5);
    

    //on recupere le nombre de free nodes
    
    std::vector<int> path = dij.launch_dijsktra(3,nb_free_nodes-150);


    ROS_INFO("Enregistrement du trace chemin dans un fichier PNG ...");
    div.display_subcell_state(path, folder_where_to_save_im);
    ROS_INFO("===================> SAVING DONE  ");

    // on charge les parametres de path_yaml_map
    double resolution;
    double* origin = new double[3];
    



    

    std::vector<std::vector<float>> coordonnees_consignes = std::vector<std::vector<float>>();
    
    // sachant que les coordonnées renvoyée par get_x et get_y sont pixel, le (0,0) est en haut a gauche
    //alors que le centre de la map est donné par la pos du robot
    // et on sait ou est le pixel en bas a gauche de la map (-100,-100) dans le referentiel de la map

    //donc on en deduit les coordonnées du chemin dans le repère map
    
    /*
    float x_bas_gauche = -100 +0 + sub_bas_gauche->get_x()*0.05;
    float y_bas_gauche = -100 + pixel_hauteur*0.05 - sub_bas_gauche->get_y()*0.05;

    */

    geometry_msgs::PoseArray trajectory;

   
    for (Subcell* ptr_sub : dij.get_sub_path()){
        float x = -100 + 0 + (ptr_sub->get_x()*0.05);
        float y = -100 + div.get_rows()*0.05 - (ptr_sub->get_y()*0.05);
        


        // on remplit trajectory
        geometry_msgs::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = 0.0;

        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 0.0;

        trajectory.poses.push_back(pose);
        
    }

    

    trajectory.header.frame_id = "map";
    trajectory.header.stamp = ros::Time::now();

    ROS_INFO("Nombre de points dans la trajectoire: %d", trajectory.poses.size());





    /************************** PUBLICATION ***************************/

    ros::Publisher trajectory_pub = nh.advertise<geometry_msgs::PoseArray>("/trajectory", 1);

    ros::Rate rate(1);  // Fréquence de publication (1 Hz, par exemple)

    ROS_INFO("PUBLICATION DE LA TRAJECTOIRE ...");
    while (ros::ok()) {
        // Afficher l'image

        trajectory_pub.publish(trajectory);
        ros::spinOnce();
        rate.sleep();

        
    }

    return 0;
}
