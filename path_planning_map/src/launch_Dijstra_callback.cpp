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
#include <geometry_msgs/PoseArray.h>



#include "path_planning_map/getUnicyclePos.h"
#include "path_planning_map/Divide.hpp"
#include "path_planning_map/Dijsktra.hpp"
#include "path_planning_map/occupancy_grid_loader.hpp"


class PathPlanningNode {



private:

    // Noeud ROS + Publisher(publie la traj)  + Subscriber(recupere le point d'arrivée via Rviz) + Service (pour récupérer la position du robot)
    ros::NodeHandle nh_;
    ros::Subscriber move_goal_sub_;
    ros::Publisher path_pub_;
    ros::ServiceClient client;
    ros::Publisher traj_pub_;
    /***************************************************** Pour la discrétisation *****************************************************************************/
    Divide div;

    /***************************************************** Pour visu *****************************************************************************/
    std::string folder_where_to_save_im_;

    /***************************************************** Trajectoire *****************************************************************************/
    nav_msgs::Path path_msg_;


    /***************************************************** Paramètres de la map *****************************************************************************/
    cv::Mat image_binaire_from_pgm;
    float resolution_map;
    double* origin_map = new double[3];


    /***************************************************** Paramètres de la nav *****************************************************************************/
    int pas_;
    int marge_;

public:


    PathPlanningNode(ros::NodeHandle& nh, cv::Mat im, int pas_divide, int marge, std::string folder_where_to_save_im, float resolution, double* origin) : 
    nh_(nh), image_binaire_from_pgm(im){

        /**
         * @brief Descripteur du noeud PathPlanning
         * @param nh : noeud ROS
         * @param im : image binaire de la map (obstacle consideré comme noir)
         * @param pas_divide : pas de la discrétisation
         * @param marge : marge de sécurité
         * @param folder_where_to_save_im : chemin du dossier où sauvegarder les images
         * @param resolution : resolution de la map
         * @param origin : coordonnée du points bas gauche de la map dans le repère map
         * 
         */


        ROS_INFO(" ====================  INTIALISATION DU NOEUD PATH PLANNING ..." );

        /***************************************************** Paramètres de la map *****************************************************************************/
        origin_map[0] = origin[0];
        origin_map[1] = origin[1];
        origin_map[2] = origin[2];

        resolution_map = resolution;

        /***************************************************** TOPIC IN OUT  ET CLIENT *****************************************************************************/

        move_goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &PathPlanningNode::moveGoalCallback, this);
        path_pub_ = nh_.advertise<nav_msgs::Path>("/trajectory", 1);
        
       
        client = nh_.serviceClient<path_planning_map::getUnicyclePos>("/get_unicycle_pos");

        // Initialisation de la trajectoire : elle sera toujours dans le repère map
        path_msg_.header.frame_id = "map";

        this->folder_where_to_save_im_ = folder_where_to_save_im;

        /***************************************************** DISCRETISATION A FAIRE UNE FOIS AU DEBUT  *****************************************************************************/
        
        div = Divide(image_binaire_from_pgm, pas_divide);
        div.divide_map(marge);
        div.build_graph_free_subcells();

        this->marge_ = marge;
        this->pas_ = pas_divide;

        

    }




    void moveGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        
        /**
         * @brief Fonction executé dès qu'un objectif est envoyé au noeud : ainsi, dès qu'un objectif est envoyé au noeud, on récupère la position du robot et on lance l'algorithme de Dijkstra
         * avec comme point de départ la position du robot et comme point d'arrivée l'objectif envoyé au noeud
         * 
         * @param msg : objectif de position GOAL envoyé via Rviz
         */



        

        ROS_INFO(" ======================>  NOUVEL OBJECTIF RECU !");


        /*discretisation encore */
        div = Divide(image_binaire_from_pgm, pas_);
        div.divide_map(marge_);
        div.build_graph_free_subcells();

        ROS_INFO("Save in : %s",folder_where_to_save_im_.c_str()); //save le path
        /* =================================== FOR DEBUG  ===================================*/
        div.display_subcells(folder_where_to_save_im_);
    /* =================================== FOR DEBUG  ===================================*/

        

        path_planning_map::getUnicyclePos srv;

        if (client.call(srv)) {

            /********************************** START ==> Récupération de la position du robot et conversion en subcell *******************************************************************/


            nav_msgs::Odometry unicycle_position = srv.response.unicycle_position;

            double x_metres = unicycle_position.pose.pose.position.x;
            double y_metres = unicycle_position.pose.pose.position.y;

            // Convertir la position du robot en une subcell[][] de la grille
            int lignes_subcell, colonnes_subcell;
            int a = div.convert_from_meters_to_free_subcells(x_metres, y_metres, &lignes_subcell, &colonnes_subcell,resolution_map, origin_map, false, false);
            if (a == -1) {
                ROS_ERROR("Le robot est sur un mur.");
                return;
            }

            /********************************** END ==> Récupération de la position de l'objectif et conversion en subcell *******************************************************************/

            double x_end = msg->pose.position.x;
            double y_end = msg->pose.position.y;
            int lignes_subcell_end, colonnes_subcell_end;
            a = div.convert_from_meters_to_free_subcells(x_end, y_end,&lignes_subcell_end, &colonnes_subcell_end,resolution_map, origin_map, false, false);

            if (a == -1) {
                ROS_ERROR("La destination est sur un mur.");
                return;
            }




            /********************************** CONVERTION EN FREE SUBCELL *******************************************************************/

            Subcell* subcell_depart = div.get_one_subcell_with_index(colonnes_subcell,lignes_subcell);
            int id_start = subcell_depart->get_id();

            Subcell* subcell_end = div.get_one_subcell_with_index(colonnes_subcell_end,lignes_subcell_end);
            int id_end = subcell_end->get_id();

            /********************************** DIJKSTRA *******************************************************************/

            std::vector<Subcell*> tableau_pointurs_free_nodes = div.get_subcells_free();
            int nb_free_nodes = div.get_nb_free_nodes();
            Dijsktra dij = Dijsktra(&tableau_pointurs_free_nodes, nb_free_nodes);
            std::vector<int> path = dij.launch_dijsktra(id_start, id_end);

            ROS_INFO("Save in : %s",folder_where_to_save_im_.c_str()); //save le path
            div.display_subcell_state(path, folder_where_to_save_im_);

            // Remplissage et constitution du PATH
            path_msg_.poses.clear();
            path_msg_.header.stamp = ros::Time::now();
        
        
            std::vector<Subcell*> sub_path = dij.get_sub_path();

            for (int i=sub_path.size()-1; i>=0; i--){
                geometry_msgs::PoseStamped pose;
                double x = origin_map[0] + 0 + (sub_path[i]->get_x()* resolution_map);
                double y = origin_map[1] + div.get_rows() * resolution_map - (sub_path[i]->get_y() * resolution_map);

                pose.pose.position.x = x;
                pose.pose.position.y = y;
                pose.pose.position.z = 0.0;
                path_msg_.poses.push_back(pose);

                // pour traj
                
                geometry_msgs::Pose p;
                p.position.x = x;
                p.position.y = y;
                p.position.z = 0.0;



            }


            /********************************** PUBLICATION DU PATH *******************************************************************/
            ROS_INFO(" ....................  Trajectoire calculée et publiée  .................... !");
            path_pub_.publish(path_msg_);



        } else {
            ROS_ERROR("Impossible de récupérer la position du robot.");
            return;
        }
    }

    void run() {
        ros::spin();
    }

};




cv::Mat from_pgm_to_binary(std::string path){
    /**
     * @brief Permet de passer de PGM en binaire pour avoir une image avec des pixels blancs et noirs
     * @param path : chemin de l'image PGM
     * 
     */
  
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





int main(int argc, char** argv) {

    ROS_INFO("LANCEMENT DU NOEUD PATH PLANNING ...");

    //initie le noeud
    ros::init(argc, argv, "PathPlanning");
    ros::NodeHandle nh("~");

    /*********************   YAML contenant les config       ************************/
    std::string param_file_path;
    nh.getParam("param_file", param_file_path);

    if (param_file_path.empty()) {
        ROS_ERROR("Aucun fichier de paramètres spécifié.");
        return -1;
    }

    YAML::Node config = YAML::LoadFile(param_file_path);

    // Initialisation des paramètres
    int pas_divide = config["pas"].as<int>();
    int marge = config["marge"].as<int>();

    std::string folder_where_to_save_im = config["folder_image_saver"].as<std::string>();
    std::string path_pgm_map = config["map_pgm_path"].as<std::string>();
    std::string path_yaml_map = config["map_yaml_path"].as<std::string>();

    
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
    

    cv::Mat image;
    image = from_pgm_to_binary(path_pgm_map);


    PathPlanningNode pathPlanningNode(nh,image,pas_divide,marge,folder_where_to_save_im,resolution,origin);
    pathPlanningNode.run();

    return 0;
}
