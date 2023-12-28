#include <opencv4/opencv2/highgui.hpp>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include "path_planning_map/Divide.hpp"

using namespace std;

int main(int argc, char** argv)
{
    // Initialisation du nœud ROS
    ros::init(argc, argv, "simple_node");
    ros::NodeHandle nh;

    // Charger une image OpenCV en binaire
    cv::Mat image = cv::imread("/home/spi-2019/test1.png", cv::IMREAD_GRAYSCALE);
    Divide div = Divide(image, 5);
    //print dimensions
    //cout << "rows: " << div.get_rows() << endl;
    //cout << "cols: " << div.get_cols() << endl;

    div.display_image_with_tab();


    // diviser la map
    int nb_celles = div.divide_map();
    ROS_INFO("============= INFO =======================");
    ROS_INFO("Nombre lignes: %d", div.get_rows());
    ROS_INFO("Nombre colonnes: %d", div.get_cols());
    ROS_INFO("Pas: %d", div.get_pas());
    ROS_INFO("Nombre de subcells: %d", nb_celles);
    ROS_INFO("==========================================");

    //display
    div.display_subcells();



    

    // Vérifier si l'image est chargée avec succès
    if (image.empty()) {
        ROS_ERROR("Impossible de charger l'image.");
        return -1;
    }

    // Créer un éditeur pour publier sur le topic "/image_topic"
    ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("/image_topic", 1);

    ros::Rate rate(1);  // Fréquence de publication (1 Hz, par exemple)

    while (ros::ok()) {
        // Afficher l'image
        cv::imshow("Image chargée", image);
        cv::waitKey(1);  // Attendre un court instant pour permettre l'affichage

        // Créer un message Image ROS
        sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

        // Publier le message image sur le topic "/image_topic"
        image_pub.publish(image_msg);
        ROS_INFO("Image publiée sur le topic /image_topic");

        // Attendre que le message soit publié avant de passer à la prochaine itération
        ros::spinOnce();

        // Attendre pour maintenir la fréquence spécifiée
        rate.sleep();
    }

    return 0;
}
