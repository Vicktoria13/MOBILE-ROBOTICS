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

    // Charger une image OpenCV en binnaire
    cv::Mat image = cv::imread("src/minilab_simulation/map/buvette.pgm");
//    Divide div = Divide(image, 10);
    //print dimensions




    

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
