#include "autonomous_explore/UnicycleAutonome.hpp"
#include <iostream>
#include <ros/ros.h>


UnicycleAutonome::UnicycleAutonome()
{
    this->current_pose_x_meters = 0;
    this->current_pose_y_meters = 0;
    this->current_pose_theta_radians = 0;
}


UnicycleAutonome::UnicycleAutonome( std::vector<std::vector<Subcell>>* map,float current_pose_x_meters, float current_pose_y_meters, float current_pose_theta_radians)
{
    this->current_pose_x_meters = current_pose_x_meters;
    this->current_pose_y_meters = current_pose_y_meters;
    this->current_pose_theta_radians = current_pose_theta_radians;


    this->map = map;


}

void UnicycleAutonome::update_pose_robot(float current_pose_x_meters, float current_pose_y_meters, float current_pose_theta_radians){

    /**
     * @brief Permet d'update les parametres du robot : va etre a appele a chaque callbaack
     * 
     */

    this->current_pose_x_meters = current_pose_x_meters;
    this->current_pose_y_meters = current_pose_y_meters;
    this->current_pose_theta_radians = current_pose_theta_radians;


}

void UnicycleAutonome::update_map(std::vector<std::vector<Subcell>>* map){

    /**
     * @brief Permet d'update la map : va etre a appele a chaque callbaack
     * 
     */

    this->map = map;

}



void UnicycleAutonome::display_rayon_action(){

    //on affiche le rayon d'action


}

UnicycleAutonome::~UnicycleAutonome()
{

}

//getter
float UnicycleAutonome::get_current_pose_x_meters(){
    return this->current_pose_x_meters;
}

float UnicycleAutonome::get_current_pose_y_meters(){
    return this->current_pose_y_meters;
}

float UnicycleAutonome::get_current_pose_theta_radians(){
    return this->current_pose_theta_radians;
}
