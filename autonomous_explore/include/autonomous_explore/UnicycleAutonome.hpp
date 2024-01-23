#ifndef UNICYCLEAUTONOME_HPP
#define UNICYCLEAUTONOME_HPP

#include "path_planning_map/Divide.hpp"
#include "path_planning_map/Subcell.hpp"

class UnicycleAutonome
{
private:
    float current_pose_x_meters;
    float current_pose_y_meters;
    float current_pose_theta_radians;

    std::vector<std::vector<Subcell>>* map;

public:
    UnicycleAutonome();

    UnicycleAutonome(std::vector<std::vector<Subcell>>* map,float current_pose_x_meters, float current_pose_y_meters, float current_pose_theta_radians);
    ~UnicycleAutonome();

    void update_pose_robot(float current_pose_x_meters, float current_pose_y_meters, float current_pose_theta_radians);
    void update_map(std::vector<std::vector<Subcell>>* map);


    void display_rayon_action();


    //getters
    float get_current_pose_x_meters();
    float get_current_pose_y_meters();
    float get_current_pose_theta_radians();

};




#endif