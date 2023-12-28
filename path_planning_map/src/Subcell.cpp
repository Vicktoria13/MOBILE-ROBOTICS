#include "path_planning_map/Subcell.hpp"


Subcell::Subcell(bool is_occupied, int size, int x, int y,int id){

    /**
     * @brief Construct a new Subcell:: Subcell object
     * 
     * @param is_occupied : si la subcell contient un pixel noir
     * @param pas : la taille de la subcell
     * @param x : la position en x de la subcell
     * @param y : la position en y de la subcell
     * 
     */

    this->is_occupied = is_occupied;
    this->size = size;
    this->x = x;
    this->y = y;
    this->id = id;

}



//destructeur
Subcell::~Subcell(){

    /**
     * @brief Destroy the Subcell:: Subcell object
     * 
     */

}
