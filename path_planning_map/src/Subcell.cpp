#include "path_planning_map/Subcell.hpp"
#include <ros/ros.h>




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

    // coordonnees du coin haut gauche en pixels
    this->x = x;
    this->y = y;
    this->is_visited = false; // par defaut, on considere que la subcell n'est pas visitee

    // liste des voisins adjacents : 4 au maximum !! 
    this->voisins_adjacents = std::vector<Subcell*>();
    
    this->cost_for_each_voisin = std::vector<int>();
    this->nb_voisins_adjacents = 0;

    //distance par rapport au point de depart non definie encore
    this->distance_from_start = INFINI;

    // il s'agit de la id-ieme subcell libre du tableau 2D de subcells
    this->id = id;
}



//destructeur
Subcell::~Subcell(){

    /**
     * @brief Destroy the Subcell:: Subcell object
     * 
     */

    //destruction pour les vecteurs
    
    // destruction pointeurs vers les voisins
    for (int i = 0; i < this->voisins_adjacents.size(); i++){
        delete this->voisins_adjacents[i];
    }


}



int Subcell::nb_voisins_one_subcell(std::vector<std::vector<Subcell>> *full_grid){
    /**
     * @brief Permet de savoir combien de voisins adjacents a une subcell
     * 
     */

    int nb_voisins = 0;

    if (full_grid->size() == 0){
        ROS_INFO("Erreur : le tableau 2D de subcells est vide");
        return -1;
    }

    //on recupere sa position dans le tableau 2D
    int indice_ligne = this->y / this->size;
    int indice_colonne = this->x / this->size;

    // UP
    if (indice_ligne - 1 >= 0){
        if (not (*full_grid)[indice_ligne - 1][indice_colonne].get_is_occupied()){
            nb_voisins++;
        }
    }

    if (indice_ligne + 1 < (*full_grid).size()){
        if (not (*full_grid)[indice_ligne + 1][indice_colonne].get_is_occupied()){
            nb_voisins++;
        }
    }

    if (indice_colonne > 0){
        if (not (*full_grid)[indice_ligne][indice_colonne - 1].get_is_occupied()){
            nb_voisins++;
        }
    }
    
    if (indice_colonne + 1 < (*full_grid)[0].size()){
        if (not (*full_grid)[indice_ligne][indice_colonne + 1].get_is_occupied()){
            nb_voisins++;
        }
    }

    return nb_voisins;

}


void Subcell::add_voisin_adjacent(std::vector<std::vector<Subcell>> *full_grid){



    int indice_ligne = this->y / this->size;
    int indice_colonne = this->x / this->size;

    // on regarde les voisins UP DOWN LEFT RIGHT
    if (indice_ligne - 1 >= 0){
        // on regarde le voisin du bas
        if (not (*full_grid)[indice_ligne - 1][indice_colonne].get_is_occupied()){
            this->voisins_adjacents.push_back(&((*full_grid)[indice_ligne - 1][indice_colonne]));
            this->nb_voisins_adjacents++;

            this->cost_for_each_voisin.push_back(1);
            
        
        }
    }

    if (indice_ligne + 1 < (*full_grid).size()){
        if (not (*full_grid)[indice_ligne + 1][indice_colonne].get_is_occupied()){
            this->voisins_adjacents.push_back(&((*full_grid)[indice_ligne + 1][indice_colonne]));
            this->nb_voisins_adjacents++;
            this->cost_for_each_voisin.push_back(1);
        }
    }


    if (indice_colonne > 0){
        // on regarde le voisin de gauche
        if (not (*full_grid)[indice_ligne][indice_colonne - 1].get_is_occupied()){
            this->voisins_adjacents.push_back(&((*full_grid)[indice_ligne][indice_colonne - 1]));
            this->nb_voisins_adjacents++;
            
            this->cost_for_each_voisin.push_back(1);
        }
    }



    if (indice_colonne + 1 < (*full_grid)[0].size()){
        // on regarde le voisin de droite
        if (not (*full_grid)[indice_ligne][indice_colonne + 1].get_is_occupied()){
            this->voisins_adjacents.push_back(&((*full_grid)[indice_ligne][indice_colonne + 1]));
            this->nb_voisins_adjacents++;
            
            this->cost_for_each_voisin.push_back(1);
        }
    }

}

//getter

bool Subcell::get_is_occupied(){
    return this->is_occupied;
}

int Subcell::get_size(){
    return this->size;
}

int Subcell::get_x(){
    return this->x;
}

int Subcell::get_y(){
    return this->y;
}


int Subcell::get_id(){
    return this->id;
}


int Subcell::get_id_previous(){
    return this->id_previous;
}


bool Subcell::get_is_visited(){
    return this->is_visited;
}

std::vector<Subcell*>* Subcell::get_voisins(){
    return &(this->voisins_adjacents);
}


std::vector<int>Subcell::get_cost_for_each_voisin(){
    return this->cost_for_each_voisin;
}

int Subcell::get_nb_voisins_adjacents(){
    return this->nb_voisins_adjacents;
}

int Subcell::get_distance_from_start(){
    return this->distance_from_start;
}


//setter

void Subcell::set_distance_from_start(int distance){
    this->distance_from_start = distance;
}

void Subcell::set_is_visited(bool is_visited){
    this->is_visited = is_visited;
}

void Subcell::set_id_previous(int id_previous){
    this->id_previous = id_previous;
}