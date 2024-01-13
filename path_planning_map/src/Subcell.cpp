#include "path_planning_map/Subcell.hpp"
#include <ros/ros.h>


#define MARGE 3
//marge en nombre de pixels
// cela signifie que les 2 pixels a partir d'un bord de mur auront au cout elevé


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
    
    // destruction pointeurs vers les voisins
    for (int i = 0; i < this->voisins_adjacents.size(); i++){
        delete this->voisins_adjacents[i];
    }


}



int Subcell::nb_voisins_one_subcell(std::vector<std::vector<Subcell>> *full_grid){
    /**
     * @brief Permet de savoir combien de voisins adjacents a une subcell
     * Hypothese: la subcell en argument n'est pas un mur !
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





bool Subcell::check_neighbour_around_marge(std::vector<std::vector<Subcell>> *full_grid,int marge, int* distance_detected_from_wall){
    /**
     * @brief Etant donné une subcell et une marge pix, on regarde si la subcell a au moins 
     * marge pixel vers le haut qui ne sont pas des murs
     * marge pixel vers le bas qui ne sont pas des murs
     * marge pixel vers la droite qui ne sont pas des murs
     * marge pixel vers la gauche qui ne sont pas des murs
     * 
     * si oui, renvoie True et  met distance_detected_from_wall a -1
     * 
     * sinon , renvoie False et met distance_detected_from_wall a la distance du mur le plus proche
     * 
     * En gros, regarde si le cercle de marge pixel autour est bien libre
     * 
     * Hypothse : la subcell existe et n'est pas un mur
     */

    //on recupere sa position dans le tableau 2D
    int indice_ligne = this->y / this->size;
    int indice_colonne = this->x / this->size;


    bool flag_wall_detected_in_the_marge = false;

    // UP


    for (int i_eme_voisin=1; i_eme_voisin <= marge and indice_ligne - i_eme_voisin >= 0 and not flag_wall_detected_in_the_marge; i_eme_voisin++){
        if ((*full_grid)[indice_ligne - i_eme_voisin][indice_colonne].get_is_occupied()){
            *distance_detected_from_wall = i_eme_voisin;
            flag_wall_detected_in_the_marge = true;
        }
    }

    if (flag_wall_detected_in_the_marge){
        return false;
    }

    //down

    for (int i_eme_voisin=1; i_eme_voisin <= marge and indice_ligne + i_eme_voisin < (*full_grid).size() and not flag_wall_detected_in_the_marge; i_eme_voisin++){
        if ((*full_grid)[indice_ligne + i_eme_voisin][indice_colonne].get_is_occupied()){
            *distance_detected_from_wall = i_eme_voisin;
            flag_wall_detected_in_the_marge = true;
        }
    }

    if (flag_wall_detected_in_the_marge){
        return false;
    }

    //left

    for (int i_eme_voisin=1; i_eme_voisin <= marge and indice_colonne - i_eme_voisin >= 0 and not flag_wall_detected_in_the_marge;i_eme_voisin++){
        if ((*full_grid)[indice_ligne][indice_colonne - i_eme_voisin].get_is_occupied()){
            *distance_detected_from_wall = i_eme_voisin;
            flag_wall_detected_in_the_marge = true;
        }
    }

    if (flag_wall_detected_in_the_marge){
        return false;
    }

    //right

    for (int i_eme_voisin=1; i_eme_voisin <= marge and indice_colonne + i_eme_voisin < (*full_grid)[0].size() and not flag_wall_detected_in_the_marge;i_eme_voisin++){
        if ((*full_grid)[indice_ligne][indice_colonne + i_eme_voisin].get_is_occupied()){
            *distance_detected_from_wall = i_eme_voisin;
            flag_wall_detected_in_the_marge = true;
        }
    }

    if (flag_wall_detected_in_the_marge){
        return false;
    }

    // les 4 diagonales

    // UP LEFT

    for (int i_eme_voisin=1; i_eme_voisin <= marge and indice_ligne - i_eme_voisin >= 0 and indice_colonne - i_eme_voisin >= 0 and not flag_wall_detected_in_the_marge;i_eme_voisin++){
        if ((*full_grid)[indice_ligne - i_eme_voisin][indice_colonne - i_eme_voisin].get_is_occupied()){
            *distance_detected_from_wall = i_eme_voisin;
            flag_wall_detected_in_the_marge = true;
        }
    }

    if (flag_wall_detected_in_the_marge){
        return false;
    }

    // UP RIGHT

    for (int i_eme_voisin=1; i_eme_voisin <= marge and indice_ligne - i_eme_voisin >= 0 and indice_colonne + i_eme_voisin < (*full_grid)[0].size() and not flag_wall_detected_in_the_marge;i_eme_voisin++){
        if ((*full_grid)[indice_ligne - i_eme_voisin][indice_colonne + i_eme_voisin].get_is_occupied()){
            *distance_detected_from_wall = i_eme_voisin;
            flag_wall_detected_in_the_marge = true;
        }
    }

    if (flag_wall_detected_in_the_marge){
        return false;
    }

    // DOWN LEFT

    for (int i_eme_voisin=1; i_eme_voisin <= marge and indice_ligne + i_eme_voisin < (*full_grid).size() and indice_colonne - i_eme_voisin >= 0 and not flag_wall_detected_in_the_marge;i_eme_voisin++){
        if ((*full_grid)[indice_ligne + i_eme_voisin][indice_colonne - i_eme_voisin].get_is_occupied()){
            *distance_detected_from_wall = i_eme_voisin;
            flag_wall_detected_in_the_marge = true;
        }
    }

    if (flag_wall_detected_in_the_marge){
        return false;
    }

    // DOWN RIGHT

    for (int i_eme_voisin=1; i_eme_voisin <= marge and indice_ligne + i_eme_voisin < (*full_grid).size() and indice_colonne + i_eme_voisin < (*full_grid)[0].size() and not flag_wall_detected_in_the_marge;i_eme_voisin++){
        if ((*full_grid)[indice_ligne + i_eme_voisin][indice_colonne + i_eme_voisin].get_is_occupied()){
            *distance_detected_from_wall = i_eme_voisin;
            flag_wall_detected_in_the_marge = true;
        }
    }

    if (flag_wall_detected_in_the_marge){
        return false;
    }




    // si on arrive ici, c'est que la subcell a au moins marge pixels vers le haut, le bas, la gauche et la droite qui ne sont pas des murs
    // on met donc distance_detected_from_wall a -1
    *distance_detected_from_wall = -1;
    return true;
    

    
}


void Subcell::add_voisin_adjacent(std::vector<std::vector<Subcell>> *full_grid){



    int indice_ligne = this->y / this->size;
    int indice_colonne = this->x / this->size;




    int distance_detected_from_wall = -1;

    //on regarde le voisin du haut
    if (indice_ligne - 1 >= 0){
        if (not (*full_grid)[indice_ligne - 1][indice_colonne].get_is_occupied()){
            this->voisins_adjacents.push_back(&((*full_grid)[indice_ligne - 1][indice_colonne]));
            this->nb_voisins_adjacents++;

            if (not (*full_grid)[indice_ligne - 1][indice_colonne].check_neighbour_around_marge(full_grid,MARGE,&distance_detected_from_wall)){
                this->cost_for_each_voisin.push_back((MARGE - distance_detected_from_wall+1) * 25);
            }
            else{
                this->cost_for_each_voisin.push_back(1);
            }
            
        }


    }

    //on regarde le voisin du bas

    if (indice_ligne+1<(*full_grid).size()){
        if (not (*full_grid)[indice_ligne + 1][indice_colonne].get_is_occupied()){
            this->voisins_adjacents.push_back(&((*full_grid)[indice_ligne + 1][indice_colonne]));
            this->nb_voisins_adjacents++;

            //cout : on regarde si le voisin du bas a un cerle de rayon MARGE autour de lui qui est bien libre
            // si oui, on met un cout de 1
            // sinon, on met un cout proportionnel a la distance du mur le plus proche
            if (not (*full_grid)[indice_ligne + 1][indice_colonne].check_neighbour_around_marge(full_grid,MARGE,&distance_detected_from_wall)){
                this->cost_for_each_voisin.push_back((MARGE - distance_detected_from_wall+1) * 25);
            }
            else{
                this->cost_for_each_voisin.push_back(1);
            }
        }
    }

    //on regarde le voisin de gauche

    if (indice_colonne-1>=0){
        if (not (*full_grid)[indice_ligne][indice_colonne - 1].get_is_occupied()){
            this->voisins_adjacents.push_back(&((*full_grid)[indice_ligne][indice_colonne - 1]));
            this->nb_voisins_adjacents++;

            //cout : on regarde si le voisin de gauche a un cerle de rayon MARGE autour de lui qui est bien libre
            // si oui, on met un cout de 1
            // sinon, on met un cout proportionnel a la distance du mur le plus proche
            if (not (*full_grid)[indice_ligne][indice_colonne - 1].check_neighbour_around_marge(full_grid,MARGE,&distance_detected_from_wall)){
                this->cost_for_each_voisin.push_back((MARGE - distance_detected_from_wall+1) * 25);
            }
            else{
                this->cost_for_each_voisin.push_back(1);
            }
        }
    }

    //on regarde le voisin de droite

    if (indice_colonne+1<(*full_grid)[0].size()){
        if (not (*full_grid)[indice_ligne][indice_colonne + 1].get_is_occupied()){
            this->voisins_adjacents.push_back(&((*full_grid)[indice_ligne][indice_colonne + 1]));
            this->nb_voisins_adjacents++;

            //cout : on regarde si le voisin de droite a un cerle de rayon MARGE autour de lui qui est bien libre
            // si oui, on met un cout de 1
            // sinon, on met un cout proportionnel a la distance du mur le plus proche
            if (not (*full_grid)[indice_ligne][indice_colonne + 1].check_neighbour_around_marge(full_grid,MARGE,&distance_detected_from_wall)){
                this->cost_for_each_voisin.push_back((MARGE - distance_detected_from_wall+1) * 25);
            }
            else{
                this->cost_for_each_voisin.push_back(1);
            }
        }
    }
    
    



    /*

    // on regarde les voisins UP DOWN LEFT RIGHT
    if (indice_ligne - 1 >= 0){
        if (not (*full_grid)[indice_ligne - 1][indice_colonne].get_is_occupied()){
            this->voisins_adjacents.push_back(&((*full_grid)[indice_ligne - 1][indice_colonne]));
            this->nb_voisins_adjacents++;
            
            int i = 1;
            bool flag_cout_deja_mis = false;

            while (i <= MARGE and indice_ligne - i >= 0 and not flag_cout_deja_mis){
                if (not (*full_grid)[indice_ligne - i][indice_colonne].get_is_occupied()){
                    // le i-eme voisin du haut est n'est pas un mur, donc on regarde combien de voisins il a
                    if ((*full_grid)[indice_ligne - i][indice_colonne].nb_voisins_one_subcell(full_grid) != 4){
                        this->cost_for_each_voisin.push_back((MARGE - i+1) * 5);
                        flag_cout_deja_mis = true;
                    }
                }

            
                i++;
            }

            if (not flag_cout_deja_mis){
                this->cost_for_each_voisin.push_back(1);
            }
          

        }
    }

    if (indice_ligne + 1 < (*full_grid).size()){
        if (not (*full_grid)[indice_ligne + 1][indice_colonne].get_is_occupied()){
            this->voisins_adjacents.push_back(&((*full_grid)[indice_ligne + 1][indice_colonne]));
            this->nb_voisins_adjacents++;
            
            
            int i = 1;
            bool flag_cout_deja_mis = false;

            while (i <= MARGE and indice_ligne + i < (*full_grid).size() and not flag_cout_deja_mis){
                if (not (*full_grid)[indice_ligne + i][indice_colonne].get_is_occupied()){
                    // le i-eme voisin du bas est n'est pas un mur, donc on regarde combien de voisins il a
                    if ((*full_grid)[indice_ligne + i][indice_colonne].nb_voisins_one_subcell(full_grid) != 4){
                        this->cost_for_each_voisin.push_back((MARGE - i+1) * 5);
                        flag_cout_deja_mis = true;
                    }
                }

                i++;
            }

            if (not flag_cout_deja_mis){
                this->cost_for_each_voisin.push_back(1);
            }
        }
    }

    // plus je detecte un mur proche, plus i petit, plus le cout est eleve

    if (indice_colonne > 0){
        // on regarde le voisin de gauche
        if (not (*full_grid)[indice_ligne][indice_colonne - 1].get_is_occupied()){
            this->voisins_adjacents.push_back(&((*full_grid)[indice_ligne][indice_colonne - 1]));
            this->nb_voisins_adjacents++;
            
            /*
            if ( (*full_grid)[indice_ligne][indice_colonne - 1].nb_voisins_one_subcell(full_grid) == 4){
                this->cost_for_each_voisin.push_back(1);
            }
            else{
                this->cost_for_each_voisin.push_back(20);
            }
            

           // on regarde pour le 1er voisins de gauche, le 2e, puis le 3e, jusqu'au MARGE-ieme
           // pour le 1er : S'il n'est pas un mur, on regarde combien de voisins il a
           // s'il n'en a pas 4 alors c'est sans doute un bord, donc on lui met un cout elevée de (MARGE=3 - 1) * 5 = 10
           // s'il en a 4, alors on explore celui encore a gauche : s'il n'est pas un mur, on lui met un cout de (MARGE=3 - 2) * 5 = 5

            int i = 1;
            bool flag_cout_deja_mis = false;
            
            while (i <= MARGE and indice_colonne - i >= 0 and not flag_cout_deja_mis){
                if (not (*full_grid)[indice_ligne][indice_colonne - i].get_is_occupied()){
                    // le i-eme voisin de gauche est n'est pas un mur, donc on regarde combien de voisins il a
                    if ((*full_grid)[indice_ligne][indice_colonne - i].nb_voisins_one_subcell(full_grid) != 4){
                        this->cost_for_each_voisin.push_back((MARGE - i+1) * 5);
                        flag_cout_deja_mis = true;
                    }
                }

               
                i++;

                
            }

            if (not flag_cout_deja_mis){
                this->cost_for_each_voisin.push_back(1);
            }

            
        }
    }



    if (indice_colonne + 1 < (*full_grid)[0].size()){
        // on regarde le voisin de droite
        if (not (*full_grid)[indice_ligne][indice_colonne + 1].get_is_occupied()){
            this->voisins_adjacents.push_back(&((*full_grid)[indice_ligne][indice_colonne + 1]));
            this->nb_voisins_adjacents++;
            
            int i = 1;
            bool flag_cout_deja_mis = false;

            while (i <= MARGE and indice_colonne + i < (*full_grid)[0].size() and not flag_cout_deja_mis){
                if (not (*full_grid)[indice_ligne][indice_colonne + i].get_is_occupied()){
                    // le i-eme voisin de droite est n'est pas un mur, donc on regarde combien de voisins il a
                    if ((*full_grid)[indice_ligne][indice_colonne + i].nb_voisins_one_subcell(full_grid) != 4){
                        this->cost_for_each_voisin.push_back((MARGE - i+1) * 5);
                        flag_cout_deja_mis = true;
                    }
                }

              
                i++;
            }

            if (not flag_cout_deja_mis){
                this->cost_for_each_voisin.push_back(1);
            }
        }
    }

    */

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