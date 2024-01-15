#include "path_planning_map/Dijsktra.hpp"
#include <limits>


Dijsktra::Dijsktra(std::vector<Subcell*>* only_free_node_from_grid,int nb_free_cells){
    /**
     * @brief 
     * @param only_free_node_from_grid : adresse d'un tableau qui ne contient que des adresses 'de subcells libres'
     * 
     */
    ROS_INFO("Initialisation de Dijsktra avec %d noeuds libres", nb_free_cells);
    this->nb_nodes = nb_free_cells;

    this->unvisited = only_free_node_from_grid;

    this->nb_unvisited_node_left = nb_free_cells;
    this-> nb_visited_node = 0;
}


Dijsktra::~Dijsktra()
{
    //destructeur
    
}




int Dijsktra::minDistance(){
    /**
     * @brief Retourne l'indice ! vers la subcell du tableau pointe par unvisited qui a la plus petite distance
     * 
     */


   int indice_min = 0;
   int min = std::numeric_limits<int>::max();

   //unvisited est donc de type std::vector<Subcell*>* : un pointeur vers un tableau de pointeur type Subcell
   //this->unvisited->at(i) est un pointeur vers une subcell situé a i


    for (int i = 0; i < this->nb_nodes; i++){
    
        if (this->unvisited->at(i)->get_is_visited() == false){

            if (this->unvisited->at(i)->get_distance_from_start() < min){
                min = this->unvisited->at(i)->get_distance_from_start();
                indice_min = i;}

        }     
    }

    return indice_min;
    
}


std::vector<int> Dijsktra::launch_dijsktra(int indice_start, int indice_end){
    /**
     * @brief 
     * @param indice_start : indice de la subcell de depart dans this->unvisited
     * @param indice_end : indice de la subcell d'arrivee dans this->unvisited
     * @return std::vector<Subcell> : le chemin a suivre
     */
    

    std::vector<int> path;
    std::vector<Subcell*> subcells_path;
    
    /*********** ETAPE 1 : /on met la distance de start a 0, et les autres a infini ***********/
    //on met la distance de start a 0 : unvisited est une adresse qui pointe vers un tableau d'adresses de subcells !!
    this->unvisited->at(indice_start)->set_distance_from_start(0);

    //ajout a path
    path.push_back(indice_end);
    

   /*********** Etape 2 : on parcourt tous les noeuds *********/

   while (nb_unvisited_node_left > 0){

        //indice_minimum_unvisited : indice du subcell a distance minimum dans UNVISITED
        int indice_minimum_unvisited = minDistance(); 
        std::vector<Subcell*>* voisins = this->unvisited->at(indice_minimum_unvisited)->get_voisins();

        for (int S=0;S<voisins->size();S++){

            // current_distance = dist(START, unvisited) + cout(unvisited, voisin)
            int current_distance = this->unvisited->at(indice_minimum_unvisited)->get_distance_from_start() + this->unvisited->at(indice_minimum_unvisited)->get_cost_for_each_voisin()[S];

            if (current_distance < voisins->at(S)->get_distance_from_start()){
                // on met a jour la distance du S-eme voisin de la subcell que l'on visite
                voisins->at(S)->set_distance_from_start(current_distance);    
                voisins->at(S)->set_id_previous(indice_minimum_unvisited);
            }
        }
        // on set le flag is_visited a true
        this->unvisited->at(indice_minimum_unvisited)->set_is_visited(true);

        
        this->nb_unvisited_node_left--;
        this->nb_visited_node++;
        
   }


    //construction du chemin a partir des ID_PREVIOUS : on commence par le sommet d'arrivee END
    int current_id = indice_end;
    Subcell* current_subcell = this->unvisited->at(current_id);

    while (current_id != indice_start){
        path.push_back(current_id);
        subcells_path.push_back(current_subcell);

        current_id = this->unvisited->at(current_id)->get_id_previous();
        current_subcell = this->unvisited->at(current_id);
    }

    path.push_back(indice_start);

    Subcell* start_subcell = this->unvisited->at(indice_start);
    subcells_path.push_back(start_subcell);

    this->sub_path = subcells_path;
   
   ROS_INFO("FIN DE LA RECHERCHE DU CHEMIN : il y a %d nodes dans le path", path.size());
   return path;
   

}

