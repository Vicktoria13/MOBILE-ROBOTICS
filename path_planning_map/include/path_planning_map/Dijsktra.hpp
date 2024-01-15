#ifndef DIJSKTRA_HPP
#define DIJSKTRA_HPP


#include "path_planning_map/Divide.hpp"

class Dijsktra
{
private:
    int nb_nodes;

    //pointeur vers un tableau de pointeurs
    std::vector<Subcell*>* unvisited;

    // vecteur des pointeurs sur subcells du chemin
    std::vector<Subcell*> sub_path;

    std::vector<Subcell> previous;

    int nb_unvisited_node_left;
    int nb_visited_node;

public:
    Dijsktra(std::vector<Subcell*>* only_free_node_from_grid,int nb_free_cells);
    ~Dijsktra();
    int minDistance();

    // algo dijsktra : retourne une liste de coordonnées [[1,2], [3,4], [5,6] ...]
    std::vector<int> launch_dijsktra(int indice_start, int indice_end);

    std::vector<Subcell*> get_sub_path(){
        return this->sub_path;
    }
};





#endif