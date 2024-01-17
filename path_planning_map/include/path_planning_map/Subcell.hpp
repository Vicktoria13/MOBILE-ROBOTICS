#ifndef SUBCELL_HPP
#define SUBCELL_HPP

#include <vector>

#define INFINI 10000000

/**
 * @brief Sous cellule : chaque SOUS cellule est caractérisé par sa taille et un booléen indiquant si elle est occupée ou non
 *  On considère qu'une sous cellule est occupée si elle contient au moins un pixel noir
*/

class Subcell
{
private:
    bool is_occupied;
    bool is_visited;

    int size;

    int x;
    int y;

    // cela signifie que cette subcell est la id-ieme subcell LIBRE du tableau 2D de subcells, 0 étant la première subcell libre a partir de haut à gauche
    int id; 

    // liste des voisins : chaque subcell aura une liste de maximum 4 voisins adjacents ( 8 si on compte les diagonales)
    //vecteur de pointeurs de subcells
    std::vector<Subcell*> voisins_adjacents;
    int nb_voisins_adjacents;

    // cost_for_each_voisin est une liste dans l'ordre des voisins adjacents
    // Ainsi, cost_for_each_voisin[0] correspond au cout entre cette subcell et voisins_adjacents[0]
    // cost_for_each_voisin[1] correspond au cout entre cette subcell et voisins_adjacents[1] ...
    std::vector<int> cost_for_each_voisin;

    int distance_from_start;

    // chaque subcell contient aussi l'id de la subcell precedente (ie la id_previous-ieme subcell libre du tableau 2D de subcells)
    int id_previous;


public:

    Subcell(bool is_occupied, int size, int x, int y,int id);
    ~Subcell();

    //getter
    bool get_is_occupied();
    int get_size();
    int get_x();
    int get_y();
    int get_nb_voisins_adjacents();
    int get_distance_from_start();
    int get_id();
    std::vector<Subcell*>* get_voisins();
    std::vector<int> get_cost_for_each_voisin();
    int get_id_previous();



    //setter
    void set_distance_from_start(int distance);
    void set_is_visited(bool is_visited);
    void set_id_previous(int id_previous);

    

    // permet d'ajouter les voisins adjacents + cout unitaire ==> premier passage
    void add_voisin_adjacent(std::vector<std::vector<Subcell>> *full_grid, int margin);

    bool check_neighbour_around_marge(std::vector<std::vector<Subcell>> *full_grid,int marge, int* distance_detected_from_wall);
    int nb_voisins_one_subcell(std::vector<std::vector<Subcell>> *full_grid);
    
    bool get_is_visited();


};




#endif
