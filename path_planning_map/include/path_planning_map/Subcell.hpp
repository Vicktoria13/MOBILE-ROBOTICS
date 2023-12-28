#ifndef SUBCELL_HPP
#define SUBCELL_HPP


/**
 * @brief Sous cellule : chaque SOUS cellule est caractérisé par sa taille et un booléen indiquant si elle est occupée ou non
 *  On considère qu'une sous cellule est occupée si elle contient au moins un pixel noir
*/

class Subcell
{
private:
    bool is_occupied;

    int size;
    //position du coin haut gauche
    int x;
    int y;

    int id;

public:

    Subcell(bool is_occupied, int size, int x, int y,int id);
    ~Subcell();

    //getter
    bool get_is_occupied();
    int get_size();
    int get_x();
    int get_y();
    int get_id();


};




#endif