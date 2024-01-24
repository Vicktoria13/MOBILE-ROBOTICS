#ifndef DIVIDE_H_PP
#define DIVIDE_H_PP

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "path_planning_map/Subcell.hpp"
#include <vector>
#include <ros/ros.h>


/**
 * @brief Classe permettant de faire une division de la map en plusieurs parties
 * 
 */
class Divide{
    protected :

        // the map reçu par le noeud map_processing
        cv::Mat map_image_original;
        //dimensions en pixels de l'image !
        int rows;
        int cols;

        //pas de la grille pour permettre de diviser la map
        int pas;

        // l'occupancy grid sous forme de tableau 2D de subcells
        std::vector<std::vector<Subcell>> subcells;

        // tableau avec seulement les subcells libres
        //std::vector<Subcell> subcells_free;
        std::vector<Subcell*> subcells_free;

        int nb_rows_discrete;
        int nb_cols_discrete;

        int nb_free_nodes;

        

    public :

        Divide(){};
        // constructeur
        Divide(cv::Mat map_image_original, int pas);
        
        // destructeur par defaut
        virtual ~Divide(){};

        // une methode pour permettre de diviser la map en plusieurs parties
        int divide_map(int marge_for_voisins);
        void divide_map_for_exploring();


        bool detect_subcells(cv::Mat sous_rectangle);

        //display

        void display_subcells(std::string path_folder);
        void display_exploration(std::string path_folder,int id_ligne_rob, int id_colonne_rob);

        //for debug
        void get_rid_inconsitencies();

        cv::Mat filter_free_subcells();

        void build_graph_free_subcells();

        void display_subcell_state(std::vector<int> path, std::string path_folder);

    

        int convert_from_meters_to_free_subcells(float x_meters, float y_meters, int* x_subcells, int* y_subcells, float resolution, double* origin_coin_bas_gauche,bool debug_draw,bool for_explore);
        

        // getters
        std::vector<std::vector<Subcell>> get_subcells();

        std::vector<Subcell*> get_subcells_free(){
            return this->subcells_free;
        }

        Subcell* get_one_subcell_free_with_index(int index){
            return this->subcells_free[index];
        
        }


        Subcell* get_one_subcell_with_index(int index_row, int index_col){
            return &this->subcells[index_row][index_col];
        }




        cv::Mat get_map_image_original();
        int get_pas();

        int get_rows();
        int get_cols();
        int get_nb_rows_discrete();
        int get_nb_cols_discrete();

        int get_nb_free_nodes();


};

#endif