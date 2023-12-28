#ifndef DIVIDE_H_PP
#define DIVIDE_H_PP

#include <opencv2/highgui/highgui.hpp>
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
        //dimensions
        int rows;
        int cols;

        //pas de la grille pour permettre de diviser la map
        int pas;

        // un tableau de subcells via la bib C++
        std::vector<Subcell> subcells;

    public :

        // constructeur
        Divide(cv::Mat map_image_original, int pas);
        
        // destructeur par defaut
        virtual ~Divide(){};

        // une methode pour permettre de diviser la map en plusieurs parties
        void divide_map();

        bool detect_subcells(cv::Mat sous_rectangle);

        //display

        void display_subcells();


        // getters
        std::vector<Subcell> get_subcells();
        cv::Mat get_map_image_original();
        int get_pas();

        int get_rows();
        int get_cols();

};

#endif