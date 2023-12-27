#ifndef DIVIDE_H_PP
#define DIVIDE_H_PP

#include <opencv2/highgui/highgui.hpp>
#include "Subcell.hpp"
#include <vector>


/**
 * @brief Classe permettant de faire une division de la map en plusieurs parties
 * 
 */
class Divide{
    protected :

        // the map
        cv::Mat map_image_original;
        //dimensions
        int rows;
        int cols;

        //pas
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

};

#endif