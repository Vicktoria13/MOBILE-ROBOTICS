#include "path_planning_map/Divide.hpp"
#include <iostream>
#include <ros/ros.h>



// on implemente les fonctions




Divide::Divide(cv::Mat map_image_original, int pas){

    /**
     * @brief Construct a new Divide:: Divide object
     * 
     * @param map_image_original : la map
     * @param pas : la taille des subcells
     * 
     */

    this->map_image_original = map_image_original;
    this->pas = pas;

    // on recupere les dimensions de la map
    this->rows = map_image_original.rows;
    this->cols = map_image_original.cols;

}


int Divide::divide_map(){

    /**
     * @brief On divise , cree subcell, leur attribue un bool et on stocke dans un tableau
     * 
     */


    // on parcourt les ROI de la map, de taille pas*pas
    int compteur_subcells = 0;

    for (int r = 0; r < rows; r+=pas) {
        for (int c = 0; c < cols; c+=pas) {

            // on verifie que la subcell ne depasse pas de la map
            if (r < rows && c  < cols){
                compteur_subcells++;
                cv::Mat sous_rectangle = map_image_original(cv::Rect(c, r, pas, pas));
                bool is_occupied = detect_subcells(sous_rectangle);

       
                subcells.push_back(Subcell(is_occupied, pas, c, r, compteur_subcells));
                
              
            }
            
        }
        //fin de la ligne r
        //ROS_INFO("fin de la ligne %d", r);
        //ROS_INFO("compteur_subcells: %d", compteur_subcells);
    }

    //retourne la taille du tableau de subcells
    return compteur_subcells;

    

}





bool Divide::detect_subcells(cv::Mat sous_rectangle){

    /**
     * @brief Etant donné une subcell donné, on regarde si il y a au moins un pixel noir
     * Si oui, on la considère comme une subcell occupée
     * 
     * @return true si la subcell est occupée, false sinon
     */

    
    for (int r = 0; r < sous_rectangle.rows; r++) {
        for (int c = 0; c < sous_rectangle.cols; c++) {

            int8_t map_value = sous_rectangle.at<uchar>(r, c);
            // Si 0, alors c'est noir, donc c occupé
            if (map_value == 0 ){
                // TRUE = OCCUPE
                return true;
            }

        }
    }    
    //cv::imwrite("/home/spi-2019/res/libre/" + std::to_string(random()) + ".png", sous_rectangle);
    return false;

}



void Divide::display_subcells(){


    // affiche sous forme d'image et enregistre
    // la fenetre est séparée en 2 : a gauche, la map d'origine, a droite , la map avec les subcells avec un grillage
    cv::Mat map_image = map_image_original.clone();

    //convert to RGB
    cv::cvtColor(map_image, map_image, cv::COLOR_GRAY2RGB);

    // on creer une image RGB
    cv::Mat map_image_subcells = map_image_original.clone();
    cv::cvtColor(map_image_subcells, map_image_subcells, cv::COLOR_GRAY2RGB);

    // on parcourt les subcells
    for (int i = 0; i < subcells.size(); i++) {
        Subcell subcell = subcells[i];
        // on recupere les coordonnées de la subcell
        int x = subcell.get_x();
        int y = subcell.get_y();
        int size = subcell.get_size();

        // on dessine un rectangle situé aux coordonnées de la subcell : celles occupées sont en B, celles libre en vert
        if (subcell.get_is_occupied())
        {
            cv::rectangle(map_image_subcells, cv::Point(x, y), cv::Point(x+size, y+size), cv::Scalar(0, 0, 255), 1);
        }     
    }

    //2e boucle pour dessiner par dessus les lignes verticales et horizontales
    for (int i = 0; i < subcells.size(); i++) {
        Subcell subcell = subcells[i];
        // on recupere les coordonnées de la subcell
        int x = subcell.get_x();
        int y = subcell.get_y();
        int size = subcell.get_size();

        // on dessine un rectangle situé aux coordonnées de la subcell : celles occupées sont en B, celles libre en vert
        if (not subcell.get_is_occupied())
        {
            cv::rectangle(map_image_subcells, cv::Point(x, y), cv::Point(x+size, y+size), cv::Scalar(255, 0, 0), 1);
        }     
    }

    // on concatene les 2 images
    cv::Mat map_image_concat;
    cv::hconcat(map_image, map_image_subcells, map_image_concat);

    // on enregistre
    cv::imwrite("/home/spi-2019/compare.png", map_image_concat);

    

}


void Divide::display_image_with_tab(){
    // affiche this->map_image_original sous forme de tab dans le terminal
    

    cv::Mat tmp = cv::Mat::zeros(rows, cols, CV_8UC1);
    for (int r = 0; r < tmp.rows; r++) {
        for (int c = 0; c < tmp.cols; c++) {

            int8_t map_value = map_image_original.at<uchar>(r, c);
            // 0 = obstacle
            if (map_value == 0 ){
                tmp.at<uchar>(r, c) = 0;
            }
            else if (map_value == -1){
                tmp.at<uchar>(r, c) = 255;
            }
        }
    }

    this->map_image_original = tmp.clone();
    /*
    int nb_unknown = 0;

    cv::Mat tmp = map_image_original.clone();
    
    for (int r = 0; r < tmp.rows; r++) {
        for (int c = 0; c < tmp.cols; c++) {
                
                int8_t map_value = tmp.at<uchar>(r, c);
                // Si 0, alors c'est noir, donc c occupé
                if (map_value == 0 ){
                    nb_noir++;
                }
                else if (map_value == 255){
                    nb_blanc++;
                }

                else{
                    nb_unknown++;
                }
            }
            
    }

    ROS_INFO("nb_blanc: %d", nb_blanc);
    ROS_INFO("nb_noir: %d", nb_noir);
    ROS_INFO("nb_unknown: %d", nb_unknown);
    ROS_INFO("==========================================");   

    cv::imwrite("/home/spi-2019/caca.png", tmp);
    */
}


// getters


std::vector<Subcell> Divide::get_subcells(){
    return subcells;
}


int Divide::get_pas(){
    return pas;
}



int Divide::get_rows(){
    return rows;
}

int Divide::get_cols(){
    return cols;
}
