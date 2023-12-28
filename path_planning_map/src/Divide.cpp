#include "path_planning_map/Divide.hpp"

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


void Divide::divide_map(){

    /**
     * @brief On divise et on stocke les subcells dans un tableau
     * 
     */


    // on parcourt les ROI de la map, de taille pas*pas

    for (int r = 0; r < rows; r+=pas) {
        for (int c = 0; c < cols; c+=pas) {

            // on verifie que la subcell ne depasse pas de la map
            if (r + pas< rows && c + pas < cols){
                cv::Mat sous_rectangle = map_image_original(cv::Rect(c, r, pas, pas));
                bool is_occupied = detect_subcells(sous_rectangle);
                subcells.push_back(Subcell(is_occupied, pas, c, r, 999));
             
            }
            
        }
    }


    

}





bool Divide::detect_subcells(cv::Mat sous_rectangle){

    /**
     * @brief Etant donné une subcell donné, on regarde si il y a au moins un pixel noir
     * Si oui, on la considère comme une subcell occupée
     * 
     * @return true si la subcell est occupée, false sinon
     */

    
    // on parcourt la sous matrice

    for (int r = 0; r < sous_rectangle.rows; r++) {
        for (int c = 0; c < sous_rectangle.cols; c++) {
            int8_t map_value = sous_rectangle.at<uchar>(r, c);

            // Si 0, alors c'est noir, donc c occupé
            if (map_value == 0) {
                return true;
            }
        }
    }
    return false;

}



void Divide::display_subcells(){

    // affiche sous forme d'image et enregistre openCV fenetre AVEC GRILLE

    ROS_INFO("display_subcells");
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
