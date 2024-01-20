#include "path_planning_map/Divide.hpp"
#include <iostream>
#include <ros/ros.h>



// on implemente les fonctions




Divide::Divide(cv::Mat map_image_original, int pas){

    /**
     * @brief Construct a new Divide:: Divide object
     * 
     * @param map_image_original : la map obtenu via openCV
     * @param pas : la taille des subcells
     * 
     */

    ROS_INFO("INITIALISATION DU TABLEAU DE SUBCELLS");
    this->map_image_original = map_image_original;
    this->pas = pas;

    // on recupere les dimensions de la map
    this->rows = map_image_original.rows;
    this->cols = map_image_original.cols;

 
    this->nb_rows_discrete = this->rows / pas;
    this->nb_cols_discrete = this->cols / pas;

    // creation d'un tableau 2D pour stocker les subcells
    this->subcells = std::vector<std::vector<Subcell>>(this->nb_rows_discrete, 
                                                       std::vector<Subcell>(this->nb_cols_discrete, Subcell(false, pas, 0, 0, -1)));
    //que les free : on initie un tableau null
    this->subcells_free = std::vector<Subcell*>(0, nullptr);
    


    this->nb_free_nodes = 0;

}


int Divide::divide_map(int marge_for_voisins){

    /**
     * @brief On divise , cree subcell, leur attribue un bool et on stocke dans un tableau
     * 
     */

    ROS_INFO("DIVISION DE LA MAP EN SOUS CELLULES");


    // on parcourt les ROI de la map, de taille pas*pas
    int compteur_subcells = 0;

    int id ;
    int nodes_libres = 0;
    for (int r = 0; r < rows; r+=pas) {
        for (int c = 0; c < cols; c+=pas) {

            // on verifie que la subcell ne depasse pas de la map
            if (r+pas <= rows and c+pas <=cols){
                compteur_subcells++;
                cv::Mat sous_rectangle = map_image_original(cv::Rect(c, r, pas, pas));
                bool is_occupied = detect_subcells(sous_rectangle);

       
                // on range dans le tab 2D
                int index_row = r/pas;
                int index_col = c/pas;

           
                if (not is_occupied){
                    id = nodes_libres;
                    nodes_libres++;
                }

                else{
                    id = -1;
                }

                // on cree la subcell
                
                // on l'ajoute dans le tableau 2D
                subcells[index_row][index_col] = Subcell(is_occupied, pas, c, r, id);

                
              
            }
            
        }
      
    }


    //pour chaque subcell libre de SUBCELLS, on ajoute ses voisins adjacents
    ROS_INFO("il y a %d subcells libres", nodes_libres);
    this->nb_free_nodes = nodes_libres;
    ROS_INFO("DIVISION TERMINEE =======> Calcul des voisins adjacents ...");

    for (int i = 0; i < subcells.size(); i++) {
        for (int j = 0; j < subcells[i].size(); j++) {
            if (not subcells[i][j].get_is_occupied()){                
                subcells[i][j].add_voisin_adjacent(&subcells,marge_for_voisins);
            }
        }
    }

    ROS_INFO("CHAQUE SUBCELL A SES VOISINS ADJACENTS !");
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
    return false;

}



void Divide::display_subcells(std::string path_folder){
    // Est appelée apres DIvide::Diide() donc elle les voisins ont déja été calculés.

    // la fenetre est séparée en 2 : a gauche, la map d'origine, a droite , la map avec les subcells avec un grillage
    cv::Mat map_image = map_image_original.clone();
    cv::cvtColor(map_image, map_image, cv::COLOR_GRAY2RGB);

    // on creer une image RGB
    cv::Mat map_image_subcells = map_image_original.clone();
    cv::cvtColor(map_image_subcells, map_image_subcells, cv::COLOR_GRAY2RGB);

    //on cree une image noir ou on ne metra que les cases libres en blancs sur fond noir
    cv::Mat map_free_subcells = cv::Mat::zeros(rows, cols, CV_8UC3);

  


    // D'abord les cellules occupés : on les trace en rouge
    for (int i = 0; i < subcells.size(); i++) {
        for (int j = 0; j < subcells[i].size(); j++) {
            // on recupere les coordonnées de la subcell
            int x = subcells[i][j].get_x();
            int y = subcells[i][j].get_y();
            int size = subcells[i][j].get_size();

            if (subcells[i][j].get_is_occupied())
            {
                cv::rectangle(map_image_subcells, cv::Point(x, y), cv::Point(x+size, y+size), cv::Scalar(0, 0, 255), 1); //bleu
            }

             if (not subcells[i][j].get_is_occupied() )
            {
                // 1 signifie rectangle vide, -1 signifie rectangle plein
                cv::rectangle(map_image_subcells, cv::Point(x, y), cv::Point(x+size, y+size), cv::Scalar(255, 0, 0), 1); //rouge

             
            }
          
        }
    }

    cv::rectangle(map_image_subcells, cv::Point(0, 0), cv::Point(cols, rows), cv::Scalar(0, 255, 0), 1); //vert

    map_free_subcells = filter_free_subcells();
    cv::rectangle(map_free_subcells, cv::Point(0, 0), cv::Point(cols, rows), cv::Scalar(0, 255, 0), 1); //vert

    cv::rectangle(map_image, cv::Point(0, 0), cv::Point(cols, rows), cv::Scalar(0, 255, 0), 1); //vert

    // on concatene les 3 images
    cv::Mat map_image_concat;
    cv::Mat map_2_images;


    cv::hconcat(map_image, map_image_subcells, map_image_concat);
    cv::hconcat(map_image_concat, map_free_subcells, map_image_concat);
    cv::hconcat(map_image, map_free_subcells, map_2_images);
    
    //  On ajoute pour une subcell libre au hasard ces voisins via des carrés de couleur 
    // random entre 0 et this->nb_rows_discrete
    
    // on parcoure la grille de subcells et on prend la premiere subcell libre
    // on recupere ses voisins adjacents

    

    // Image test voisins
    cv::Mat test_voisins = map_free_subcells.clone();

    //on cherche la n-ieme subcell libre
    int n=51;
    int i = 0;
    int j = 0;
    int compteur = 0;
    while (compteur < n){
        if (not subcells[i][j].get_is_occupied()){
            compteur++;
        }
        j++;
        if (j == subcells[i].size()){
            j = 0;
            i++;
        }
    }


    //on le colorie en magenta
    int x = subcells[i][j].get_x();
    int y = subcells[i][j].get_y();
    int size = subcells[i][j].get_size();
    cv::rectangle(test_voisins, cv::Point(x, y), cv::Point(x+size, y+size), cv::Scalar(255, 0, 255), -1);

    // on recupere ses voisins
    std::vector<Subcell*>* voisins = subcells[i][j].get_voisins();

    //colorie en vert les voisins
    for (int k = 0; k < voisins->size(); k++) {
        int x = voisins->at(k)->get_x();
        int y = voisins->at(k)->get_y();
        int size = voisins->at(k)->get_size();
        cv::rectangle(test_voisins, cv::Point(x, y), cv::Point(x+size, y+size), cv::Scalar(187, 255, 0), -1);
    }
  
    int nb_voisins_adjacents = voisins->size();
    

    // on enregistre
    cv::imwrite(path_folder+"occupancy_grid_discretized.png", map_2_images);
    cv::imwrite(path_folder + "map_discrete.png", map_free_subcells);
    cv::imwrite(path_folder+"compare.png", map_image_concat);
    cv::imwrite(path_folder+"test_voisins.png", test_voisins);

    

}



int Divide::convert_from_meters_to_free_subcells(float x_meters, float y_meters, int* x_subcells, int* y_subcells, float resolution, double* origin_coin_bas_gauche,bool debug_draw){
    /**
     * @brief Etant donné le tableau de subcells 2d, convertis les coordonnées en mètres en coordonnées en subcells libres
     * 
     * @param x_meters : coordonnée en x en metres dans le repere map
     * @param y_meters : coordonnée en y en metres dans le repere map
     * @param x_subcells : colonne de la subcell libre dans le tableau 2D de subcells
     * @param y_subcells : ligne de la subcell libre dans le tableau 2D de subcells
     * @param resolution : resolution de la map
    * @param origin_coin_bas_gauche : coordonnées du coin bas gauche de la map dans le repere map donné par YML
    * @param debug_draw : si true, on dessine les coordonnées du point dans l'image
    * 
    * @return 1 si tout s'est bien passé, -1 si la subcell est un mur
    */

    // etape 1 : on cherche les coordonnées du points dans le repère image
    // OT = OA + MA  + MT avec O (coin haut gauche de l'image) et M (centre du repere map) et T coordonnées du point dans le repere map
    // A point bas gauche de l'image

    double* OA = new double[2]; // OA vaut normalement (0, -200)
    OA[1] = -this->rows*resolution;
    OA[0] = 0;
    
    
    double* AM = new double[2]; // AM vaut normalement (100, 100 )
    AM[0] = - origin_coin_bas_gauche[0];
    AM[1] = - origin_coin_bas_gauche[1];

    
    double* MT = new double[2]; // MT vaut (x_meters, y_meters)
    MT[0] = x_meters;
    MT[1] = y_meters;

    double* OT = new double[2];
    OT[0] = OA[0] + AM[0] + MT[0];
    OT[1] = OA[1] + AM[1] + MT[1];

    // OT represente donc les coordonnées en meters du points recherchés dans le referentiel map, O etant le coin haut gauche de l'image

    //matrice de rotation autour de l'axe x de 90°
    // | 1  0 |
    // | 0 -1 |

    //on applique la rotation a OT
    double* OT_rotated = new double[2];
    OT_rotated[0] = OT[0];
    OT_rotated[1] = -OT[1];


    // ici, on a les coordonnées en pixels du point dans le repere image
    *x_subcells = OT_rotated[0] / resolution;
    *y_subcells = OT_rotated[1] / resolution;

    if (debug_draw){
        ROS_INFO("pixel_line : %d, pixel_col : %d", *y_subcells, *x_subcells);
    }


    // on divise par le pas pour avoir les coordonnées en subcells
    *x_subcells = *x_subcells / this->pas;
    *y_subcells = *y_subcells / this->pas;


    if (debug_draw){
        ROS_INFO("ligne : %d, colonne : %d", *y_subcells, *x_subcells);
    }
    


    // on verifie que les coordonnées sont bien dans le tableau
    if (*x_subcells > this->nb_cols_discrete || *y_subcells > this->nb_rows_discrete){
        ROS_ERROR("ERREUR : les coordonnées sont en dehors du tableau");
        exit(0);
    }


    if (debug_draw){
      
    
        cv::Mat test_convert = map_image_original.clone();
        cv::cvtColor(test_convert, test_convert, cv::COLOR_GRAY2RGB);

        test_convert = filter_free_subcells();

        
        Subcell* subcell = &(subcells[*y_subcells][*x_subcells]);
        int x = subcell->get_x();
        int y = subcell->get_y();
        int size = subcell->get_size();
        cv::rectangle(test_convert, cv::Point(x, y), cv::Point(x+size, y+size), cv::Scalar(255, 0, 255), -1);

        // on trace les coordonnées en metres du point x_meters, y_meters
        cv::Point point(OT_rotated[0]/resolution, OT_rotated[1]/resolution);
        
        test_convert.at<cv::Vec3b>(point) = cv::Vec3b(255, 255, 255);
        
        cv::rectangle(test_convert, cv::Point(0, 0), cv::Point(cols, rows), cv::Scalar(0, 255, 0), 1); //vert

        cv::imwrite("/home/spi-2019/test.png", test_convert);
        ROS_INFO("image enregistrée dans /home/spi-2019/test.png");
    

    }

    // on regarde l'etat de la subcell : mur ou libre
    if (subcells[*y_subcells][*x_subcells].get_is_occupied()){
        ROS_WARN(" ===== ATTENTION : la subcell est un mur =====  ");
        return -1;
    }

    return 1;


}

void Divide::display_subcell_state(std::vector<int> path, std::string path_folder){
    /**
     * @brief Permet d'afficher les subcells avec leur etat (libre ou pas)
     * 
     */
    

    // creer une image : pour l'instant noir

    cv::Mat draw = cv::Mat::zeros(rows, cols, CV_8UC3);

    // on parcourt les subcells
    for (int i=0;i<subcells.size();i++){
        for (int j=0;j<subcells[i].size();j++){

            // on recupere les coordonnées de la subcell
            int x = subcells[i][j].get_x();
            int y = subcells[i][j].get_y();
            int size = subcells[i][j].get_size();

            if (subcells[i][j].get_is_occupied()){
                cv::rectangle(draw, cv::Point(x, y), cv::Point(x+size, y+size), cv::Scalar(0, 0, 0), -1); //noit
                
            }
            else{
                cv::rectangle(draw, cv::Point(x, y), cv::Point(x+size, y+size), cv::Scalar(255, 255, 255), -1); //blanc
                cv::rectangle(draw, cv::Point(x, y), cv::Point(x+size, y+size), cv::Scalar(0, 255, 0), 1); //vert
            }

            //tracer du path en mauve
            for (int k = 0; k < path.size(); k++) {


                if (subcells[i][j].get_id() == path[k]){

                    //begin
                    if (k==0){
                        cv::rectangle(draw, cv::Point(x, y), cv::Point(x+size, y+size), cv::Scalar(0, 0, 255), -1); //blue
                    }

                    else if (k==path.size()-1){ //end
                        cv::rectangle(draw, cv::Point(x, y), cv::Point(x+size, y+size), cv::Scalar(134, 0, 0), -1); //red
                    }
                    else{
                        //yellow
                        cv::rectangle(draw, cv::Point(x, y), cv::Point(x+size, y+size), cv::Scalar(0, 255, 255), -1); //yellow
                    }

                    break;  

                }
             

            }
         

        }
           
    }



    //save
    cv::imwrite(path_folder+"path_dijsktra.png", draw);

    
}




void Divide::get_rid_inconsitencies(){
    // affiche this->map_image_original sous forme de tab dans le terminal
    
    ROS_INFO("Permet d'avoir des valeurs binaires sur la map");
    ROS_INFO("%d lignes et %d colonnes", this->rows, this->cols);
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
    
}




void Divide::build_graph_free_subcells(){
    /**
     * @brief Permet de construire le graph des subcells libres pour l'algorithme de dijsktra
     * ATTENTION !! Chaque subcell libre a une liste de pointeurs vers ses voisins adjacents ... qui sont situés dans le tableau 2D de subcells !!
     * Ce n'est pas ce qu'on veut, on veut que chaque subcell libre ait une liste de pointeurs vers ses voisins adjacents ... qui sont situés dans le tableau 1D de subcells_free !!
     */

    // remplissage du tableau 1D subcells_free qui est un tableau de pointeurs vers des subcells libres de subcells[][]
    ROS_INFO("CONSTRUCTION DU GRAPH DES SUBCELLS LIBRES ... ");

    for (int i = 0;i<subcells.size();i++){
        for (int j = 0;j<subcells[i].size();j++){
            if (not subcells[i][j].get_is_occupied()){
                this->subcells_free.push_back(&(subcells[i][j]));
            }
        }
    }

    //il faut que le pointeur pointe vers une subcell du tableau 1D subcells_free
    ROS_INFO("CONSTRUCTION TERMINEE : il y a %d adresses de subcells dans subcells_free", subcells_free.size());
}



cv::Mat Divide::filter_free_subcells(){

    /**
     * @brief Permet de filtrer les subcells libres, et l'enregistrer sur fond noir dans une image
     * 
     */

    // cree une image
    cv::Mat only_free = cv::Mat::zeros(rows, cols, CV_8UC3);

    for (int i = 0; i < subcells.size(); i++) {
        
        for (int j = 0; j < subcells[i].size(); j++) {
            int x = subcells[i][j].get_x();
            int y = subcells[i][j].get_y();
            int size = subcells[i][j].get_size();

            if (not subcells[i][j].get_is_occupied()){
  
                cv::rectangle(only_free, cv::Point(x, y), cv::Point(x+size, y+size), cv::Scalar(255, 255, 255), -1);
                cv::rectangle(only_free, cv::Point(x, y), cv::Point(x+size, y+size), cv::Scalar(0, 255, 0), 1);
            }
           
        }
    }

    // on enregistre
    return only_free;
}




// getters


std::vector<std::vector<Subcell>> Divide::get_subcells(){
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


int Divide::get_nb_rows_discrete(){
    return nb_rows_discrete;
}

int Divide::get_nb_cols_discrete(){
    return nb_cols_discrete;
}

int Divide::get_nb_free_nodes(){
    return nb_free_nodes;
}

