/*
 * Project: Interactive ARAP
 * File:    main.cpp
 * Authors: Kilian Peis, �mer K�se, Natalie Adam, Timur Kr�ger
 */

#include "include/Visualizer.h"
#include "include/Mesh.h"

int main() {
    Visualizer visualizer("../Data/Armadillo/Armadillo.ply");
    visualizer.launch();
    
    return 0;
};
