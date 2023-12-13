/*
 * Project: Interactive ARAP
 * File:    main.cpp
 * Authors: Kilian Peis, Ömer Köse, Natalie Adam, Timur Krüger
 */

#include "include/Visualizer.h"
#include "include/Mesh.h"

int main() {
    Visualizer visualizer("../Data/Armadillo/Armadillo.ply");
    visualizer.launch();
    
    return 0;
};
