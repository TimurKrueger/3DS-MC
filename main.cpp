/*
 * Project: Interactive ARAP
 * File:    main.cpp
 * Authors: Kilian Peis, �mer K�se, Natalie Adam, Timur Kr�ger
 */

#include <iostream>
#include "include/Visualizer.h"
#include "include/Mesh.h"

int main() {
	// Create an Visualizer class instance
	Visualizer visualizer("../Data/Knight/knight.off");
	Mesh currentMesh = visualizer.getCurrentMesh();

    // Launch the visualizer
	visualizer.launch();
};