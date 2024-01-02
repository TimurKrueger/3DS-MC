/*
 * Project: Interactive ARAP
 * File:    main.cpp
 * Authors: Kilian Peis, �mer K�se, Natalie Adam, Timur Kr�ger
 */

#include "include/Visualizer.h"
#include "include/Mesh.h"
#include "include/Arap.h"
#include <iostream>

int main() {
	// Create an instance Visualizer class
	Visualizer visualizer("../Data/Cow/cow.ply");
	Mesh currentMesh = visualizer.getCurrentMesh();
	
	Arap arap(visualizer);

	// Launch the visualizer
	visualizer.launch();
};
