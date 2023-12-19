/*
 * Project: Interactive ARAP
 * File:    main.cpp
 * Authors: Kilian Peis, Ömer Köse, Natalie Adam, Timur Krüger
 */

#include "include/Visualizer.h"
#include "include/Mesh.h"
#include <iostream>;

int main() {

	// Create an instance Visualizer class
	Visualizer visualizer("../Data/Cow/cow.ply");
	Mesh currentMesh = visualizer.getCurrentMesh();

	// Launch the visualizer
	visualizer.launch();
};