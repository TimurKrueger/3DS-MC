/*
 * Project: Interactive ARAP
 * File:    main.cpp
 * Authors: Kilian Peis, Ömer Köse, Natalie Adam, Timur Krüger
 */

#include "include/Visualizer.h"
#include "include/Mesh.h"
#include <iostream>;

int main() {
	// Define the force vector and the target vertex index
	Eigen::Vector3d force(0, -20, 0); // Example force
	int target_vertex_index = 300; // Example vertex index

	// Create an instance Visualizer class
	Visualizer visualizer("../Data/Armadillo/Armadillo.ply");
	Mesh currentMesh = visualizer.getCurrentMesh();

	// Define the keyboard callback to apply the force
	visualizer.setKeyboardCallback([&](unsigned char key, int modifier) {
		std::cout << "Pressed key" << std::endl;

		// Apply the force to the target vertex
		currentMesh.applyForce(target_vertex_index, force);

		// Update the visualization
		visualizer.updateMesh(currentMesh);
	});

	visualizer.setMouseCallback([&](const Eigen::Vector2f& mousePosition) {
		std::cout << "Pressed mouse at pos: " << mousePosition << std::endl;
	});

	// Launch the visualizer
	visualizer.launch();
};