/*
 * Project: Interactive ARAP
 * File:    main.cpp
 * Authors: Kilian Peis, �mer K�se, Natalie Adam, Timur Kr�ger
 */

#include "include/Visualizer.h"
#include "include/Mesh.h"
#include "include/Arap.h"
#include <iostream>


#include <igl/cotmatrix.h>

int main() {
	// Create an instance Visualizer class
	Visualizer visualizer("../Data/armadillo/armadillo_500.off");
	Mesh currentMesh = visualizer.getCurrentMesh();
    
	Arap arap(currentMesh);
	Eigen::SparseMatrix<double> L_ours = arap.m_systemMatrix.sparseView();
    Eigen::SparseMatrix<double> L;
    igl::cotmatrix(currentMesh.getVertices(), currentMesh.getFaces(), L);
	
	std::cout << (L + L_ours).norm() << std::endl;
	// Launch the visualizer
	visualizer.launch();
};
