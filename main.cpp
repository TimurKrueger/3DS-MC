/*
 * Project: Interactive ARAP
 * File:    main.cpp
 * Authors: Kilian Peis, �mer K�se, Natalie Adam, Timur Kr�ger
 */

#include <iostream>
#include "include/Visualizer.h"
#include "include/Mesh.h"
#include "include/Arap.h"
#include <igl/cotmatrix.h>

int main() {
	// Create an Visualizer class instance
	Visualizer visualizer("../Data/Cactus/cactus_small.off");
	Mesh currentMesh = visualizer.getCurrentMesh();
    
    Eigen::SparseMatrix<double> L;
    igl::cotmatrix(currentMesh.getVertices(), currentMesh.getFaces(), L);
    
    // Create an ARAP class instance
	Arap arap(currentMesh);
    Eigen::SparseMatrix<double, Eigen::RowMajor> ourLaplacian = arap.m_systemMatrix;
    Eigen::SparseMatrix<double, Eigen::ColMajor> ourLaplacianCol(ourLaplacian);
    
    Eigen::SparseMatrix<double> diff = L + ourLaplacianCol;
    std::cout << diff.norm() << "\n";

    // Launch the visualizer
	visualizer.launch();
};