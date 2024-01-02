/*
 * Project: Interactive ARAP
 * File:    Mesh.h
 * Authors: Kilian Peis, Ömer Köse, Natalie Adam, Timur Krüger
*/


#ifndef ARAP_H
#define ARAP_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "Mesh.h"
#include "Visualizer.h"

#include <queue>
#include <vector>
#include <algorithm>


class Arap
{
public:
    explicit Arap(Visualizer& visualizer);
private:
    void m_constructNeighborhood();
    // void m_updateWeightMatrix();
    void m_updateSparseWeightMatrix();
    void m_setSystemMatrix();
    void collectFixedVertices();
    void updateSystemMatrix(int movedVertex);
    void updateSystemMatrixRecursively(int vertexIndex, Eigen::SparseMatrix<double>& updatedSystemMatrix);
private:
    Mesh& m_mesh;
    Visualizer& m_visualizer;
    std::vector<int> fixedVertices;
    std::vector<std::vector<int>> m_neighbors;
    // Eigen::MatrixXd m_weightMatrix; // This will be updated per solve
    Eigen::SparseMatrix<double> m_weightMatrix; // This will be updated per solve
    Eigen::SparseMatrix<double, Eigen::RowMajor> m_systemMatrix;
};

#endif
