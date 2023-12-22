/*
 * Project: Interactive ARAP
 * File:    Mesh.h
 * Authors: Kilian Peis, ÷mer Kˆse, Natalie Adam, Timur Kr¸ger
*/


#ifndef ARAP_H
#define ARAP_H

#include <Eigen/Core>
#include "Mesh.h"

#include <vector>
#include <algorithm>


class Arap
{
public:
    explicit Arap(Mesh& mesh);
private:
    void m_constructNeighborhood();
    void m_updateWeightMatrix();
private:
    Mesh& m_mesh;
    std::vector<std::vector<int>> m_neighbors;
    Eigen::MatrixXd m_weightMatrix; // This will be updated per solve
};

#endif
