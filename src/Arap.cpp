/*
 * Project: Interactive ARAP
 * File:    Mesh.cpp
 * Authors: Kilian Peis, ÷mer Kˆse, Natalie Adam, Timur Kr¸ger
 */


#include "../include/Arap.h"

Arap::Arap(Mesh& mesh)
    :
    m_mesh(mesh)
{
    m_constructNeighborhood();
}

void Arap::m_constructNeighborhood()
{
    m_neighbors.resize(m_mesh.getNumVertices());
    const Eigen::MatrixXi& F = m_mesh.getFaces();
    for(int i = 0; i < F.rows(); ++i)
    {
        int v1 = F(i, 0);
        int v2 = F(i, 1);
        int v3 = F(i, 2);
        
        // Add neighbor relations
        m_neighbors[v1].push_back(v2);
        m_neighbors[v1].push_back(v3);
        
        m_neighbors[v2].push_back(v1);
        m_neighbors[v2].push_back(v3);
        
        m_neighbors[v3].push_back(v1);
        m_neighbors[v3].push_back(v2);
    }
    
    // Remove the duplicates
    for(int i = 0; i < m_neighbors.size(); ++i)
    {
        std::sort(m_neighbors[i].begin(), m_neighbors[i].end());
        m_neighbors[i].erase(std::unique(m_neighbors[i].begin(), m_neighbors[i].end()), m_neighbors[i].end());
    }
}


void Arap::m_updateWeightMatrix()
{
    
}


