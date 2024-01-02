/*
 * Project: Interactive ARAP
 * File:    Mesh.cpp
 * Authors: Kilian Peis, Ömer Köse, Natalie Adam, Timur Krüger
 */


#include "../include/Arap.h"

Arap::Arap(Mesh& mesh)
    :
    m_mesh(mesh)
{
    m_constructNeighborhood();
    // m_updateWeightMatrix();
    m_updateSparseWeightMatrix();
    m_setSystemMatrix();
    
    std::cout << m_weightMatrix.coeff(0, 0) << "\n";

    // for debugging
    /*
    for (int k = 0; k < m_systemMatrix.outerSize(); ++k) {
        for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(m_systemMatrix, k); it; ++it) {
            std::cout << "(" << it.row() << ", " << it.col() << ") = " << it.value() << "\n";
        }
    }
    */
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

/*
void Arap::m_updateWeightMatrix()
{
    m_weightMatrix = Eigen::MatrixXd::Zero(m_mesh.getNumVertices(), m_mesh.getNumVertices());
    const Eigen::MatrixXi& F = m_mesh.getFaces();
    const Eigen::MatrixXd& V = m_mesh.getVertices();

    for(int i = 0; i < F.rows(); ++i)
    {
     // Directed edges for cotangent computations (in CCT)
     Eigen::Vector3d e01 = V.row(F(i, 1)) - V.row(F(i, 0));
     Eigen::Vector3d e12 = V.row(F(i, 2)) - V.row(F(i, 1));
     Eigen::Vector3d e20 = V.row(F(i, 0)) - V.row(F(i, 2));
     
     // Computing Cotangents
     double c0 = (e01.dot(-e20)) / ((e01.cross(-e20)).norm());
     double c1 = (e12.dot(-e01)) / ((e12.cross(-e01)).norm());
     double c2 = (e20.dot(-e12)) / ((e20.cross(-e12)).norm());
     
     // Symmetrically load the weight matrix
     m_weightMatrix(F(i, 0), F(i, 1)) += 0.5 * c2;
     m_weightMatrix(F(i, 1), F(i, 0)) += 0.5 * c2;
     
     m_weightMatrix(F(i, 1), F(i, 2)) += 0.5 * c0;
     m_weightMatrix(F(i, 2), F(i, 1)) += 0.5 * c0;
     
     m_weightMatrix(F(i, 0), F(i, 2)) += 0.5 * c1;
     m_weightMatrix(F(i, 2), F(i, 0)) += 0.5 * c1;
    }

    // Set the diagonal weights to 1
    for(int i = 0; i < m_mesh.getNumVertices(); ++i)
    {
     m_weightMatrix(i, i) = 1.0;
    }
}
*/

void Arap::m_updateSparseWeightMatrix()
{
    const Eigen::MatrixXi& F = m_mesh.getFaces();
    const Eigen::MatrixXd& V = m_mesh.getVertices();
    
    typedef Eigen::Triplet<double> T;
    std::vector<T> triplets;
    m_weightMatrix = Eigen::SparseMatrix<double>(m_mesh.getNumVertices(), m_mesh.getNumVertices());
    
    for(int i = 0; i < F.rows(); ++i)
    {
        // Directed edges for cotangent computations (in CCT)
        Eigen::Vector3d e01 = V.row(F(i, 1)) - V.row(F(i, 0));
        Eigen::Vector3d e12 = V.row(F(i, 2)) - V.row(F(i, 1));
        Eigen::Vector3d e20 = V.row(F(i, 0)) - V.row(F(i, 2));
        
        // Computing Cotangents
        double c0 = (e01.dot(-e20)) / ((e01.cross(-e20)).norm());
        double c1 = (e12.dot(-e01)) / ((e12.cross(-e01)).norm());
        double c2 = (e20.dot(-e12)) / ((e20.cross(-e12)).norm());
        
        // Symmetrically load the weight matrix
        triplets.emplace_back(F(i, 0), F(i, 1), 0.5 * c2);
        triplets.emplace_back(F(i, 1), F(i, 0), 0.5 * c2);
        
        triplets.emplace_back(F(i, 1), F(i, 2), 0.5 * c0);
        triplets.emplace_back(F(i, 2), F(i, 1), 0.5 * c0);
        
        triplets.emplace_back(F(i, 0), F(i, 2), 0.5 * c1);
        triplets.emplace_back(F(i, 2), F(i, 0), 0.5 * c1);
    }
    
    // Set the diagonal weights to 1
    for(int i = 0; i < m_mesh.getNumVertices(); ++i)
    {
        triplets.emplace_back(i, i, 1.0);
    }
    
    m_weightMatrix.setFromTriplets(triplets.begin(), triplets.end());
}

void Arap::m_setSystemMatrix()
{
    m_systemMatrix.resize(m_weightMatrix.rows(), m_weightMatrix.cols());
    m_systemMatrix = -m_weightMatrix;

    // Add diagonal value
    for (int i = 0; i < m_weightMatrix.rows(); ++i)
    {
        m_systemMatrix.coeffRef(i, i) += -m_systemMatrix.row(i).sum();
    }
}

std::vector<Eigen::Matrix3d> Arap::m_computeRotations(Eigen::MatrixXd& V_deformed)
{
    const Eigen::MatrixXd& V = m_mesh.getVertices();
    std::vector<Eigen::Matrix3d> R(V.rows());

    for (int i = 0; i < V.rows(); ++i)
    {
        // Edges of vertices
        Eigen::MatrixXd P(3, m_neighbors[i].size());
        // Diagonal matrix with weights
        Eigen::MatrixXd D(m_neighbors[i].size(), m_neighbors[i].size());
        // Edges of deformed vertices
        Eigen::MatrixXd P_p(3, m_neighbors[i].size());

        for (int j = 0; j < m_neighbors[i].size(); ++j)
        {
            P.col(j) = V.row(i) - V.row(m_neighbors[i][j]);
            D(j, j) = m_weightMatrix.coeffRef(i, m_neighbors[i][j]);
            P_p.col(j) = V_deformed.row(i) - V_deformed.row(m_neighbors[i][j]);
        }
        
        // Covariance matrix: S = P * D * P'^T
        Eigen::MatrixXd S = P * D * P_p.transpose();

        // Perform SVD: S = U * sum(V^T)
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(S, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::MatrixXd U = svd.matrixU();
        Eigen::MatrixXd V = svd.matrixV();

        // Ensure determinant is positive
        if (U.determinant() < 0)
        {
            U.col(2) *= -1.0;
        }

        // R = V * U^T
        R[i] = V * U.transpose();
    }

    return R;
}