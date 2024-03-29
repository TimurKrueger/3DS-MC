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
#include <Eigen/SparseCholesky>
#include "Mesh.h"

#include <vector>
#include <unordered_set>
#include <algorithm>
#include <limits>

// TODO: Discuss about these constants
#define RIGIDITY_ENERGY_THRESHOLD 1e-6 // Not sure what this should be we should analyze the rigidity energy
#define MAX_ITERATIONS 10

class Arap
{
public:
    explicit Arap(Mesh& mesh);
    Arap& operator=(const Arap& other);
    Eigen::MatrixXd computeDeformation(int movedVertexId, const Eigen::Vector3d& movedVertexPos);
    std::vector<int> m_fixedVertices;
private:
    void m_constructNeighborhood();
    void m_updateSparseWeightMatrix();
    void m_setSystemMatrix();
    void updateSystemMatrix(int movedVertex);
    std::vector<Eigen::Matrix3d> m_computeRotations(Eigen::MatrixXd&);
    double m_computeRigidityEnergy(const Eigen::MatrixXd& V_deformed, const std::vector<Eigen::Matrix3d>& rotations);
    Eigen::MatrixXd m_computeRHS(const std::vector<Eigen::Matrix3d>& rotations, int movedVertexId, const Eigen::Vector3d& movedVertexPos); // TODO: Complete after the constraint stuff has been finished
    
    // Mesh is always referenced from ARAP class. Therefore, the referenced data of the mesh will always stay updated. This is important for some calculations that uses the previous mesh vertices as a reference.
    Mesh& m_mesh;
    // Topology is constant, so it can be safely computed once and stored.
    std::vector<std::vector<int>> m_neighbors;
    Eigen::SparseMatrix<double> m_weightMatrix;
    Eigen::SparseMatrix<double> m_systemMatrix;
public:
    void setFixedVertices(std::map<int, bool> fixedFaces);
};

#endif
