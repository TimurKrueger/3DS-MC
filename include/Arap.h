/*
 * Project: Interactive ARAP
 * File:    Arap.h
 * Authors: Kilian Peis, Ömer Köse, Natalie Adam, Timur Krüger
*/

#ifndef ARAP_H
#define ARAP_H

#include <Eigen/Core>
#include "Mesh.h"

class Arap {
private:
    Mesh& mesh;
    Eigen::SparseMatrix<double> systemMatrix;

public:
    explicit Arap(Mesh& mesh);

    void collectFixedVertices(const std::vector<int>& fixedVertices);
    void updateSystemMatrix();
};

#endif