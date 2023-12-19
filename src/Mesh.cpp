/*
 * Project: Interactive ARAP
 * File:    Mesh.cpp
 * Authors: Kilian Peis, Ömer Köse, Natalie Adam, Timur Krüger
 */

#include <igl/readPLY.h>
#include "../include/Mesh.h"
#include <Eigen/Geometry>
#include <Eigen/StdVector>

Mesh::Mesh(const std::string& modelName) {
    igl::readPLY(modelName, vertices, faces);
    colors = Eigen::MatrixXd::Constant(faces.rows(), 3, 1);
}

void Mesh::load(const std::string& filename) {
    igl::readPLY(filename, vertices, faces);
}

void Mesh::applyForce(int vertexIndex, const Eigen::Vector3d& force) {

    if (vertexIndex >= 0 && vertexIndex < vertices.rows()) {
        vertices.row(vertexIndex) += force.transpose();
    }
}

int Mesh::getClosestVertexId(const Eigen::MatrixXi& faces, int faceId, const Eigen::Vector3f& barycentricPosition) {
    int closestVertex = -1;
    float maxBarycentric = -1.0f;
    for (int i = 0; i < 3; ++i) {
        if (barycentricPosition[i] > maxBarycentric) {
            maxBarycentric = barycentricPosition[i];
            closestVertex = faces(faceId, i);
        }
    }
    return closestVertex;
}

const Eigen::MatrixXd& Mesh::getVertices() const {
    return vertices;
}

const Eigen::MatrixXi& Mesh::getFaces() const {
    return faces;
}

const Eigen::MatrixXd& Mesh::getColors() const {
    return colors;
}