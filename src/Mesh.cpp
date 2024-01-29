/*
 * Project: Interactive ARAP
 * File:    Mesh.cpp
 * Authors: Kilian Peis, �mer K�se, Natalie Adam, Timur Kr�ger
 */

#include "../include/Mesh.h"
#include <igl/readPLY.h>
#include <igl/readOFF.h>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

Mesh::Mesh(const std::string& modelName) {
    if (hasExtension(modelName, "ply")) {
        igl::readPLY(modelName, vertices, faces);
        colors = Eigen::MatrixXd::Constant(faces.rows(), 3, 1);
        initColors = Eigen::MatrixXd::Constant(faces.rows(), 3, 1);
    }
    else if (hasExtension(modelName, "off")) {
        igl::readOFF(modelName, vertices, faces);
        colors = Eigen::MatrixXd::Constant(faces.rows(), 3, 1);
        initColors = Eigen::MatrixXd::Constant(faces.rows(), 3, 1);
    }
    else {
        std::cout << "Unknown or unsupported file format." << std::endl;
    }
}

void Mesh::load(const std::string& filename) {
    igl::readPLY(filename, vertices, faces);
}

void Mesh::applyForce(int vertexIndex, const Eigen::Vector3d& force) {
    if (vertexIndex >= 0 && vertexIndex < vertices.rows()) {
        vertices.row(vertexIndex) += force.transpose();
    }
}

void Mesh::setVertexPos(int vertexIndex, const Eigen::Vector3d& newPos) {
    if (vertexIndex >= 0 && vertexIndex < vertices.rows()) {
        vertices.row(vertexIndex) = newPos.transpose();
    }
}

void Mesh::setVertices(Eigen::MatrixXd matrix) {
    vertices = matrix;
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

const Eigen::MatrixXd& Mesh::getInitColors() const {
    return initColors;
}

void Mesh::setInitColors() {
    colors = initColors;
}

int Mesh::getNumVertices() const {
    return vertices.rows();
}

// Checks if loaded mesh is .ply or .off format
bool Mesh::hasExtension(const std::string& fileName, const std::string& ext) {
    size_t dotPos = fileName.find_last_of('.');

    if (dotPos == std::string::npos || dotPos == fileName.length() - 1) {
        return false;
    }

    std::string fileExt = fileName.substr(dotPos + 1);
    std::transform(fileExt.begin(), fileExt.end(), fileExt.begin(),
        [](unsigned char c) { return std::tolower(c); });

    return fileExt == ext;
}
