/*
 * Project: Interactive ARAP
 * File:    Mesh.h
 * Authors: Kilian Peis, Ömer Köse, Natalie Adam, Timur Krüger
*/

#ifndef MESH_H
#define MESH_H

#include <igl/readPLY.h>

class Mesh {
private:
    Eigen::MatrixXd vertices, colors; // Vertex matrix & Color matrix
    Eigen::MatrixXi faces; // Face matrix

public:
    explicit Mesh(const std::string&);
    ~Mesh() = default;

    // Load mesh from a file
    void load(const std::string& filename);
    // Apply force to a vertex
    void applyForce(int vertexIndex, const Eigen::Vector3d& force);

    // Getter for vertices, faces and colors
    const Eigen::MatrixXd& getVertices() const;
    const Eigen::MatrixXd& getColors() const;
    const Eigen::MatrixXi& getFaces() const;
    // Getter for statistics
    int getNumVertices() const;
};
#endif
