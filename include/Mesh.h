/*
 * Project: Interactive ARAP
 * File:    Mesh.h
 * Authors: Kilian Peis, �mer K�se, Natalie Adam, Timur Kr�ger
*/

#ifndef MESH_H
#define MESH_H

#include <igl/readPLY.h>

class Mesh {
private:
    Eigen::MatrixXd vertices, colors; // Vertex matrix & Color matrix
    Eigen::MatrixXi faces; // Face matrix
    Eigen::MatrixXd initColors;

public:
    explicit Mesh(const std::string&);
    ~Mesh() = default;

    // Load mesh from a file
    void load(const std::string& filename);
    // Apply force to a vertex
    void applyForce(int vertexIndex, const Eigen::Vector3d& force);
    void setVertexPos(int vertexIndex, const Eigen::Vector3d& newPos);
    void setVertices(Eigen::MatrixXd matrix);

    int getClosestVertexId(const Eigen::MatrixXi& faces, int faceId, const Eigen::Vector3f& barycentricPosition);

    // Getter for vertices, faces and colors
    const Eigen::MatrixXd& getVertices() const;
    const Eigen::MatrixXd& getColors() const;
    const Eigen::MatrixXi& getFaces() const;
    const Eigen::MatrixXd& getInitColors() const;
    // Getter for statistics
    int getNumVertices() const;
};
#endif
