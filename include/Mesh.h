/*
 * Project: Interactive ARAP
 * File:    Mesh.h
 * Authors: Kilian Peis, …mer Kšse, Natalie Adam, Timur Kruger
*/

#ifndef MESH_H
#define MESH_H

#include <igl/readPLY.h>

class Mesh {
public:
    explicit Mesh(const std::string&);
    ~Mesh() = default;

    // Vertices, colors and faces of the model
    Eigen::MatrixXd m_vertices{}, m_colors{};
    Eigen::MatrixXi m_faces{};
};

#endif


