/*
 * Project: Interactive ARAP
 * File:    Mesh.h
 * Authors: Kilian Peis, Ömer Köse, Natalie Adam, Timur Krüger
 */

#include <igl/readPLY.h>

class Mesh {
public:
    explicit Mesh(const std::string&);
    ~Mesh() = default;

    // Vertices, colors and faces of the model
    Eigen::MatrixXd m_vertices{}, m_colors{};
    Eigen::MatrixXi m_faces{}; 
};