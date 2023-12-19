/*
 * Project: Interactive ARAP
 * File:    Visualizer.h
 * Authors: Kilian Peis, Ömer Köse, Natalie Adam, Timur Krüger
*/

#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <igl/opengl/glfw/Viewer.h>
#include <igl/unproject_onto_mesh.h>
#include "Mesh.h"

class Visualizer {
private:
    igl::opengl::glfw::Viewer viewer;
    Mesh currentMesh;
public:
    explicit Visualizer(const std::string& meshPath);
    ~Visualizer() = default;

    // Get current mesh
    Mesh getCurrentMesh();
    // Set the mesh for visualization
    void setMesh(const Mesh& mesh);
    // Update the mesh visualization
    void updateMesh(const Mesh& mesh);
    // Set the keyboard callback
    void setKeyboardCallback(const std::function<void(unsigned char, int)>& callback);
    // Set the mouse callback
    void setMouseCallback(const std::function<void(const Eigen::Vector2f&)>& callback);
    // Launch the viewer
    void launch();
};
#endif