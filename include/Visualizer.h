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

class Visualizer
{
public:
    explicit Visualizer(const std::string& meshPath);
    ~Visualizer() = default;
    
    void setMesh(const std::string& meshPath);
    void setWireframeLineMode(bool wireframe);
    void launch();
private:
    igl::opengl::glfw::Viewer m_viewer;
    Mesh m_mesh; // For now, we will have only one mesh
};

#endif

