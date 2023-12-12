/*
 * Project: Interactive ARAP
 * File:    main.cpp
 * Authors: Kilian Peis, Ömer Köse, Natalie Adam, Timur Krüger
 */

#include <igl/opengl/glfw/Viewer.h>
#include <igl/unproject_onto_mesh.h>
#include "include/Mesh.h"

int main() {
    // Setup Viewer (TBD in instance of visualizer)
    igl::opengl::glfw::Viewer viewer;

    // Load Meshes
    Mesh mesh("../Data/Armadillo/Armadillo.ply");
    viewer.data().set_mesh(mesh.m_vertices, mesh.m_faces);
    viewer.data().set_colors(mesh.m_colors);
    viewer.launch();

    return 0;
};