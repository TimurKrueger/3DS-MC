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
#include "Arap.h"
#include <map>

class Visualizer {
private:
    igl::opengl::glfw::Viewer viewer;
    Mesh currentMesh;
    Arap m_arap;
    std::map<int, bool> selectedFaces;
    bool selectionFixedFaces;
    bool fixedMovement;
    bool movingVertex;
    int movingVertexId;
    
public:
    explicit Visualizer(const std::string& meshPath);
    ~Visualizer() = default;

    // Get current mesh
    Mesh getCurrentMesh();
    void handleImGUI();
    // Get the mouse Position in screen position
    Eigen::Vector2f getMousePosition();

    std::map<int, bool> getFixedFaces();
    // Set the mesh for visualization
    void setMesh(const Mesh& mesh);
    void setArap(const Arap& arap);
    // Update the mesh visualization
    void updateMesh(const Mesh& mesh);
    // Set the keyboard callback
    //void setKeyboardCallback(const std::function<void(unsigned char, int)>& callback);
    void handleKeyDown();
    void handleKeyRelease();
    // Set the mouse callback
    //void setMouseCallback(const std::function<void(const Eigen::Vector2f&)>& callback);
    void handleMouseDown();
    void handleMouseMove();
    // Launch the viewer
    void launch();
};
#endif