/*
 * Project: Interactive ARAP
 * File:    Visualizer.cpp
 * Authors: Kilian Peis, Ömer Köse, Natalie Adam, Timur Krüger
*/

#include "../include/Visualizer.h"

Visualizer::Visualizer(const std::string& meshPath) :currentMesh(meshPath) {
    viewer.data().set_mesh(currentMesh.getVertices(), currentMesh.getFaces());
    viewer.data().set_colors(currentMesh.getColors());
}

Mesh Visualizer::getCurrentMesh() {
    return currentMesh;
}

void Visualizer::setMesh(const Mesh& mesh) {
    currentMesh = mesh;
    viewer.data().set_mesh(currentMesh.getVertices(), currentMesh.getFaces());
}

void Visualizer::updateMesh(const Mesh& mesh) {
    currentMesh = mesh;
    viewer.data().set_vertices(currentMesh.getVertices());
    viewer.data().compute_normals();
}

void Visualizer::launch() {
    viewer.launch();
}

void Visualizer::setKeyboardCallback(const std::function<void(unsigned char, int)>& callback) {
    viewer.callback_key_down = [callback](igl::opengl::glfw::Viewer&, unsigned char key, int modifier) {
        callback(key, modifier);
        return false;
    };
}

void Visualizer::setMouseCallback(const std::function<void(const Eigen::Vector2f&)>& callback) {
    viewer.callback_mouse_down = [this, callback](igl::opengl::glfw::Viewer& viewer, int button, int modifier) -> bool {
        Eigen::Vector2f mousePosition(viewer.current_mouse_x, viewer.current_mouse_y);

        int faceId;
        Eigen::Vector3f barycentricPosition;
        if (igl::unproject_onto_mesh(mousePosition, viewer.core().view, viewer.core().proj,
            viewer.core().viewport, currentMesh.getVertices(), currentMesh.getFaces(),
            faceId, barycentricPosition)) {
            Eigen::Vector3d force(0, -20, 0);

            int vertexId = currentMesh.getClosestVertexId(currentMesh.getFaces(), faceId, barycentricPosition);
            currentMesh.applyForce(vertexId, force);
            
            updateMesh(currentMesh);
            callback(mousePosition);
        }

        return false;
    };
}