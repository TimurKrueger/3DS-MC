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