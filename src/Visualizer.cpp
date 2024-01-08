/*
 * Project: Interactive ARAP
 * File:    Visualizer.cpp
 * Authors: Kilian Peis, Ömer Köse, Natalie Adam, Timur Krüger
*/

#include "../include/Visualizer.h"

Visualizer::Visualizer(const std::string& meshPath) 
    :
    currentMesh(meshPath), 
    selectionFixedFaces(false) ,
    fixedMovement(false)
{
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
    handleKeyDown();
    handleKeyRelease();
    handleMouseDown();
    viewer.launch();
}

Eigen::Vector2f Visualizer::getMousePosition()
{
    //since igl window coordinates start on bottom left and have minimum values of (0,0) we need to flip y coordinate to match the coordinate system
    int width, height;
    glfwGetWindowSize(viewer.window, &width, &height);
    
    int max_x_coordinate = width;
    int max_y_coordinate = height;

    int viewer_mouse_x = viewer.current_mouse_x;
    int viewer_mouse_y = max_y_coordinate - viewer.current_mouse_y;

    //std::cout << "First x y:  " << viewer.current_mouse_x << ", " << viewer.core().viewport(3) - (float)viewer.current_mouse_y << "  and window :  " << window_x << ", " << window_y << std::endl;
    return Eigen::Vector2f(viewer_mouse_x, viewer_mouse_y);
}

void Visualizer::handleMouseDown() {
    viewer.callback_mouse_down = [this](igl::opengl::glfw::Viewer& viewer, int button, int modifier) -> bool {
        Eigen::Vector2f mousePosition = getMousePosition();
        std::cout << selectionFixedFaces << std::endl;
        if (selectionFixedFaces) {
            int faceId;
            Eigen::Vector3f barycentricPosition;
            if (igl::unproject_onto_mesh(mousePosition, viewer.core().view, viewer.core().proj,
                viewer.core().viewport, currentMesh.getVertices(), currentMesh.getFaces(),
                faceId, barycentricPosition)) {

                bool selected = !selectedFaces[faceId];
                std::cout << "selected: "<<selected << std::endl;

                selectedFaces[faceId] = selected;

                if (selected) {
                    const Eigen::Vector3d selectedColor(255, 0, 0);

                    Eigen::MatrixXd& mutableColors = const_cast<Eigen::MatrixXd&>(currentMesh.getColors());  // Remove constness
                    Eigen::Block<Eigen::MatrixXd, 1, -1, false> faceColorBlock = mutableColors.row(faceId);
                    faceColorBlock = selectedColor.transpose();
                }
                else {
                    Eigen::MatrixXd& mutableColors = const_cast<Eigen::MatrixXd&>(currentMesh.getColors());  // Remove constness
                    Eigen::Block<Eigen::MatrixXd, 1, -1, false> faceColorBlock = mutableColors.row(faceId);
                    faceColorBlock = currentMesh.getInitColors().row(faceId).transpose();
                }
                viewer.data().set_colors(currentMesh.getColors());
            }
        }
        else if (fixedMovement) {
            int faceId;
            Eigen::Vector3f barycentricPosition;
            if (igl::unproject_onto_mesh(mousePosition, viewer.core().view, viewer.core().proj,
                viewer.core().viewport, currentMesh.getVertices(), currentMesh.getFaces(),
                faceId, barycentricPosition)) {
                Eigen::Vector3d force(0, -20, 0);

                int vertexId = currentMesh.getClosestVertexId(currentMesh.getFaces(), faceId, barycentricPosition);
                currentMesh.applyForce(vertexId, force);

                updateMesh(currentMesh);
                return true;
            }
        }

        return false;
    };
}

void Visualizer::handleKeyDown() {
    viewer.callback_key_down = [this](igl::opengl::glfw::Viewer&, unsigned char key, int modifier) {

        std::cout << "Pressed key" << std::endl;
        // Define the force vector and the target vertex index
        Eigen::Vector3d force(0, -20, 0); // Example force
        int target_vertex_index = 300; // Example vertex index

        switch (key) {
            case ' ':
                // Apply the force to the target vertex
                currentMesh.applyForce(target_vertex_index, force);

                // Update the visualization
                updateMesh(currentMesh);
                return true;
            case '1':
                selectionFixedFaces = true;
                return true;
            case '2':
                fixedMovement = true;
                return true;
            case 'r':
                selectedFaces.clear();
                selectionFixedFaces = false;
                viewer.data().set_colors(currentMesh.getInitColors());
                return true;
        }
        return false;
    };
}

void Visualizer::handleKeyRelease() {
    viewer.callback_key_up = [this](igl::opengl::glfw::Viewer&, unsigned char key, int modifier) {

        switch (key) {
            case '1':
                selectionFixedFaces = false;
                return true;
            case '2':
                fixedMovement = false;
                return true;
        }
        return false;
    };
}