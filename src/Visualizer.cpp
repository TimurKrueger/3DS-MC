/*
 * Project: Interactive ARAP
 * File:    Visualizer.cpp
 * Authors: Kilian Peis, �mer K�se, Natalie Adam, Timur Kr�ger
*/

#include "../include/Visualizer.h"

Visualizer::Visualizer(const std::string& meshPath) 
    :
    currentMesh(meshPath), 
    m_arap(currentMesh),
    selectionFixedFaces(false) ,
    fixedMovement(false),
    movingVertex(false),
    movingVertexId(-1)
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
    handleMouseMove();
    viewer.launch();
}

std::map<int, bool> Visualizer::getFixedFaces() {
    return selectedFaces;
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

void Visualizer::handleMouseMove() {
    viewer.callback_mouse_move = [this](igl::opengl::glfw::Viewer& viewer, int button, int modifier) -> bool {
        if (movingVertex) {
            if (movingVertexId >= 0) {
                Eigen::Vector2f mousePosition = getMousePosition();
                Eigen::Vector3f mouseWorldPos;
                std::cout << "moving the thinf" << std::endl;
                if (igl::unproject_onto_mesh(mousePosition, viewer.core().view, viewer.core().proj,
                    viewer.core().viewport, currentMesh.getVertices(), currentMesh.getFaces(),
                    movingVertexId, mouseWorldPos)) {
                    int vertexId = currentMesh.getClosestVertexId(currentMesh.getFaces(), movingVertexId, mouseWorldPos);

                    //-------------
                    // edit this, used from old solution!!!
                    Eigen::Vector3f vertexPosition = {
                        (float)currentMesh.getVertices().row(vertexId).x(), (float)currentMesh.getVertices().row(vertexId).y(), (float)currentMesh.getVertices().row(vertexId).z()
                    };

                    Eigen::Vector3f projection = igl::project(vertexPosition, viewer.core().view, viewer.core().proj, viewer.core().viewport);
                    Eigen::Vector3f worldPosition = igl::unproject(Eigen::Vector3f(mousePosition.x(), mousePosition.y(), projection.z()),
                        viewer.core().view, viewer.core().proj, viewer.core().viewport);

                    //-------------


                    currentMesh.setVertexPos(vertexId, worldPosition.cast<double>());
                    updateMesh(currentMesh);
                }
            }
        }
        return false;
    };
}

void Visualizer::handleMouseDown() {
    viewer.callback_mouse_down = [this](igl::opengl::glfw::Viewer& viewer, int button, int modifier) -> bool {
        Eigen::Vector2f mousePosition = getMousePosition();
        //std::cout << selectionFixedFaces << std::endl;
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
        else if (movingVertex) {
           if (movingVertexId == -1) {
                Eigen::Vector2f mousePosition = getMousePosition();
                int faceId;
                Eigen::Vector3f mouseWorldPos;
                std::cout << "moving Vertex" << std::endl;
                if (igl::unproject_onto_mesh(mousePosition, viewer.core().view, viewer.core().proj,
                    viewer.core().viewport, currentMesh.getVertices(), currentMesh.getFaces(),
                    faceId, mouseWorldPos)) {
                    movingVertexId = faceId;
                }
            }
        }

        return false;
    };
}

void Visualizer::handleKeyDown() {
    viewer.callback_key_down = [this](igl::opengl::glfw::Viewer&, unsigned char key, int modifier) {
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
            case '3':
                movingVertex = true;
                return true;
            case 'R':
                std::cout << "r" << std::endl;
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
                m_arap.setFixedVertices(getFixedFaces());
                return true;
            case '2':
                fixedMovement = false;
                return true;
            case '3':
                movingVertex = false;
                movingVertexId = -1;
                return true;
        }
        return false;
    };
}