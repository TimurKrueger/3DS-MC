/*
 * Project: Interactive ARAP
 * File:    Visualizer.cpp
 * Authors: Kilian Peis, Ömer Köse, Natalie Adam, Timur Krüger
*/

#include "../include/Visualizer.h"
#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <../../../lib/ImGuiFileDialog/ImGuiFileDialog.h>

Visualizer::Visualizer(const std::string& meshPath)
	:
	currentMesh(meshPath),
	m_arap(currentMesh),
	selectionFixedFaces(false),
	fixedMovement(false),
	movingVertex(false),
	movingVertexId(-1),
	mouseClicked(false)
{
	viewer.data().set_mesh(currentMesh.getVertices(), currentMesh.getFaces());
	viewer.data().set_colors(currentMesh.getColors());
}

Mesh Visualizer::getCurrentMesh() {
	return currentMesh;
}

void Visualizer::setMesh(const Mesh& mesh) {

	viewer.data().clear();
	currentMesh = mesh;
	viewer.data().set_mesh(currentMesh.getVertices(), currentMesh.getFaces());
	viewer.core().align_camera_center(currentMesh.getVertices(), currentMesh.getFaces());
}

void Visualizer::updateMesh(const Mesh& mesh) {
	currentMesh = mesh;
	viewer.data().set_vertices(currentMesh.getVertices());
	viewer.data().compute_normals();
}

igl::opengl::glfw::Viewer Visualizer::getViewer() {
	return viewer;
}

void Visualizer::launch() {
	handleKeyDown();
	handleKeyRelease();
	handleMouseDown();
	handleMouseMove();
	handleImGUI();
}
void Visualizer::handleImGUI() {
	igl::opengl::glfw::imgui::ImGuiPlugin plugin;
	viewer.plugins.push_back(&plugin);
	igl::opengl::glfw::imgui::ImGuiMenu menu;
	plugin.widgets.push_back(&menu);
	
	menu.callback_draw_viewer_menu = [&]()
	{
		ImVec2 menuWindowSize = ImVec2(120, 100);
		ImGui::SetNextWindowSize(menuWindowSize, ImGuiCond_FirstUseEver);

		if (ImGui::Begin("Menu", nullptr, ImGuiWindowFlags_NoResize)) // Optional: Use ImGuiWindowFlags_NoResize to disable manual resizing
		{
			if (ImGui::CollapsingHeader("Mesh", ImGuiTreeNodeFlags_DefaultOpen))
			{
				if (ImGui::Button("Load Mesh", ImVec2(-1, 0)))
				{
					IGFD::FileDialogConfig config;
					config.path = "../Data";
					ImGuiFileDialog::Instance()->OpenDialog("ChooseFileDlgKey", "Choose Mesh File", ".off,.ply", config);
				}
			}

			ImVec2 menuWindowSize = ImVec2(600, 450);
			ImGui::SetNextWindowSize(menuWindowSize, ImGuiCond_FirstUseEver);

			if (ImGuiFileDialog::Instance()->Display("ChooseFileDlgKey"))
			{
			
				if (ImGuiFileDialog::Instance()->IsOk())
				{
					std::string filePathName = ImGuiFileDialog::Instance()->GetFilePathName();
					std::string filePath = ImGuiFileDialog::Instance()->GetCurrentPath();
					std::cout << filePathName << std::endl;
					Mesh newMesh(filePathName);
					setMesh(newMesh);
				}			

				ImGuiFileDialog::Instance()->Close();
			}

			ImGui::End();
		}
	};

	viewer.launch();
}

std::map<int, bool> Visualizer::getFixedFaces() {
	return selectedFaces;
}

Eigen::Vector2f Visualizer::getMousePosition()
{
	// Since igl window coordinates start on bottom left and have minimum values of (0,0) we need to flip y coordinate to match the coordinate system
	int width, height;
	glfwGetWindowSize(viewer.window, &width, &height);

	int max_x_coordinate = width;
	int max_y_coordinate = height;

	int viewer_mouse_x = viewer.current_mouse_x;
	int viewer_mouse_y = max_y_coordinate - viewer.current_mouse_y;

	return Eigen::Vector2f(viewer_mouse_x, viewer_mouse_y);
}

void Visualizer::handleMouseMove() {
	viewer.callback_mouse_move = [this](igl::opengl::glfw::Viewer& viewer, int button, int modifier) -> bool {
		if (movingVertex) {
			if (movingVertexId >= 0) {
				Eigen::Vector2f mousePosition = getMousePosition();
				Eigen::Vector3f mouseWorldPos;
				int vertexId = movingVertexId;

				if (igl::unproject_onto_mesh(mousePosition, viewer.core().view, viewer.core().proj,
					viewer.core().viewport, currentMesh.getVertices(), currentMesh.getFaces(),
					vertexId, mouseWorldPos)) {

					// Mark: Used from old years solutioon
					Eigen::Vector3f vertexPosition = {
						(float)currentMesh.getVertices().row(movingVertexId).x(), (float)currentMesh.getVertices().row(movingVertexId).y(), (float)currentMesh.getVertices().row(movingVertexId).z()
					};
					std::cout << movingVertexId << std::endl;
					Eigen::Vector3f projection = igl::project(vertexPosition, viewer.core().view, viewer.core().proj, viewer.core().viewport);
					Eigen::Vector3f worldPosition = igl::unproject(Eigen::Vector3f(mousePosition.x(), mousePosition.y(), projection.z()),
						viewer.core().view, viewer.core().proj, viewer.core().viewport);
					//----------

					currentMesh.setVertexPos(movingVertexId, worldPosition.cast<double>());
					Eigen::MatrixXd matrix = m_arap.computeDeformation(movingVertexId);
					currentMesh.setVertices(matrix);
					updateMesh(currentMesh);
					return true;
				}
			}
		}
		else if (selectionFixedFaces) {
			if (mouseClicked) {
				Eigen::Vector2f mousePosition = getMousePosition();
				int faceId;
				Eigen::Vector3f barycentricPosition;
				std::cout << "size of selectedFaces in moving: " << selectedFaces.size() << std::endl;
				std::cout << "size of m_arap.selectedFaces in moving: " << m_arap.m_fixedVertices.size() << std::endl;

				if (igl::unproject_onto_mesh(mousePosition, viewer.core().view, viewer.core().proj,
					viewer.core().viewport, currentMesh.getVertices(), currentMesh.getFaces(),
					faceId, barycentricPosition)) {
					bool selected = !selectedFaces[faceId];
					selectedFaces[faceId] = selected;
					const Eigen::Vector3d selectedColor(255, 0, 0);
					Eigen::MatrixXd& mutableColors = const_cast<Eigen::MatrixXd&>(currentMesh.getColors());
					Eigen::Block<Eigen::MatrixXd, 1, -1, false> faceColorBlock = mutableColors.row(faceId);
					faceColorBlock = selectedColor.transpose();
					viewer.data().set_colors(currentMesh.getColors());
				}
				return true;
			}
		}

		return false;
	};
}

void Visualizer::handleMouseDown() {
	viewer.callback_mouse_down = [this](igl::opengl::glfw::Viewer& viewer, int button, int modifier) -> bool {
		Eigen::Vector2f mousePosition = getMousePosition();
		mouseClicked = true;
		if (selectionFixedFaces) {
			int faceId;
			Eigen::Vector3f barycentricPosition;

			if (igl::unproject_onto_mesh(mousePosition, viewer.core().view, viewer.core().proj,
				viewer.core().viewport, currentMesh.getVertices(), currentMesh.getFaces(),
				faceId, barycentricPosition)) {

				bool selected = !selectedFaces[faceId];
				selectedFaces[faceId] = selected;

				if (selected) {
					const Eigen::Vector3d selectedColor(255, 0, 0);
					Eigen::MatrixXd& mutableColors = const_cast<Eigen::MatrixXd&>(currentMesh.getColors());
					Eigen::Block<Eigen::MatrixXd, 1, -1, false> faceColorBlock = mutableColors.row(faceId);
					faceColorBlock = selectedColor.transpose();
				}
				else {
					Eigen::MatrixXd& mutableColors = const_cast<Eigen::MatrixXd&>(currentMesh.getColors());
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

				Eigen::MatrixXd matrix = m_arap.computeDeformation(movingVertexId);
				currentMesh.setVertices(matrix);
				updateMesh(currentMesh);
				return true;
			}
		}
		else if (movingVertex) {
			if (movingVertexId == -1) {
				int faceId;
				Eigen::Vector3f mouseWorldPos;
				if (igl::unproject_onto_mesh(mousePosition, viewer.core().view, viewer.core().proj,
					viewer.core().viewport, currentMesh.getVertices(), currentMesh.getFaces(),
					faceId, mouseWorldPos)) {
					movingVertexId = currentMesh.getClosestVertexId(currentMesh.getFaces(), faceId, mouseWorldPos);
				}
			}
		}
		return false;
	};
}

void Visualizer::handleKeyDown() {
	viewer.callback_key_down = [this](igl::opengl::glfw::Viewer&, unsigned char key, int modifier) {
		Eigen::Vector3d force(0, -20, 0); 
		int target_vertex_index = 300;

		switch (key) {
		case ' ':
			currentMesh.applyForce(target_vertex_index, force);
			updateMesh(currentMesh);
			return true;
		case '1':
			selectionFixedFaces = !selectionFixedFaces;
			fixedMovement = false;
			movingVertex = false;
			return true;
		case '2':
			//fixedMovement = true;
			return true;
		case '3':
			movingVertex = !movingVertex;
			selectionFixedFaces = false;
			fixedMovement = false;
			if (!movingVertex) {
				const Eigen::Vector3d selectedColor(255, 0, 0);
				for (auto f : selectedFaces) {
					auto f_id = f.first;
					Eigen::MatrixXd& mutableColors = const_cast<Eigen::MatrixXd&>(currentMesh.getColors());
					Eigen::Block<Eigen::MatrixXd, 1, -1, false> faceColorBlock = mutableColors.row(f_id);
					faceColorBlock = selectedColor.transpose();
				}
			}
			else {
				const Eigen::Vector3d selectedColor(0, 255, 0);
				for (auto f : selectedFaces) {
					auto f_id = f.first;
					Eigen::MatrixXd& mutableColors = const_cast<Eigen::MatrixXd&>(currentMesh.getColors());
					Eigen::Block<Eigen::MatrixXd, 1, -1, false> faceColorBlock = mutableColors.row(f_id);
					faceColorBlock = selectedColor.transpose();
				}
			}
			viewer.data().set_colors(currentMesh.getColors());
			return true;
		case 'R':
			selectedFaces.clear();
			m_arap.setFixedVertices(selectedFaces);
			selectionFixedFaces = false;
			movingVertex = false;
			fixedMovement = false;
			currentMesh.setInitColors();
			viewer.data().set_colors(currentMesh.getColors());
			return true;
		}
		return false;
	};
}

void Visualizer::handleKeyRelease() {
	viewer.callback_mouse_up = [this](igl::opengl::glfw::Viewer& viewer, int button, int modifier) -> bool {
		mouseClicked = false;

		return false;
	};
}
