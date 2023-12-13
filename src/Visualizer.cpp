#include "../include/Visualizer.h"


Visualizer::Visualizer(const std::string& meshPath)
    :
    m_mesh{meshPath}
{
    m_viewer.data().set_mesh(m_mesh.m_vertices, m_mesh.m_faces);
    m_viewer.data().set_colors(m_mesh.m_colors);
}

void Visualizer::setMesh(const std::string& meshPath)
{
    m_mesh = Mesh(meshPath);
    m_viewer.data().set_mesh(m_mesh.m_vertices, m_mesh.m_faces);
    m_viewer.data().set_colors(m_mesh.m_colors);
    return;
}

void Visualizer::setWireframeLineMode(bool wireframe)
{
    m_viewer.data().show_lines = wireframe;
}

void Visualizer::launch()
{
    setWireframeLineMode(false);
    m_viewer.launch();
    return;
}
