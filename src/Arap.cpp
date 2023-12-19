/*
 * Project: Interactive ARAP
 * File:    Arap.cpp
 * Authors: Kilian Peis, Ömer Köse, Natalie Adam, Timur Krüger
*/

#include "Arap.h"
#include <Eigen/Sparse>

Arap::Arap(Mesh& mesh) : mesh(mesh) {}

void Arap::collectFixedVertices(const std::vector<int>& fixedVertices) {
    // Example: Store the fixed vertices indices for later use
    // This could be stored in a member variable for use in updateSystemMatrix
}

void Arap::updateSystemMatrix() {
    // This function should update the system matrix based on constraints
    // The implementation depends on how your ARAP system is designed

    // Example pseudo-implementation:
    // 1. Reset or initialize the system matrix
    // 2. Iterate over the mesh's vertices
    // 3. For each vertex, compute its contribution to the system matrix
    //    This may involve calculating local transformations and rotations
    // 4. Consider the fixed vertices in the system matrix computation
    // 5. Apply any additional constraints or conditions
    // 6. Finalize the system matrix

    // Note: The actual implementation details will vary greatly depending on your specific ARAP setup and the data structures you're using.
}
