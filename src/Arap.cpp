/*
 * Project: Interactive ARAP
 * File:    Mesh.cpp
 * Authors: Kilian Peis, Ömer Köse, Natalie Adam, Timur Krüger
 */


#include "../include/Arap.h"

Arap::Arap(Mesh& mesh)
    :
    m_mesh(mesh)
{
    m_constructNeighborhood();
    // m_updateWeightMatrix();
    m_updateSparseWeightMatrix();
    m_setSystemMatrix();
    
    std::cout << m_weightMatrix.coeff(0, 0) << "\n";

    // for debugging
    /*
    for (int k = 0; k < m_systemMatrix.outerSize(); ++k) {
        for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(m_systemMatrix, k); it; ++it) {
            std::cout << "(" << it.row() << ", " << it.col() << ") = " << it.value() << "\n";
        }
    }
    */
}

/*
    The main functions that computes the deformation of the mesh referenced in the class.
    TODO: Discuss whether we should incorporate constraints inside the function (by giving the constraint indices as a parameter) or incorporate it outside and then call this function.
    This is just a semantic flavour both will work. I think incorporating constraints inside the function would be better as it will be cleaner in the visualizer end
 
    TODO: Discuss whether we should update the mesh vertices in this function, or we should do it in the caller side.
*/
Eigen::MatrixXd Arap::computeDeformation()
{
    // First update the weight and system matrices as they changes when mesh gets deformed.
    m_updateSparseWeightMatrix();
    m_setSystemMatrix();
    // I would say also set constraints here
    
    // As the Laplacian is symmetric positive definite (actually it is semidefinite we need to be careful here even though in the paper it says it is positive definite), we can use Cholesky factorization
    // Idk, why but it seems Cholesky factorization does not work with RowMajor Sparse Matrices (maybe I did something wrong)
    // Converting System Matrix to the Column Major from Row Major
    Eigen::SparseMatrix<double, Eigen::ColMajor> m_systemMatrixColMajor(m_systemMatrix);
    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double, Eigen::ColMajor>> chol(m_systemMatrixColMajor); // Factorize the system matrix
    // Initial guess will be the vertices of the previous frame (which are the current vertices we have)
    Eigen::MatrixXd deformedVertices = m_mesh.getVertices();
    
    double prevRigidityEnergy = std::numeric_limits<double>::max();
    for(int i = 0; i < MAX_ITERATIONS; ++i)
    {
        // Optimize for the rotations
        std::vector<Eigen::Matrix3d> rotations = m_computeRotations(deformedVertices);
        
        // Compuite RHS
        Eigen::MatrixXd rhs = m_computeRHS(rotations);
        
        // Optimize for the vertices
        deformedVertices = chol.solve(rhs);
        
        double rigidityEnergy = m_computeRigidityEnergy(deformedVertices, rotations);
        std::cout << "Rigidity Energy at the iteration " << i << " is " << rigidityEnergy << "\n";
        // We can stop prematurely
        if(abs(rigidityEnergy - prevRigidityEnergy) <= RIGIDITY_ENERGY_THRESHOLD)
        {
            break;
        }
        
        prevRigidityEnergy = rigidityEnergy;
    }
    
    return deformedVertices;
    
}

void Arap::m_constructNeighborhood()
{
    m_neighbors.resize(m_mesh.getNumVertices());
    const Eigen::MatrixXi& F = m_mesh.getFaces();
    for(int i = 0; i < F.rows(); ++i)
    {
        int v1 = F(i, 0);
        int v2 = F(i, 1);
        int v3 = F(i, 2);
        
        // Add neighbor relations
        m_neighbors[v1].push_back(v2);
        m_neighbors[v1].push_back(v3);
        
        m_neighbors[v2].push_back(v1);
        m_neighbors[v2].push_back(v3);
        
        m_neighbors[v3].push_back(v1);
        m_neighbors[v3].push_back(v2);
    }
    
    // Remove the duplicates
    for(int i = 0; i < m_neighbors.size(); ++i)
    {
        std::sort(m_neighbors[i].begin(), m_neighbors[i].end());
        m_neighbors[i].erase(std::unique(m_neighbors[i].begin(), m_neighbors[i].end()), m_neighbors[i].end());
    }
}

/*
void Arap::m_updateWeightMatrix()
{
    m_weightMatrix = Eigen::MatrixXd::Zero(m_mesh.getNumVertices(), m_mesh.getNumVertices());
    const Eigen::MatrixXi& F = m_mesh.getFaces();
    const Eigen::MatrixXd& V = m_mesh.getVertices();

    for(int i = 0; i < F.rows(); ++i)
    {
     // Directed edges for cotangent computations (in CCT)
     Eigen::Vector3d e01 = V.row(F(i, 1)) - V.row(F(i, 0));
     Eigen::Vector3d e12 = V.row(F(i, 2)) - V.row(F(i, 1));
     Eigen::Vector3d e20 = V.row(F(i, 0)) - V.row(F(i, 2));
     
     // Computing Cotangents
     double c0 = (e01.dot(-e20)) / ((e01.cross(-e20)).norm());
     double c1 = (e12.dot(-e01)) / ((e12.cross(-e01)).norm());
     double c2 = (e20.dot(-e12)) / ((e20.cross(-e12)).norm());
     
     // Symmetrically load the weight matrix
     m_weightMatrix(F(i, 0), F(i, 1)) += 0.5 * c2;
     m_weightMatrix(F(i, 1), F(i, 0)) += 0.5 * c2;
     
     m_weightMatrix(F(i, 1), F(i, 2)) += 0.5 * c0;
     m_weightMatrix(F(i, 2), F(i, 1)) += 0.5 * c0;
     
     m_weightMatrix(F(i, 0), F(i, 2)) += 0.5 * c1;
     m_weightMatrix(F(i, 2), F(i, 0)) += 0.5 * c1;
    }

    // Set the diagonal weights to 1
    for(int i = 0; i < m_mesh.getNumVertices(); ++i)
    {
     m_weightMatrix(i, i) = 1.0;
    }
}
*/

void Arap::m_updateSparseWeightMatrix()
{
    const Eigen::MatrixXi& F = m_mesh.getFaces();
    const Eigen::MatrixXd& V = m_mesh.getVertices();
    
    typedef Eigen::Triplet<double> T;
    std::vector<T> triplets;
    m_weightMatrix = Eigen::SparseMatrix<double>(m_mesh.getNumVertices(), m_mesh.getNumVertices());
    
    for(int i = 0; i < F.rows(); ++i)
    {
        // Directed edges for cotangent computations (in CCT)
        Eigen::Vector3d e01 = V.row(F(i, 1)) - V.row(F(i, 0));
        Eigen::Vector3d e12 = V.row(F(i, 2)) - V.row(F(i, 1));
        Eigen::Vector3d e20 = V.row(F(i, 0)) - V.row(F(i, 2));
        
        // Computing Cotangents
        double c0 = (e01.dot(-e20)) / ((e01.cross(-e20)).norm());
        double c1 = (e12.dot(-e01)) / ((e12.cross(-e01)).norm());
        double c2 = (e20.dot(-e12)) / ((e20.cross(-e12)).norm());
        
        // Symmetrically load the weight matrix
        triplets.emplace_back(F(i, 0), F(i, 1), 0.5 * c2);
        triplets.emplace_back(F(i, 1), F(i, 0), 0.5 * c2);
        
        triplets.emplace_back(F(i, 1), F(i, 2), 0.5 * c0);
        triplets.emplace_back(F(i, 2), F(i, 1), 0.5 * c0);
        
        triplets.emplace_back(F(i, 0), F(i, 2), 0.5 * c1);
        triplets.emplace_back(F(i, 2), F(i, 0), 0.5 * c1);
    }
    
    // Set the diagonal weights to 1
    for(int i = 0; i < m_mesh.getNumVertices(); ++i)
    {
        triplets.emplace_back(i, i, 1.0);
    }
    
    m_weightMatrix.setFromTriplets(triplets.begin(), triplets.end());
}

void Arap::m_setSystemMatrix()
{
    m_systemMatrix.resize(m_weightMatrix.rows(), m_weightMatrix.cols());
    m_systemMatrix = -m_weightMatrix;

    // Add diagonal value
    for (int i = 0; i < m_weightMatrix.rows(); ++i)
    {
        m_systemMatrix.coeffRef(i, i) += -m_systemMatrix.row(i).sum();
    }
}

std::vector<Eigen::Matrix3d> Arap::m_computeRotations(Eigen::MatrixXd& V_deformed)
{
    const Eigen::MatrixXd& V = m_mesh.getVertices();
    std::vector<Eigen::Matrix3d> R(V.rows());

    for (int i = 0; i < V.rows(); ++i)
    {
        // Edges of vertices
        Eigen::MatrixXd P(3, m_neighbors[i].size());
        // Diagonal matrix with weights
        Eigen::MatrixXd D(m_neighbors[i].size(), m_neighbors[i].size());
        // Edges of deformed vertices
        Eigen::MatrixXd P_p(3, m_neighbors[i].size());

        for (int j = 0; j < m_neighbors[i].size(); ++j)
        {
            P.col(j) = V.row(i) - V.row(m_neighbors[i][j]);
            D(j, j) = m_weightMatrix.coeffRef(i, m_neighbors[i][j]);
            P_p.col(j) = V_deformed.row(i) - V_deformed.row(m_neighbors[i][j]);
        }
        
        // Covariance matrix: S = P * D * P'^T
        Eigen::MatrixXd S = P * D * P_p.transpose();

        // Perform SVD: S = U * sum(V^T)
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(S, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::MatrixXd U = svd.matrixU();
        Eigen::MatrixXd V = svd.matrixV();

        // Ensure determinant is positive
        if (U.determinant() < 0)
        {
            U.col(2) *= -1.0;
        }

        // R = V * U^T
        R[i] = V * U.transpose();
    }

    return R;
}


/*
    Computes the rigidity energy given the deformed vertices and the rotations. We will use it as an additional stopping condition.
*/
double Arap::m_computeRigidityEnergy(const Eigen::MatrixXd& V_deformed, const std::vector<Eigen::Matrix3d>& rotations)
{
    double rigidityEnergy = 0.0;
    const Eigen::MatrixXd& V = m_mesh.getVertices();
    
    for(int i = 0; i < V.rows(); ++i)
    {
        double energy = 0.0;
        const std::vector<int>& neighbors = m_neighbors[i];
        for(int j = 0; j < neighbors.size(); ++j)
        {
            int neighborIdx = neighbors[j];
            Eigen::Vector3d diffDeformed = V_deformed.row(i) - V_deformed.row(neighborIdx);
            Eigen::Vector3d diffInitial  = V.row(i) - V.row(neighborIdx);
            energy += m_weightMatrix.coeff(i, neighborIdx) * (diffDeformed - rotations[i] * diffInitial).squaredNorm();
        }
        
        rigidityEnergy += m_weightMatrix.coeff(i, i) * energy;
    }
    
    return rigidityEnergy;
}


Eigen::MatrixXd Arap::m_computeRHS(const std::vector<Eigen::Matrix3d>& rotations)
{
    return m_mesh.getVertices();
}
