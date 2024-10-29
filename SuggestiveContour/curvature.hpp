#ifndef curvature_hpp
#define curvature_hpp

#include <vector>
#include "mesh.hpp"

struct MeshCurvature
{
    std::vector<double> max;
    std::vector<double> min;
    std::vector<Eigen::Vector3d> max_direction;
    std::vector<Eigen::Vector3d> min_direction;
    std::vector<Eigen::Matrix3d> tensor;
    std::vector<double> radial;
    std::vector<double> derivative;

//    MeshCurvature();
//    void computePrincipalCurvature(HalfedgeMesh& m);
//    void computeRadialCurvature(HalfedgeMesh const& m, Eigen::Vector3d const& viewpoint);
    void computePrincipalCurvature(Mesh& m);
    void computeRadialCurvature(Mesh const& m, Eigen::Vector3d const& viewpoint);
};

#endif /* curvature_hpp */
