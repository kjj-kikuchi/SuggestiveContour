#ifndef contour_hpp
#define contour_hpp

#include <vector>
#include "curvature.hpp"

std::vector<double> computeOrientationFunction(Mesh const& m, std::vector<double>& orientation, Eigen::Vector3d const& viewpoint);

struct Contour
{
    std::vector<Eigen::Vector3d> points;
    std::vector<Eigen::Vector2i> lines;

    void generateContour(Mesh& m, std::vector<double> const& orientation);

    void generateSuggestiveContour(Mesh const& m, MeshCurvature const& curv,
                                   double const& angle_threshold, double const& der_threshold,
                                   Eigen::Vector3d const& viewpoint);
};

#endif /* contour_hpp */
