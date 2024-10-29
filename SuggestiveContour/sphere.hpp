#ifndef sphere_hpp
#define sphere_hpp

#include <Eigen/Core>
#include "mesh.hpp"

struct Sphere
{
    Eigen::Vector3d center;
    double radius;
};

Sphere update_min_bounding_sphere(Eigen::Vector3d const& c_old, double const& r_old, Eigen::Vector3d const& pi);

Sphere min_bounding_sphere(Mesh const& m);

#endif /* sphere_hpp */
