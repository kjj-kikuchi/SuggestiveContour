

#include "sphere.hpp"

Sphere update_min_bounding_sphere(Eigen::Vector3d const& c_old, double const& r_old, Eigen::Vector3d const& pi)
{
    Sphere s;
    Eigen::Vector3d c_new;
    double r_new;
    double d = (pi - c_old).norm() - r_old;
    if ( (pi - c_old).norm() > r_old ) {
        c_new = c_old + (d / 2.0) * ( (pi - c_old) / (pi - c_old).norm() );
        r_new = r_old + d / 2.0;
    } else {
        c_new = c_old;
        r_new = r_old;
    }
    s = {c_new, r_new};
    return s;
}

Sphere min_bounding_sphere(Mesh const& m)
{
    Sphere s;
    Eigen::Vector3d initial_pa;
    Eigen::Vector3d initial_pb;

    // initial sphere
    double max_dist = (m.V[1] - m.V[0]).norm();
    for (int i = 0; i < m.V.size(); i++) {
        double dist = (m.V[i] - m.V[0]).norm();
        if (dist > max_dist) {
            max_dist = dist;
            initial_pa = m.V[i];
        }
    }
    max_dist = (m.V[0] - initial_pa).norm();
    for (int i = 0; i < m.V.size(); i++) {
        double dist = (m.V[i] - initial_pa).norm();
        if (dist > max_dist) {
            max_dist = dist;
            initial_pb = m.V[i];
        }
    }
    s.center = (initial_pa + initial_pb) / 2.0;
    s.radius = (initial_pa - initial_pb).norm() / 2.0;

    // updaate sphere
    for (int i=0; i<m.V.size(); i++) {
        s = update_min_bounding_sphere(s.center, s.radius, m.V[i]);
    }

    return s;
}
