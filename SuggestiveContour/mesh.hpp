#ifndef mesh_hpp
#define mesh_hpp

#include <iostream>
#include <vector>
#include <Eigen/Core>

inline double triangle_area(Eigen::Vector3d v0, Eigen::Vector3d v1, Eigen::Vector3d v2)
{
    return (v1-v0).cross(v2-v0).norm() / 2.0;
}

class Mesh
{
public:
    std::vector<Eigen::Vector3d> V;
    std::vector<Eigen::Vector3i> F;

private:
    struct Halfedge
    {
        std::vector<Eigen::Vector3i> &F;
        int idx;
        int h_opp;

        Halfedge(std::vector<Eigen::Vector3i> &f);

        int face();
        int h_next();
        int h_prev();
        int v_src();
        int v_tgt();
    };

public:
    std::vector<Halfedge> hEList;
    std::vector<int> h_out;
    std::vector<Eigen::Vector3d> normalV;
    std::vector<Eigen::Vector3d> normalF;

    Mesh();

    void make_halfedge_list();
    int h_ccw(int i);
    int h_cw(int i);
    void compute_normal();
};
#endif /* mesh_hpp */
