#include <vector>
#include <map>
#include <Eigen/Dense>
#include "mesh.hpp"

Mesh::Mesh()
{
    V.resize(0);
    F.resize(0);
    h_out.resize(0);
    normalV.resize(0);
    normalF.resize(0);
}

Mesh::Halfedge::Halfedge(std::vector<Eigen::Vector3i> &f) : F(f) {}

int Mesh::Halfedge::face()
{
    return int(idx/3);
}

int Mesh::Halfedge::h_next()
{
    if(idx % 3 == 2) return idx-2;
    else return idx+1;
}

int Mesh::Halfedge::h_prev()
{
    if(idx % 3 == 0) return idx+2;
    else return idx-1;
}

int Mesh::Halfedge::v_src()
{
    return F[face()][(idx+1)%3];
}

int Mesh::Halfedge::v_tgt()
{
    return F[face()][(idx+2)%3];
}

void Mesh::make_halfedge_list()
{
    std::map<std::pair<int, int>, int> he_map;
    for (int i=0; i<F.size(); i++)
    {
        for (int j=0; j<3; j++)
        {
            Halfedge h(F);
            h.idx = 3*i+j;

            auto key = std::make_pair(h.v_src(), h.v_tgt());
            auto keyswap = std::make_pair(h.v_tgt(), h.v_src());
            if (he_map.contains(keyswap))
            {
                h.h_opp = he_map.at(keyswap);
                hEList[he_map.at(keyswap)].h_opp = 3*i+j;
            }
            else
            {
                h.h_opp = -1;
                he_map.emplace(key, 3*i+j);
            }

            hEList.push_back(h);
        }
    }
    // h_out を計算
    h_out.resize(V.size());
    for (int i=0; i<hEList.size(); i++)
    {
        //  h_out に境界半辺が保存されていない場合のみ更新
        if (hEList[ h_out[hEList[i].v_src()] ].h_opp != -1)
        {
            h_out[hEList[i].v_src()] = i;
        }
    }
    // 境界半辺の h_opp に次の境界半辺を保存する
    /*for (int i=0; i<h_out.size(); i++) {
        if(hEList[h_out[i]].h_opp == -1){
            hEList[h_out[i]].h_opp = -h_out[ hEList[h_out[i]].v_tgt(F) ] - 1;
            // h_i1.h_opp = -i2-1;
        }
    }*/
}

int Mesh::h_ccw(int i)
{
    if(i < 0) return i;
    return hEList[ hEList[i].h_prev() ].h_opp;
}

int Mesh::h_cw(int i)
{
    if (i < 0) return i;
    else if (hEList[i].h_opp < 0) return hEList[i].h_opp;
    return hEList[ hEList[i].h_opp ].h_next();
}

void Mesh::compute_normal()
{
    normalV.resize(V.size(), Eigen::Vector3d::Zero());
    normalF.resize(F.size(), Eigen::Vector3d::Zero());

    std::vector<Eigen::Vector3d> numer(V.size(), Eigen::Vector3d::Zero());
    std::vector<double> denom(V.size());

    for (int i = 0; i < F.size(); i++)
    {
        Eigen::Vector3i f = F[i];
        normalF[i] = ( (V[f(1)] - V[f(0)]).cross(V[f(2)] - V[f(0)]) ).normalized();
        for (int j = 0; j < 3; j++)
        {
            numer[f[j]] += triangle_area(V[f(0)], V[f(1)], V[f(2)]) * normalF[i];
            denom[f(j)] += triangle_area(V[f(0)], V[f(1)], V[f(2)]);
//            numer[f[j]] += (((V[f(1)] - V[f(0)]).cross(V[f(2)] - V[f(0)])).norm() / 2.0) * normalF[i];
//            denom[f(j)] += ((V[f(1)] - V[f(0)]).cross(V[f(2)] - V[f(0)])).norm() / 2.0;
        }
    }
    for (int i = 0; i < V.size(); i++)
    {
        normalV[i] = (numer[i] / denom[i]).normalized();
    }
}
