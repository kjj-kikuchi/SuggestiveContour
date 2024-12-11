#include <map>
#include <numbers>
#include "Eigen/Dense"
#include "curvature.hpp"

double angle(Eigen::Vector3d v0, Eigen::Vector3d v1, Eigen::Vector3d v2)
{
    return acos( (v1-v0).dot(v2-v0) / ( (v1-v0).norm() * (v2-v0).norm() ) );
}

void MeshCurvature::computePrincipalCurvature(Mesh& m)
{
    max.resize(m.V.size());
    min.resize(m.V.size());
    max_direction.resize(m.V.size());
    min_direction.resize(m.V.size());
    tensor.resize(m.V.size(), Eigen::Matrix3d::Zero());

    for (int i = 0; i < m.V.size(); i++) {
        int h_temp = m.h_out[i];
        int h_end = h_temp;
        std::vector<int> edges;         // 2-ring edges
        std::map<std::pair<int, int>, int> edge_map;
        std::map<int, int> face_map;
        double area = 0;    // |B|

        // 領域内の辺を取得・領域内の面積を計算
        do {
            int v_id = m.hEList[h_temp].v_tgt();
            int h_temp2 = m.h_out[v_id];
            int h_end2 = h_temp2;

            do {
                // 探索している辺が内部辺であるとき、追加
                if (m.hEList[h_temp2].h_opp >= 0) {
                    auto key = std::make_pair(m.hEList[h_temp2].v_src(), m.hEList[h_temp2].v_tgt());
                    auto keyswap = std::make_pair(m.hEList[h_temp2].v_tgt(), m.hEList[h_temp2].v_src());
                    if (!edge_map.contains(key) && !edge_map.contains(keyswap)) {
                        edge_map.emplace(key, edge_map.size());
                        edges.push_back(h_temp2);
                    }
                }
                // 探索している辺の h_next が内部辺であるとき、追加
                if (m.hEList[ m.hEList[h_temp2].h_next() ].h_opp >= 0) {
                    auto key = std::make_pair(m.hEList[m.hEList[h_temp2].h_next()].v_src(), m.hEList[m.hEList[h_temp2].h_next()].v_tgt());
                    auto keyswap = std::make_pair(m.hEList[m.hEList[h_temp2].h_next()].v_tgt(), m.hEList[m.hEList[h_temp2].h_next()].v_src());
                    if (!edge_map.contains(key) && !edge_map.contains(keyswap)) {
                        edge_map.emplace(key, edge_map.size());
                        edges.push_back(m.hEList[h_temp2].h_next());
                    }
                }
                // halfedge の属する三角形が探索済みでないならば、その面積を足す
                int face_id = m.hEList[h_temp2].face();
                Eigen::Vector3i face = m.F[face_id];
                if (!face_map.contains(face_id)) {
                    face_map.emplace(face_id, face_map.size());
                    area += ((m.V[face(2)] - m.V[face(0)]).cross(m.V[face(1)] - m.V[face(0)])).norm() / 2.0;
                }
                h_temp2 = m.h_ccw(h_temp2);
            } while (h_temp2 != h_end2 && h_temp2 >= 0);

            h_temp = m.h_ccw(h_temp);
        } while (h_temp != h_end && h_temp >= 0);

        // テンソルを計算
        for (auto& e : edges) {
            Eigen::Vector3d edge = m.V[m.hEList[e].v_tgt()] - m.V[m.hEList[e].v_src()];
            Eigen::Vector3d n = m.normalF[m.hEList[e].face()];
            Eigen::Vector3d n_opp = m.normalF[ m.hEList[ m.hEList[e].h_opp ].face() ];

            double angle_sign = (n.cross(n_opp)).dot(edge.normalized());
            double angle = acos(n.dot(n_opp));
            if (angle_sign < 0) angle = -angle;

            tensor[i] += angle * edge.norm() * edge.normalized() * (edge.normalized()).transpose();
        }
        tensor[i] /= area;

        // 主曲率・主方向を計算
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> ES(tensor[i]);
        Eigen::Vector3d eval = ES.eigenvalues();
        Eigen::Matrix3d evec = ES.eigenvectors();
        double normal_val = std::min({std::fabs(eval(0)), std::fabs(eval(1)), std::fabs(eval(2))});
        if (normal_val == std::abs(eval(0))) {
            max[i] = eval(2);
            min[i] = eval(1);
            max_direction[i] = evec.col(1);
            min_direction[i] = evec.col(2);
        } else if (normal_val == std::abs(eval(1))) {
            max[i] = eval(2);
            min[i] = eval(0);
            max_direction[i] = evec.col(0);
            min_direction[i] = evec.col(2);
        } else if (normal_val == std::abs(eval(2))) {
            max[i] = eval(1);
            min[i] = eval(0);
            max_direction[i] = evec.col(0);
            min_direction[i] = evec.col(1);
        }
//            if (i == 3630 || i == 3600) {
//                std::cout << "i = " << i << std::endl;
//                std::cout << "min : " << min[i] << " max : " << max[i] << std::endl << std::endl;
//                std::cout << "max direction : " << max_direction[i].transpose() << std::endl << std::endl;
//                std::cout << "tensor :\n" << tensor[i] << std::endl << std::endl;
//                std::cout << "eigen value : " << eval.transpose() << std::endl << std::endl;
//                std::cout << "eigen vector :\n" << evec << std::endl << std::endl;
//            }
    }
}

void MeshCurvature::computeRadialCurvature(Mesh const& m, Eigen::Vector3d const& viewpoint)
{
    // 1. radial curvature を計算
    radial.resize(m.V.size());

    for (int i = 0; i < m.V.size(); i++) {
        Eigen::Vector3d radial_direction = (viewpoint - m.V[i]) - ((viewpoint - m.V[i]).dot(m.normalV[i])) * m.normalV[i];
        double phi = acos( (radial_direction).dot(max_direction[i]) / (radial_direction.norm() * max_direction[i].norm()) );
        radial[i] = max[i] * cos(phi)*cos(phi) + min[i] * sin(phi)*sin(phi);
    }

    // 2. 方向微分を計算
    derivative.resize(m.V.size());

    // 各面の勾配ベクトルを計算
    std::vector<Eigen::Vector3d> gradV(m.V.size(), Eigen::Vector3d::Zero());
    std::vector<Eigen::Vector3d> numer(m.V.size(), Eigen::Vector3d::Zero());
    std::vector<double> denom(m.V.size());

    for (int i = 0; i < m.F.size(); i++) {
        std::vector<Eigen::Vector3d> grad_bc(3);   // 重心座標の勾配ベクトル
        double area = triangle_area(m.V[m.F[i](0)], m.V[m.F[i](1)], m.V[m.F[i](2)]);
        // 重心座標の勾配ベクトルを計算
        // ∇λ(p0) = ( R(pi/2, normalF) * he(0)_vec ) / ( height(j) * |he(0)| )
        for (int j = 0; j < 3; j++) {
            grad_bc[j] = (Eigen::AngleAxisd(std::numbers::pi/2.0, m.normalF[i]) * (m.V[m.F[i]((j+2)%3)] - m.V[m.F[i]((j+1)%3)])) / (2*area);
        }

        // ∇f = ∇λ(p0) * f(p0) + ∇λ(p1) * f(p1) + ∇λ(p2) * f(p2)
        Eigen::Vector3d gradF = grad_bc[0] * radial[m.F[i](0)] + grad_bc[1] * radial[m.F[i](1)] + grad_bc[2] * radial[m.F[i](2)];

        for (int j = 0; j < 3; j++) {
            double theta = angle(m.V[m.F[i](j)], m.V[m.F[i]((j+1)%3)], m.V[m.F[i]((j+2)%3)]);
            numer[m.F[i](j)] += theta * gradF;
            denom[m.F[i](j)] += theta;
        }
    }
    // 各頂点の勾配ベクトルを計算し，方向微分を求める
    for (int i = 0; i < m.V.size(); i++) {
        gradV[i] = numer[i] / denom[i];
        Eigen::Vector3d view_direction = (viewpoint - m.V[i]).normalized();
        Eigen::Vector3d radial_direction = view_direction - (view_direction.dot(m.normalV[i])) * m.normalV[i];
        derivative[i] = gradV[i].dot(radial_direction);
    }
}
