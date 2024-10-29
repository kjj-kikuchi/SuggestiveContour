#include <vector>
#include <map>
#include "contour.hpp"

std::vector<double> computeOrientationFunction(Mesh const& m, std::vector<double>& orientation, Eigen::Vector3d const& viewpoint)
{
    orientation.resize(m.V.size());
    for (int i = 0; i < m.V.size(); i++) {
        orientation[i] = (viewpoint - m.V[i]).dot(m.normalV[i]);
    }
    return orientation;
}

void Contour::generateContour(Mesh& m, std::vector<double> const& orientation)
{
    std::map<std::tuple<double, double, double>, int> points_map;

    for (int i = 0; i < m.F.size(); i++)
    {
        for (int j = 0; j < 3; j++)
        {
            if (m.hEList[m.h_out[m.F[i](j)]].h_opp < 0)
            {
                Eigen::Vector3d p0 = m.V[m.F[i](j)];
                Eigen::Vector3d p1 = m.V[m.hEList[m.h_out[m.F[i](j)]].v_tgt()];
                int p0_idx, p1_idx;
                auto p0t = std::make_tuple(p0(0), p0(1), p0(2));
                auto p1t = std::make_tuple(p1(0), p1(1), p1(2));
                if (points_map.contains(p0t)) {
                    p0_idx = points_map.at(p0t);
                } else {
                    p0_idx = (int)points.size();
                    points_map.emplace(p0t, p0_idx);
                    points.push_back(p0);
                }
                if (points_map.contains(p1t)) {
                    p1_idx = points_map.at(p1t);
                } else {
                    p1_idx = (int)points.size();
                    points_map.emplace(p1t, p1_idx);
                    points.push_back(p1);
                }
                lines.push_back(Eigen::Vector2i{p0_idx, p1_idx});
            }
            if (orientation[m.F[i](j)] * orientation[m.F[i]((j+1)%3)] < 0 &&
                orientation[m.F[i](j)] * orientation[m.F[i]((j+2)%3)] < 0) {

                double t0 = orientation[m.F[i]((j+1)%3)] / (orientation[m.F[i]((j+1)%3)] - orientation[m.F[i](j)]);
                double t1 = orientation[m.F[i]((j+2)%3)] / (orientation[m.F[i]((j+2)%3)] - orientation[m.F[i](j)]);
                Eigen::Vector3d p0 = (1 - t0) * m.V[m.F[i]((j+1)%3)] + t0 * m.V[m.F[i](j)];
                Eigen::Vector3d p1 = (1 - t1) * m.V[m.F[i]((j+2)%3)] + t1 * m.V[m.F[i](j)];
                int p0_idx, p1_idx;

                auto p0t = std::make_tuple(p0(0), p0(1), p0(2));
                auto p1t = std::make_tuple(p1(0), p1(1), p1(2));
                if (points_map.contains(p0t)) {
                    p0_idx = points_map.at(p0t);
                } else {
                    p0_idx = (int)points.size();
                    points_map.emplace(p0t, p0_idx);
                    points.push_back(p0);
                }
                if (points_map.contains(p1t)) {
                    p1_idx = points_map.at(p1t);
                } else {
                    p1_idx = (int)points.size();
                    points_map.emplace(p1t, p1_idx);
                    points.push_back(p1);
                }
                lines.push_back(Eigen::Vector2i{p0_idx, p1_idx});
                break;
            }
        }
    }
}

void Contour::generateSuggestiveContour(Mesh const& m, MeshCurvature const& curv,
                               double const& angle_threshold, double const& der_threshold,
                               Eigen::Vector3d const& viewpoint)
{
    std::map<std::tuple<double, double, double>, int> points_map;

    for (int i = 0; i < m.F.size(); i++) {
        if ((viewpoint - m.V[m.F[i](0)]).dot(m.normalF[i]) < 0) {
            // Do nothing.
        } else {
            for (int j = 0; j < 3; j++) {
                Eigen::Vector3i f = m.F[i];
                if (curv.radial[f(j)] * curv.radial[f((j+1)%3)] < 0 &&
                    curv.radial[f(j)] * curv.radial[f((j+2)%3)] < 0) {

                    double t0 = curv.radial[f((j+1)%3)] / (curv.radial[f((j+1)%3)] - curv.radial[f(j)]);
                    double t1 = curv.radial[f((j+2)%3)] / (curv.radial[f((j+2)%3)] - curv.radial[f(j)]);

                    // 方向微分 Dw κr > 0 の判定
                    double derivative_p0 = (1-t0) * curv.derivative[f((j+1)%3)] + t0 * curv.derivative[f(j)];
                    if (curv.derivative[f(j)] <= 0 && curv.derivative[f((j+1)%3)] <= 0) {
                        break;
                    } else if (curv.derivative[f(j)] * curv.derivative[f((j+1)%3)] <= 0) {
                        if (derivative_p0 <= 0) {
                            break;
                        }
                    }

                    Eigen::Vector3d p0 = (1 - t0) * m.V[f((j+1)%3)] + t0 * m.V[f(j)];
                    Eigen::Vector3d p1 = (1 - t1) * m.V[f((j+2)%3)] + t1 * m.V[f(j)];

                    Eigen::Vector3d view_direction = (viewpoint - p0).normalized();
                    // 閾値 0 < θc < angle(n, p) の判定
                    Eigen::Vector3d normal_p0 = ((1-t0) * m.normalV[f((j+1)%3)] + t0 * m.normalV[f(j)]).normalized();
                    double angle = acos(normal_p0.dot(view_direction) / view_direction.norm());
                    if ((angle > 0 && angle < angle_threshold)) {
                        break;
                    }
                    // 閾値 td < Dw κr/|w| の判定
                    Eigen::Vector3d radial_direction = view_direction - (view_direction.dot(p0) * p0);
                    if (derivative_p0 / radial_direction.norm() <= der_threshold) {
                        break;
                    }

                    int p0_idx, p1_idx;

                    auto p0t = std::make_tuple(p0(0), p0(1), p0(2));
                    auto p1t = std::make_tuple(p1(0), p1(1), p1(2));
                    if (points_map.contains(p0t)) {
                        p0_idx = points_map.at(p0t);
                    } else {
                        p0_idx = (int)points.size();
                        points_map.emplace(p0t, p0_idx);
                        points.push_back(p0);
                    }
                    if (points_map.contains(p1t)) {
                        p1_idx = points_map.at(p1t);
                    } else {
                        p1_idx = (int)points.size();
                        points_map.emplace(p1t, p1_idx);
                        points.push_back(p1);
                    }
                    lines.push_back(Eigen::Vector2i{p0_idx, p1_idx});
                    break;
                }
            }
        }
    }
}
