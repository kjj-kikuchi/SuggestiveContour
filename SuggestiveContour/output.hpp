#ifndef output_hpp
#define output_hpp

#include <fstream>
#include "mesh.hpp"

void outputMesh(std::vector<Eigen::Vector3d> const& verts, std::vector<Eigen::Vector3i> const& faces, std::string name);
void outputContour(std::vector<Eigen::Vector3d> const& points, std::vector<Eigen::Vector2i> const& lines, std::string name, std::string const& contour_type);

void outputViewpointPosition(Eigen::Vector3d const& vp);

void outputFunctionValues(Mesh const& m, std::vector<double>& func, std::string name, std::string const& func_type);

void outputVectors(Mesh const& m, std::vector<Eigen::Vector3d> const& vec, std::string name, std::string const& vec_type);
void openFile(std::string name);

#endif /* output_hpp */
