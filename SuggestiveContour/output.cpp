//
//  output.cpp
//  MeshContour
//
//  Created by 菊池祐作 on 2024/07/21.
//

#include "output.hpp"

void outputMesh(std::vector<Eigen::Vector3d> const& verts, std::vector<Eigen::Vector3i> const& faces, std::string name)
{
    std::ofstream of;
    name.erase(name.length()-4);
    std::string filename = name + "_normalized.obj";
    of.open(filename, std::ios::out);
    for(auto& v : verts){
        of << "v " << v(0) << " " << v(1) << " " << v(2) << std::endl;
    }
    for(auto& f : faces){
        of << "f " << f(0)+1 << " " << f(1)+1 << " " << f(2)+1 << std::endl;
    }
    of.close();
}

void outputContour(std::vector<Eigen::Vector3d> const& points, std::vector<Eigen::Vector2i> const& lines, std::string name, std::string const& contour_type)
{
    std::ofstream of;
    name.erase(name.length()-4);
    std::string filename = name + "_" + contour_type + ".obj";
    of.open(filename, std::ios::out);
    for(auto& p : points){
        of << "v " << p(0) << " " << p(1) << " " << p(2) << std::endl;
    }
    for(auto& l : lines){
        of << "l " << l(0)+1 << " " << l(1)+1 << std::endl;
    }
    of.close();
}

void outputViewpointPosition(Eigen::Vector3d const& vp)
{
    std::ofstream of;
    std::string filename = "viewpoint.obj";
    of.open(filename, std::ios::out);
    /*std::vector<Eigen::Vector3d> v{{-0.005, -0.005, -0.005}, {0.005, -0.005,  -0.005}, {-0.005, 0.005, -0.005}, {0.005, 0.005, -0.005}, {-0.005, -0.005, 0.005}, {0.005, -0.005, 0.005}, {-0.005, 0.005, 0.005}, {0.005, 0.005, 0.005}};
    std::vector<Eigen::Vector4i> f{{1, 3, 4, 2}, {1, 5, 7, 3}, {1, 2, 6, 5}, {8, 7, 5, 6}, {8, 4, 3, 7}, {8, 6, 2, 4}};
    for(int i=0; i<v.size(); i++){
        of << "v " << vp(0)+v[i](0) << " " << vp(1)+v[i](1) << " " << vp(2)+v[i](2) << std::endl;
    }
    for(int i=0; i<f.size(); i++){
        of << "f " << f[i](0) << " " << f[i](1) << " " << f[i](2) << " " << f[i](3) << std::endl;
    }*/
    of << "v " << vp.transpose() << std::endl;
    of << "p 1" << std::endl;
    of.close();
}

void outputFunctionValues(Mesh const& m, std::vector<double>& func, std::string name, std::string const& func_type)
{
    std::ofstream of;
    name.resize(name.length() - 4);
    std::string filename = name + "_" + func_type + ".vtk";
    of.open(filename, std::ios::out);
    of << "# vtk DataFile Version 3.0\n" << "vtk output\n" << "ASCII\n" << "DATASET POLYDATA\n";
    of << "POINTS " << m.V.size() << " float\n";
    for(auto& v : m.V){
        of << v(0) << " " << v(1) << " " << v(2) << std::endl;
    }
    of << "POLYGONS " << m.F.size() << " " << 4*m.F.size() << std::endl;
    for(auto& f: m.F){
        of << "3 " << f(0) << " " << f(1) << " " << f(2) << std::endl;
    }
    of << "POINT_DATA " << m.V.size() << std::endl;
    of << "SCALARS " << func_type << " float\n";
    of << "LOOKUP_TABLE default\n";
    for(auto& f : func){
        of << f << std::endl;
    }
    of.close();
}

void outputVectors(Mesh const& m, std::vector<Eigen::Vector3d> const& vec, std::string name, std::string const& vec_type)
{
    std::ofstream of;
    name.resize(name.length() - 4);
    std::string filename = name + "_" + vec_type + ".vtk";
    of.open(filename, std::ios::out);
    of << "# vtk DataFile Version 3.0\n" << "vtk output\n" << "ASCII\n" << "DATASET POLYDATA\n";
    of << "POINTS " << m.V.size() << " float\n";
    for(auto& v : m.V){
        of << v(0) << " " << v(1) << " " << v(2) << std::endl;
    }
    of << "POLYGONS " << m.F.size() << " " << 4*m.F.size() << std::endl;
    for(auto& f: m.F){
        of << "3 " << f(0) << " " << f(1) << " " << f(2) << std::endl;
    }
    of << "POINT_DATA " << m.V.size() << std::endl;
    of << "VECTORS " << vec_type << " float\n";
    for (auto v : vec) {
        of << v(0) << " " << v(1) << " " << v(2) << std::endl;
    }
    of.close();
}

void openFile(std::string name)
{
    // normalized mesh
    name.erase(name.length()-4);
    std::string command = "open -a /Applications/ParaView-5.12.0.app " + name + "_normalized.obj";
    system(command.c_str());
    // Contour
    command = "open -a /Applications/ParaView-5.12.0.app " + name + "_contour.obj";
    system(command.c_str());
    // Suggestive Contour
    command = "open -a /Applications/ParaView-5.12.0.app " + name + "_suggestive.obj";
    system(command.c_str());

    // viewpoint
    // command = "open -a /Applications/ParaView-5.12.0.app viewpoint.obj";
    // system(command.c_str());
}
