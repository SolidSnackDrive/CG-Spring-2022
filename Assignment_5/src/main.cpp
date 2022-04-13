// C++ include
#include <iostream>
#include <string>
#include <vector>
#include <fstream>

// Utilities for the Assignment
#include "raster.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

#define DATA_DIR "/home/gstew/Documents/school/csc305/CG-Spring-2022/Assignment_4/data/"

using namespace std;
using namespace Eigen;

MatrixXd vertices;
MatrixXi facets;

const string data_dir = DATA_DIR;
const string mesh_filename(data_dir + "bunny.off");
vector<VertexAttributes> mesh_vertices; 
vector<VertexAttributes> line_vertices; 

void load_mesh() {
    ifstream in(mesh_filename);
    string token;
    in >> token;
    int nv, nf, ne;
    in >> nv >> nf >> ne;
    vertices.resize(nv,3);
    facets.resize(nf, 3);
    for (int i = 0; i < nv; ++i)
    {
        in >> vertices(i, 0) >> vertices(i, 1) >> vertices(i, 2);
    }
    for (int i = 0; i < nf; i++) {
        int s;
        in >> s >> facets(i,0) >> facets(i,1) >> facets(i,2);
        assert(s==3);
    }
    for (int i = 0; i < facets.rows(); i++) {
        Vector3i r = facets.row(i);
        for (int j = 0; j < 3; j ++) {
            VertexAttributes v(vertices(r(j), 0), vertices(r(j), 1), vertices(r(j), 2));
            mesh_vertices.push_back(v);
        }
    }
    for (int i = 0; i < facets.rows(); i++) {
        Vector3i r = facets.row(i);
        for (int j = 0; j < 2; j ++) {
            VertexAttributes v(vertices(r(j), 0), vertices(r(j), 1));
            line_vertices.push_back(v);
        }
    }

}

int main()
{
    load_mesh();

    // The Framebuffer storing the image rendered by the rasterizer
    Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> frameBuffer(500, 500);

    // Global Constants (empty in this example)
    UniformAttributes uniform;

    // Basic rasterization program
    Program program;

    // The vertex shader is the identity
    program.VertexShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        return va;
    };

    // The fragment shader uses a fixed color
    program.FragmentShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        return FragmentAttributes(1, 0, 0);
    };

    // The blending shader converts colors between 0 and 1 to uint8
    program.BlendingShader = [](const FragmentAttributes &fa, const FrameBufferAttributes &previous) {
        return FrameBufferAttributes(fa.color[0] * 255, fa.color[1] * 255, fa.color[2] * 255, fa.color[3] * 255);
    };

    // One triangle in the center of the screen
    vector<VertexAttributes> vertices;
    vertices.push_back(VertexAttributes(-1, -1, 0));
    vertices.push_back(VertexAttributes(1, -1, 0));
    vertices.push_back(VertexAttributes(0, 1, 0));

    rasterize_triangles(program, uniform, mesh_vertices, frameBuffer);
    //rasterize_lines(program, uniform, line_vertices, 0.5f, frameBuffer);

    vector<uint8_t> image;
    framebuffer_to_uint8(frameBuffer, image);
    stbi_write_png("triangle.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows() * 4);

    return 0;
}
