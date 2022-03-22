////////////////////////////////////////////////////////////////////////////////
// C++ include
#include <iostream>
#include <string>
#include <vector>
#include <limits>
#include <fstream>
#include <algorithm>
#include <numeric>
#include <queue>
#include <stack>

// Utilities for the Assignment
#include "utils.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

// Shortcut to avoid Eigen:: everywhere, DO NOT USE IN .h
using namespace Eigen;

////////////////////////////////////////////////////////////////////////////////
// Class to store tree
////////////////////////////////////////////////////////////////////////////////
class AABBTree
{
public:
    class Node
    {
    public:
        AlignedBox3d bbox;
        int parent;   // Index of the parent node (-1 for root)
        int left;     // Index of the left child (-1 for a leaf)
        int right;    // Index of the right child (-1 for a leaf)
        int triangle; // Index of the node triangle (-1 for internal nodes)
    };

    std::vector<Node> nodes;
    int root;

    AABBTree() = default;                           // Default empty constructor
    AABBTree(const MatrixXd &V, const MatrixXi &F); // Build a BVH from an existing mesh
};

////////////////////////////////////////////////////////////////////////////////
// Scene setup, global variables
////////////////////////////////////////////////////////////////////////////////
const std::string data_dir = DATA_DIR;
const std::string filename("raytrace.png");
const std::string mesh_filename(data_dir + "dragon.off");

//Camera settings
const double focal_length = 2;
const double field_of_view = 0.7854; //45 degrees
const bool is_perspective = true;
const Vector3d camera_position(0, 0, 2);

// Triangle Mesh
MatrixXd vertices; // n x 3 matrix (n points)
MatrixXi facets;   // m x 3 matrix (m triangles)
AABBTree bvh;

//Material for the object, same material for all objects
const Vector4d obj_ambient_color(0.0, 0.5, 0.0, 0);
const Vector4d obj_diffuse_color(0.5, 0.5, 0.5, 0);
const Vector4d obj_specular_color(0.2, 0.2, 0.2, 0);
const double obj_specular_exponent = 256.0;
const Vector4d obj_reflection_color(0.7, 0.7, 0.7, 0);

// Precomputed (or otherwise) gradient vectors at each grid node
const int grid_size = 20;
std::vector<std::vector<Vector2d>> grid;

//Lights
std::vector<Vector3d> light_positions;
std::vector<Vector4d> light_colors;
//Ambient light
const Vector4d ambient_light(0.2, 0.2, 0.2, 0);

//Fills the different arrays
void setup_scene()
{
    //Loads file
    std::ifstream in(mesh_filename);
    std::string token;
    in >> token;
    int nv, nf, ne;
    in >> nv >> nf >> ne;
    vertices.resize(nv, 3);
    facets.resize(nf, 3);
    for (int i = 0; i < nv; ++i)
    {
        in >> vertices(i, 0) >> vertices(i, 1) >> vertices(i, 2);
    }
    for (int i = 0; i < nf; ++i)
    {
        int s;
        in >> s >> facets(i, 0) >> facets(i, 1) >> facets(i, 2);
        assert(s == 3);
    }

    //setup tree
    std::cout << "Constructing BVH tree." << std::endl;
    bvh = AABBTree(vertices, facets);
    std::cout << "Finished constructing tree." << std::endl;

    //Lights
    light_positions.emplace_back(8, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(6, -8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(4, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(2, -8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(0, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(-2, -8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(-4, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);
}

////////////////////////////////////////////////////////////////////////////////
// BVH Code
////////////////////////////////////////////////////////////////////////////////

AlignedBox3d bbox_from_triangle(const Vector3d &a, const Vector3d &b, const Vector3d &c)
{
    AlignedBox3d box;
    box.extend(a);
    box.extend(b);
    box.extend(c);
    return box;
}

int construct_tree2(const MatrixXd& centroids, std::vector<int>& indices, int start_range, int end_range, AABBTree *tree) {
    /***
     * 
        Split the input set of triangles into two sets S1 and S2.
        Recursively build the subtree T1 corresponding to S1.
        Recursively build the subtree T2 corresponding to S2.
        Update the box of the current node by merging boxes of the root of T1 and T2.
        Return the new root node R.
    ***/

    AABBTree::Node R; //Root node for this depth
    int R_index = tree->nodes.size(); //Root index in the tree std vector
    tree->nodes.push_back(R);

    if((end_range - start_range) == 1) { //At this point we are down to just one triangle, so get bounding box from vertices
        Vector3i facet_row = facets.row(indices[start_range]);
        Vector3d a = vertices.row(facet_row(0));
        Vector3d b = vertices.row(facet_row(1));
        Vector3d c = vertices.row(facet_row(2));
        tree->nodes[R_index].bbox = bbox_from_triangle(a,b,c);
        tree->nodes[R_index].triangle = indices[start_range];
        tree->nodes[R_index].left = -1;
        tree->nodes[R_index].right = -1;

        return R_index;
    }

    Vector3d min_corner(0,0,0);
    min_corner = centroids.colwise().minCoeff();
    Vector3d max_corner(0,0,0);
    max_corner = centroids.colwise().maxCoeff();

    Vector3d diag = max_corner - min_corner;
    
    VectorXd::Index axis_index = 0;
    diag.maxCoeff(&axis_index);

    if(axis_index == 0) {
        std::sort(indices.begin() + start_range, indices.begin() + end_range, 
            [&] (int A, int B) -> bool {
                return centroids.row(A)(0) < centroids.row(B)(0);
            });
    }
    else if(axis_index == 1) {
        std::sort(indices.begin() + start_range, indices.begin() + end_range, 
            [&] (int A, int B) -> bool {
                return centroids.row(A)(1) < centroids.row(B)(1);
            });
    }
    else if(axis_index == 2) {
        std::sort(indices.begin() + start_range, indices.begin() + end_range, 
            [&] (int A, int B) -> bool {
                return centroids.row(A)(2) < centroids.row(B)(2);
            });
    }
    

    //std::size_t const half_size = indices.size() / 2;
    int const half_size = (end_range - start_range) / 2;
    /*std::vector<int> left(indices.begin(), indices.begin() + half_size);
    std::vector<int> right(indices.begin() + half_size, indices.end());*/

    int left_start = start_range;
    int left_end = start_range + half_size;

    int right_start = start_range + half_size;
    int right_end = end_range;

    int T1 = construct_tree2(centroids, indices, left_start, left_end, tree);
    int T2 = construct_tree2(centroids, indices, right_start, right_end, tree);

    tree->nodes[T1].parent = R_index;
    tree->nodes[T2].parent = R_index;
    tree->nodes[R_index].left = T1;
    tree->nodes[R_index].right = T2;
    tree->nodes[R_index].triangle = -1;
    tree->nodes[R_index].bbox.extend(tree->nodes[T1].bbox);
    tree->nodes[R_index].bbox.extend(tree->nodes[T2].bbox);

    return R_index;
}

AABBTree::AABBTree(const MatrixXd &V, const MatrixXi &F)
{
    // Compute the centroids of all the triangles in the input mesh

    MatrixXd centroids(F.rows(), V.cols());
    centroids.setZero();
    std::vector<int> indices(centroids.rows());
    std::iota(indices.begin(), indices.end(), 0);
    for (int i = 0; i < F.rows(); ++i)
    {
        for (int k = 0; k < F.cols(); ++k)
        {
            centroids.row(i) += V.row(F(i, k));
        }
        centroids.row(i) /= F.cols();
    }

    assert(centroids.rows() == indices.size());

    //this->root = construct_tree(pairs, this, -1);
    this->root = construct_tree2(centroids, indices, 0, indices.size(), this);

    // TODO

    // Top-down approach.
    // Split each set of primitives into 2 sets of roughly equal size,
    // based on sorting the centroids along one direction or another.
}


////////////////////////////////////////////////////////////////////////////////
// Intersection code
////////////////////////////////////////////////////////////////////////////////


inline bool uvt_check(double u, double v, double t) {
    return t > 0 && u >= 0 && v >= 0 && (u + v <= 1);
}

double ray_triangle_intersection(const Vector3d &ray_origin, const Vector3d &ray_direction, const Vector3d &a, const Vector3d &b, const Vector3d &c, Vector3d &p, Vector3d &N)
{
    // Compute whether the ray intersects the given triangle.
    // If you have done the parallelogram case, this should be very similar to it.
    Vector3d pgram_origin = a;
    Vector3d pgram_u = b - pgram_origin;
    Vector3d pgram_v = c - pgram_origin;

    Matrix3d linA;
    linA << pgram_u, pgram_v, -1 * ray_direction.normalized();
    Vector3d linb = ray_origin - pgram_origin;
    Vector3d linx = linA.partialPivLu().solve(linb);
    
    double t = -1;
    
    if (uvt_check(linx(0), linx(1), linx(2)))
    {
        t = linx(2);
    }
    else 
    {
        return -1;    
    }

    //TODO set the correct intersection point, update p and N to the correct values
    p = ray_origin + t * ray_direction.normalized();
    N = -1 * pgram_u.normalized().cross(pgram_v.normalized());

    return t;
}

bool ray_box_intersection(const Vector3d &ray_origin, const Vector3d &ray_direction, const AlignedBox3d &box)
{
    // Compute whether the ray intersects the given box.
    // we are not testing with the real surface here anyway.

    Vector3d box_min = box.min();
    Vector3d box_max = box.max();
    Vector3d inv_dir = ray_direction.cwiseInverse();

    Vector3d t0 = (box_min - ray_origin).cwiseProduct(inv_dir);
    Vector3d t1 = (box_max - ray_origin).cwiseProduct(inv_dir);

    Vector3d tsmall = t0.cwiseMin(t1);
    Vector3d tbig = t0.cwiseMax(t1);


    double tmin = std::max(tsmall[0], std::max(tsmall[1], tsmall[2]));
    double tmax = std::min(tbig[0], std::min(tbig[1], tbig[2]));

    return tmin <= tmax;
}

//Finds the closest intersecting object returns its index
//In case of intersection it writes into p and N (intersection point and normals)
bool find_nearest_object(const Vector3d &ray_origin, const Vector3d &ray_direction, Vector3d &p, Vector3d &N)
{
    Vector3d tmp_p, tmp_N;

    // TODO
    // Method (1): Traverse every triangle and return the closest hit.
    // Method (2): Traverse the BVH tree and test the intersection with a
    // triangles at the leaf nodes that intersects the input ray.
    
    AABBTree::Node root = bvh.nodes[bvh.root];
    if(ray_box_intersection(ray_origin, ray_direction, root.bbox) == false)
    {
        return false;
    }

    std::stack<AABBTree::Node> queue;
    std::vector<AABBTree::Node> leaves;

    queue.push(root);

    while(queue.size() > 0) {
        AABBTree::Node current = queue.top();
        queue.pop();

        if(current.left < 0 && current.right < 0) {
                leaves.push_back(current);
        }
        else {
            AABBTree::Node left_node = bvh.nodes[current.left];
            AABBTree::Node right_node = bvh.nodes[current.right];
            if(ray_box_intersection(ray_origin, ray_direction, left_node.bbox)) {
                queue.push(left_node);
            }
            if(ray_box_intersection(ray_origin, ray_direction, right_node.bbox)) {
                queue.push(right_node);
            }
        }
    }

    double t_final = MAXFLOAT;
    for(AABBTree::Node& n : leaves) {
        Vector3i facet_row = facets.row(n.triangle);
        Vector3d a = vertices.row(facet_row(0));
        Vector3d b = vertices.row(facet_row(1));
        Vector3d c = vertices.row(facet_row(2));

        double t = ray_triangle_intersection(ray_origin, ray_direction, a, b, c, tmp_p, tmp_N);

        if ( t > 0 ) {
            if( t < t_final ) {
                t_final = t;
                p = tmp_p;
                N = tmp_N;
            }  
        }
    }

    if(t_final == MAXFLOAT) {
        return false;
    }
    else {
        return true;
    }
    /*
    double t_final = MAXFLOAT;
    for (int i = 0; i < facets.rows(); i ++) {
        Vector3i facet_row = facets.row(i);
        Vector3d a = vertices.row(facet_row(0));
        Vector3d b = vertices.row(facet_row(1));
        Vector3d c = vertices.row(facet_row(2));

        double t = ray_triangle_intersection(ray_origin, ray_direction, a, b, c, tmp_p, tmp_N);

        if ( t > 0 ) {
            if( t < t_final ) {
                t_final = t;
                p = tmp_p;
                N = tmp_N;
            }  
        }
    }

    if(t_final == MAXFLOAT) {
        return false;
    }
    else {
        return true;
    }*/

}

////////////////////////////////////////////////////////////////////////////////
// Raytracer code
////////////////////////////////////////////////////////////////////////////////

Vector4d shoot_ray(const Vector3d &ray_origin, const Vector3d &ray_direction)
{
    //Intersection point and normal, these are output of find_nearest_object
    Vector3d p, N;

    const bool nearest_object = find_nearest_object(ray_origin, ray_direction, p, N);

    if (!nearest_object)
    {
        // Return a transparent color
        return Vector4d(0, 0, 0, 0);
    }

    // Ambient light contribution
    const Vector4d ambient_color = obj_ambient_color.array() * ambient_light.array();

    // Punctual lights contribution (direct lighting)
    Vector4d lights_color(0, 0, 0, 0);
    for (int i = 0; i < light_positions.size(); ++i)
    {
        const Vector3d &light_position = light_positions[i];
        const Vector4d &light_color = light_colors[i];

        Vector4d diff_color = obj_diffuse_color;

        // TODO: Add shading parameters

        // Diffuse contribution
        const Vector3d Li = (light_position - p).normalized();
        const Vector4d diffuse = diff_color * std::max(Li.dot(N), 0.0);

        // Specular contribution
        const Vector3d Hi = (Li - ray_direction).normalized();
        const Vector4d specular = obj_specular_color * std::pow(std::max(N.dot(Hi), 0.0), obj_specular_exponent);
        // Vector3d specular(0, 0, 0);

        // Attenuate lights according to the squared distance to the lights
        const Vector3d D = light_position - p;
        lights_color += (diffuse + specular).cwiseProduct(light_color) / D.squaredNorm();
    }

    // Rendering equation
    Vector4d C = ambient_color + lights_color;

    //Set alpha to 1
    C(3) = 1;

    return C;
}

////////////////////////////////////////////////////////////////////////////////

void raytrace_scene()
{
    std::cout << "Begin rendering scene." << std::endl;

    int w = 640;
    int h = 480;
    MatrixXd R = MatrixXd::Zero(w, h);
    MatrixXd G = MatrixXd::Zero(w, h);
    MatrixXd B = MatrixXd::Zero(w, h);
    MatrixXd A = MatrixXd::Zero(w, h); // Store the alpha mask

    // The camera always points in the direction -z
    // The sensor grid is at a distance 'focal_length' from the camera center,
    // and covers an viewing angle given by 'field_of_view'.
    double aspect_ratio = double(w) / double(h);
    //TODO
    double image_y = tan(field_of_view / 2.0) * focal_length;
    double image_x = image_y * aspect_ratio;

    // The pixel grid through which we shoot rays is at a distance 'focal_length'
    const Vector3d image_origin(-image_x, image_y, camera_position[2] - focal_length);
    const Vector3d x_displacement(2.0 / w * image_x, 0, 0);
    const Vector3d y_displacement(0, -2.0 / h * image_y, 0);

    int iterations = 0;
    for (unsigned i = 0; i < w; ++i)
    {
        //std::cout << "Doing column: " << i  << std::endl;
        for (unsigned j = 0; j < h; ++j)
        {
            const Vector3d pixel_center = image_origin + (i + 0.5) * x_displacement + (j + 0.5) * y_displacement;

            // Prepare the ray
            Vector3d ray_origin;
            Vector3d ray_direction;

            if (is_perspective)
            {
                // Perspective camera
                ray_origin = camera_position;
                ray_direction = (pixel_center - camera_position).normalized();
            }
            else
            {
                // Orthographic camera
                ray_origin = pixel_center;
                ray_direction = Vector3d(0, 0, -1);
            }

            const Vector4d C = shoot_ray(ray_origin, ray_direction);
            R(i, j) = C(0);
            G(i, j) = C(1);
            B(i, j) = C(2);
            A(i, j) = C(3);

        }
    }

    // Save to png
    write_matrix_to_png(R, G, B, A, filename);
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
    setup_scene();

    raytrace_scene();
    return 0;
}
