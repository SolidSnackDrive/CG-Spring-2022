// C++ include
#include <iostream>
#include <string>
#include <vector>
#include <complex>

// Utilities for the Assignment
#include "utils.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

// Shortcut to avoid Eigen:: everywhere, DO NOT USE IN .h
using namespace Eigen;
typedef std::complex<double> Point;

void raytrace_sphere()
{
    std::cout << "Simple ray tracer, one sphere with orthographic projection" << std::endl;

    const std::string filename("sphere_orthographic.png");
    MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

    // The camera is orthographic, pointing in the direction -z and covering the
    // unit square (-1,1) in x and y
    Vector3d origin(-1, 1, 1);
    Vector3d x_displacement(2.0 / C.cols(), 0, 0);
    Vector3d y_displacement(0, -2.0 / C.rows(), 0);

    // Single light source
    const Vector3d light_position(-1, 1, 1);

    for (unsigned i = 0; i < C.cols(); ++i)
    {
        for (unsigned j = 0; j < C.rows(); ++j)
        {
            // Prepare the ray
            Vector3d ray_origin = origin + double(i) * x_displacement + double(j) * y_displacement;
            Vector3d ray_direction = RowVector3d(0, 0, -1);

            // Intersect with the sphere
            // NOTE: this is a special case of a sphere centered in the origin and for
            // orthographic rays aligned with the z axis
            Vector2d ray_on_xy(ray_origin(0), ray_origin(1));
            const double sphere_radius = 0.9;

            if (ray_on_xy.norm() < sphere_radius)
            {
                // The ray hit the sphere, compute the exact intersection point
                Vector3d ray_intersection(
                    ray_on_xy(0), ray_on_xy(1),
                    sqrt(sphere_radius * sphere_radius - ray_on_xy.squaredNorm()));

                // Compute normal at the intersection point
                Vector3d ray_normal = ray_intersection.normalized();

                // Simple diffuse model
                C(i, j) = (light_position - ray_intersection).normalized().transpose() * ray_normal;

                // Clamp to zero
                C(i, j) = std::max(C(i, j), 0.);

                // Disable the alpha mask for this pixel
                A(i, j) = 1;
            }
        }
    }

    // Save to png
    write_matrix_to_png(C, C, C, A, filename);
}

bool uvt_check(double u, double v, double t) {
    return t > 0 && u >= 0 && v >= 0 && (u + v <= 1);
}

void raytrace_parallelogram()
{
    std::cout << "Simple ray tracer, one parallelogram with orthographic projection" << std::endl;
    
    const std::string filename("plane_orthographic.png");
    MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

    

    // The camera is orthographic, pointing in the direction -z and covering the
    // unit square (-1,1) in x and y
    Vector3d origin(-1, 1, 1);
    Vector3d x_displacement(2.0 / C.cols(), 0, 0);
    Vector3d y_displacement(0, -2.0 / C.rows(), 0);

    // TODO: Parameters of the parallelogram (position of the lower-left corner +
    // two sides)
    Vector3d pgram_origin (-0.5,-0.5,0);
    Vector3d pgram_u (0,0.7,-10);
    Vector3d pgram_v (1,0.4,0);

    Vector3d planeNormal = pgram_u.cross(pgram_v);
    double planeDist = -planeNormal.dot(pgram_origin);

                //double t = -(planeNormal.dot(ray_origin) + planeDist) / planeNormal.dot(ray_vector);
            //Vector3d intersect = ray_origin + t*ray_vector;

    // Single light source
    const Vector3d light_position(-1, 1, 1);

    for (unsigned i = 0; i < C.cols(); ++i)
    {
        for (unsigned j = 0; j < C.rows(); ++j)
        {
            // Prepare the ray

            Vector3d ray_origin = origin + double(i) * x_displacement + double(j) * y_displacement;
            Vector3d ray_direction = RowVector3d(0, 0, -1);

            Vector3d ray_vector = Vector3d(ray_origin(0),ray_origin(1),ray_direction(2) * 999) - ray_origin;

            // TODO: Check if the ray intersects with the parallelogram

            /*;
                a + u(b-a) + v(c-a) = e + td
                u(b-a) + v(c-a) - td = (e - a)
            */

            Matrix3d linA;
            Vector3d n_ray_vector = -1 * ray_vector;
            linA << pgram_u, pgram_v, n_ray_vector;

            Vector3d pgram_other_origin = pgram_origin + pgram_v + pgram_u;

            Vector3d linb = ray_origin - pgram_origin;

            Vector3d linx = linA.partialPivLu().solve(linb);

            Matrix3d linC;
            linC << -1 * pgram_u, -1 * pgram_v, n_ray_vector;

            Vector3d lind = ray_origin - pgram_other_origin;

            lind = linC.partialPivLu().solve(lind);
           //  std::cerr << "Point (" << i << ", " << j << " ) -- " << linx << "\n\n";

            bool inter1 = uvt_check(linx(0),linx(1),linx(2));
            bool inter2 = uvt_check(lind(0),lind(1),lind(2));

            if (inter1 || inter2) /* t > 0, u,v >= 0, u + v <= 1 */
            {
                // std::cerr << "HIT!!\n";
                // TODO: The ray hit the parallelogram, compute the exact intersection
                // point
                Vector3d ray_intersection = inter1 ? linx(2) * ray_vector : lind(2) * ray_vector;

                // TODO: Compute normal at the intersection point
                Vector3d ray_normal = -1 * pgram_u.normalized().cross(pgram_v.normalized());

                // Simple diffuse model
                C(i, j) = (light_position - ray_intersection).normalized().transpose() * ray_normal;

                // Clamp to zero
                C(i, j) = std::max(C(i, j), 0.);

                // Disable the alpha mask for this pixel
                A(i, j) = 1;
            }
        }
    }

    // Save to png
    write_matrix_to_png(C, C, C, A, filename);
}

void raytrace_perspective()
{
    std::cout << "Simple ray tracer, one parallelogram with perspective projection" << std::endl;

    const std::string filename("plane_perspective.png");
    MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

    Vector3d camera_origin(0,0,3);

    // The camera is perspective, pointing in the direction -z and covering the
    // unit square (-1,1) in x and y
    Vector3d image_origin(-1, 1, 1);
    Vector3d x_displacement(2.0 / C.cols(), 0, 0);
    Vector3d y_displacement(0, -2.0 / C.rows(), 0);
    

    // TODO: Parameters of the parallelogram (position of the lower-left corner +
    // two sides)
    Vector3d pgram_origin (-0.5,-0.5,0);
    Vector3d pgram_u (0,0.7,-10);
    Vector3d pgram_v (1,0.4,0);

    // Single light source
    const Vector3d light_position(-1, 1, 1);

    for (unsigned i = 0; i < C.cols(); ++i)
    {
        for (unsigned j = 0; j < C.rows(); ++j)
        {
            // TODO: Prepare the ray (origin point and direction)
            Vector3d ray_origin = image_origin + double(i) * x_displacement + double(j) * y_displacement;
            Vector3d ray_direction = ray_origin - camera_origin;

            //Vector3d ray_vector = Vector3d(ray_origin(0),ray_origin(1),ray_direction(2) * 999) - ray_origin;

            // TODO: Check if the ray intersects with the parallelogram

            /*;
                a + u(b-a) + v(c-a) = e + td
                u(b-a) + v(c-a) - td = (e - a)
            */

            Matrix3d linA;
            Vector3d n_ray_vector = -1 * ray_direction.normalized();
            linA << pgram_u, pgram_v, n_ray_vector;

            Vector3d pgram_other_origin = pgram_origin + pgram_v + pgram_u;

            Vector3d linb = camera_origin - pgram_origin;

            Vector3d linx = linA.partialPivLu().solve(linb);

            Matrix3d linC;
            linC << -1 * pgram_u, -1 * pgram_v, n_ray_vector;

            Vector3d lind = camera_origin - pgram_other_origin;

            lind = linC.partialPivLu().solve(lind);
           //  std::cerr << "Point (" << i << ", " << j << " ) -- " << linx << "\n\n";

            bool inter1 = uvt_check(linx(0),linx(1),linx(2));
            bool inter2 = uvt_check(lind(0),lind(1),lind(2));

            if (inter1 || inter2) /* t > 0, u,v >= 0, u + v <= 1 */
            {
                // TODO: The ray hit the parallelogram, compute the exact intersection
                // point
                Vector3d ray_intersection = inter1 ? camera_origin + linx(2) * ray_direction.normalized() : camera_origin + lind(2) * ray_direction.normalized();

                // TODO: Compute normal at the intersection point
                Vector3d ray_normal = -1 * pgram_u.normalized().cross(pgram_v.normalized());

                // Simple diffuse model
                C(i, j) = (light_position - ray_intersection).normalized().transpose() * ray_normal;

                // Clamp to zero
                C(i, j) = std::max(C(i, j), 0.);

                // Disable the alpha mask for this pixel
                A(i, j) = 1;
            }
        }
    }

    // Save to png
    write_matrix_to_png(C, C, C, A, filename);
}

void raytrace_shading()
{
    std::cout << "Simple ray tracer, one sphere with different shading" << std::endl;

    const std::string filename("shading.png");
    std::vector<MatrixXd> RGB;
    for(int i = 0; i < 3; i++) RGB.push_back(MatrixXd::Zero(800,800));
    MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

    // The camera is perspective, pointing in the direction -z and covering the
    // unit square (-1,1) in x and y
    Vector3d camera_origin(0,0,3);
    Vector3d image_origin(-1, 1, 1);
    Vector3d x_displacement(2.0 / C.cols(), 0, 0);
    Vector3d y_displacement(0, -2.0 / C.rows(), 0);

    // Single light source
    const Vector3d light_position(-1, 1, 1);
    double ambient = 0.1;
    MatrixXd diffuse = MatrixXd::Zero(800, 800);
    MatrixXd specular = MatrixXd::Zero(800, 800);

    //color params
    const Vector3d diffuse_color(1, 0, 1);
    const double specular_exponent = 100;
    const Vector3d specular_color(0., 0, 1);

    //Sphere params
    Vector3d sphere_center(0,0,0);
    const double sphere_radius = 0.9;

    for (unsigned i = 0; i < C.cols(); ++i)
    {
        for (unsigned j = 0; j < C.rows(); ++j)
        {
            Vector3d ray_origin = image_origin + double(i) * x_displacement + double(j) * y_displacement;
            Vector3d ray_direction = ray_origin - camera_origin;
            ray_direction.normalize();

            // Intersect with the sphere
            // NOTE: this is a special case of a sphere centered in the origin and for
            // orthographic rays aligned with the z axis
            Vector3d L = sphere_center - camera_origin;
            double len_origin_center = L.dot(ray_direction);
            if(len_origin_center < 0) {
                continue;
            }

            double len_center_ray = L.dot(L) - (len_origin_center * len_origin_center);
            if(len_center_ray < 0) {
                continue;
            }

            double len_center_intersect = sqrt(sphere_radius * sphere_radius - len_center_ray);
            double t = len_origin_center - len_center_intersect;
            /*
            std::cerr << "L: " << L << std::endl;
            std::cerr << "L dot L: " << L.dot(L) << std::endl;           
            std::cerr << "len_origin_center: " << len_origin_center << std::endl;
            std::cerr << "len_center_ray: " << len_center_ray << std::endl;
            std::cerr << "len_center_intersect: " << len_center_intersect << std::endl;
            std::cerr << "t: " << t << std::endl;*/

            if (t > 0)
            {
                //std::cerr << "HIT\n";
                // The ray hit the sphere, compute the exact intersection point
                Vector3d ray_intersection = camera_origin + t * ray_direction;

                // Compute normal at the intersection point
                Vector3d ray_normal = ray_intersection - sphere_center;
                ray_normal.normalize();

                // TODO: Add shading parameter here

                //h = v + l / ||v + l||

                Vector3d l = (light_position - ray_intersection).normalized();
                Vector3d v = (ray_origin - ray_intersection).normalized();
                Vector3d h = (v + l) / (v + l).norm();

                diffuse(i, j) = (light_position - ray_intersection).normalized().transpose() * ray_normal;
                specular(i, j) = h.transpose() * ray_normal;

                // Simple diffuse model
                for(int k = 0; k < 3; k++) {
                    RGB[k](i, j) = ambient + diffuse_color(k)*diffuse(i, j) + specular_color(k)*std::pow(specular(i, j), specular_exponent);

                    // Clamp to zero
                    RGB[k](i, j) = std::max(RGB[k](i, j), 0.);
                }


                // Disable the alpha mask for this pixel
                A(i, j) = 1;
            }
        }
    }
    // Save to png
    write_matrix_to_png(RGB[0], RGB[1], RGB[2], A, filename);
}

int main()
{
    raytrace_sphere();
    raytrace_parallelogram();
    raytrace_perspective();
    raytrace_shading();

    return 0;
}
