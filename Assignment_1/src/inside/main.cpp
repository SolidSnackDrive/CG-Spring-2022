////////////////////////////////////////////////////////////////////////////////
#include <algorithm>
#include <complex>
#include <fstream>
#include <iostream>
#include <numeric>
#include <vector>
////////////////////////////////////////////////////////////////////////////////

typedef std::complex<double> Point;
typedef std::vector<Point> Polygon;

double inline det(const Point &u, const Point &v) {
	return u.real()*v.imag() - u.imag()*v.real();
}

bool inline ccw(const Point &a, const Point &b, const Point &c) {
	Point ab = Point(b.real() - a.real(),b.imag() - a.imag());
	Point ac = Point(c.real() - a.real(),c.imag() - a.imag());
	return det(ab,ac) > 0;
}

// Return true iff [a,b] intersects [c,d], and store the intersection in ans
bool intersect_segment(const Point &a, const Point &b, const Point &c, const Point &d, Point &ans) {
	return ccw(a,c,d) != ccw(b,c,d) && ccw(a,b,c) != ccw(a,b,d);
}

////////////////////////////////////////////////////////////////////////////////

bool is_inside(const Polygon &poly, const Point &query) {
	// 1. Compute bounding box and set coordinate of a point outside the polygon
	//Point lowestY = *std::min_element(poly.begin(), poly.end(), [](Point &p1, Point &p2) { return p1.imag() < p2.imag(); } );
	//Point lowestX = *std::min_element(poly.begin(), poly.end(), [](Point &p1, Point &p2) { return p1.real() < p2.real(); } );
	double highestY = poly.front().imag();
	double highestX = poly.front().real();
	for(size_t i = 0; i < poly.size(); i++) {
		if(poly[i].imag() > highestY) highestY = poly[i].imag();
		if(poly[i].real() > highestX) highestX = poly[i].real();
	}

	Point outside(highestX + 900, highestY + 900);
	//std::cout << "Highest point was (" << highestX << ", " << highestY << ")\n";
	
	// 2. Cast a ray from the query point to the 'outside' point, count number of intersections
	Point dummy;
	int intersects = 0;
	for(size_t i = 0; i < poly.size() - 1; i++) {
		std::cout << "Testing against side (" << poly[i].real() << ", " << poly[i].imag() << ") and (" << poly[i+1].real() << ", " << poly[i +1].imag() << ")\n";
		if(intersect_segment(poly[i], poly[i+1], query, outside, dummy)) intersects++;
	}
	if(intersect_segment(poly[0], poly[poly.size() - 1], query, outside, dummy)) intersects++;
	std::cout << "Testing against side (" << poly[0].real() << ", " << poly[0].imag() << ") and (" << poly[poly.size() - 1].real() << ", " << poly[poly.size() - 1].imag() << ")\n";
	std::cout << intersects << " intersects for point(" << query.real() << ", " << query.imag() << ")\n";
	return intersects % 2;
}

////////////////////////////////////////////////////////////////////////////////

std::vector<Point> load_xyz(const std::string &filename) {
	std::vector<Point> points;
	std::ifstream in(filename);
	std::string line;
	std::getline(in, line);
	while(std::getline(in,line)) {
		std::istringstream iss(line);
		double a,b,c;
		if (!(iss >> a >> b >> c)) {
			break;
		}
		points.push_back(Point(a,b));
	}
	return points;
}

Polygon load_obj(const std::string &filename) {
	std::ifstream in(filename);
	Polygon poly;
	std::string line;
	if(!in.is_open()) {
		throw std::runtime_error("failed to open file " + filename);
	}

	while(std::getline(in,line)) {
		std::istringstream iss(line);
		char t;
		if(!(iss >> t) || t == 'f') {
			break;
		}
		double a, b, c;
		if(!(iss >> a >> b >> c)) {
			break;
		}

		poly.push_back(Point(a,b));
	}

	for (const auto &v : poly) {
		std::cout << "Read point (" << v.real() << ", " << v.imag() <<  ")\n";
	}
	return poly;
}

void save_xyz(const std::string &filename, const std::vector<Point> &points) {
	std::ofstream out(filename);
	if (!out.is_open()) {
		throw std::runtime_error("failed to create file " + filename);
	}
	out << std::fixed;
	out << points.size() << "\n";
	for (const auto &v : points) {
		out << v.real() << ' ' << v.imag() << " 0\n";
	}
	out << std::endl;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char * argv[]) {
	if (argc <= 3) {
		std::cerr << "Usage: " << argv[0] << " points.xyz poly.obj result.xyz" << std::endl;
	}
	std::vector<Point> points = load_xyz(argv[1]);
	Polygon poly = load_obj(argv[2]);
	std::vector<Point> result;
	for (size_t i = 0; i < points.size(); ++i) {
		if (is_inside(poly, points[i])) {
			result.push_back(points[i]);
		}
	}
	save_xyz(argv[3], result);
	return 0;
}
