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

#define PI 3.14159265

double inline det(const Point &u, const Point &v) {
	return u.real()*v.imag() - u.imag()*v.real();
}

double inline absangle(Point &a) {
	double theta = atan2(a.imag(),a.real());
	if(theta < 0) {
		return 2*PI - theta;
	}
	return theta;
}


struct Compare {
	Point p0; // Leftmost point of the poly
	bool operator ()(const Point &p1, const Point &p2) {
		Point p01 = Point(p1.real() - p0.real(), p1.imag() - p0.imag());
		Point p02 = Point(p2.real() - p0.real(), p2.imag() - p0.imag());

		double theta1 = absangle(p01);
		double theta2 = absangle(p02);
		return theta1 < theta2;
	}
};


bool inline salientAngle(Point &a, Point &b, Point &c) {
	Point ab = Point(b.real() - a.real(), b.imag() - a.imag());
	Point bc = Point(c.real() - b.real(), c.imag() - b.imag());
	double z_component = det(ab, bc);
	return z_component > 0;
}

////////////////////////////////////////////////////////////////////////////////

Polygon convex_hull(std::vector<Point> &points) {
	Compare order;
	Point lowest = *std::min_element(points.begin(), points.end(), [](Point &p1, Point &p2) { return p1.imag() < p2.imag(); } );
	for(int i = 0; i < points.size(); i++ ) {
		if(points[i].imag() == lowest.imag() && points[i].real() < lowest.real()) {
			lowest = points[i];
		}
	}
	order.p0 = lowest;

	std::sort(points.begin(), points.end(), order);

	Polygon hull;

	for (const auto& v : points) {
		if(hull.size() < 3) {
			hull.push_back(v);
			continue;
		}
		
		while(hull.size() > 2 && !salientAngle(*(hull.end()-3),*(hull.end()-2),*(hull.end()-1))) {
			hull.erase(hull.end()-2);
		}

		hull.push_back(v);
	}

	return hull;
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

void save_obj(const std::string &filename, Polygon &poly) {
	std::ofstream out(filename);
	if (!out.is_open()) {
		throw std::runtime_error("failed to open file " + filename);
	}
	out << std::fixed;
	for (const auto &v : poly) {
		out << "v " << v.real() << ' ' << v.imag() << " 0\n";
	}
	for (size_t i = 0; i < poly.size(); ++i) {
		out << "l " << i+1 << ' ' << 1+(i+1)%poly.size() << "\n";
	}
	out << std::endl;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char * argv[]) {
	if (argc <= 2) {
		std::cerr << "Usage: " << argv[0] << " points.xyz output.obj" << std::endl;
	}
	std::vector<Point> points = load_xyz(argv[1]);
	Polygon hull = convex_hull(points);
	save_obj(argv[2], hull);
	return 0;
}
