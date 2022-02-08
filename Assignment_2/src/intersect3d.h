#ifndef INTERSECT3D_H
#define INTERSECT3D_H

////////////////////////////////////////////////////////////////////////////////
#include <algorithm>
#include <complex>
#include <fstream>
#include <iostream>
#include <numeric>
#include <vector>
#include <Eigen/Dense>
////////////////////////////////////////////////////////////////////////////////

typedef std::complex<double> Point;
typedef std::vector<Point> Polygon;

using namespace Eigen;

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
		//std::cout << "Testing against side (" << poly[i].real() << ", " << poly[i].imag() << ") and (" << poly[i+1].real() << ", " << poly[i +1].imag() << ")\n";
		if(intersect_segment(poly[i], poly[i+1], query, outside, dummy)) intersects++;
	}
	if(intersect_segment(poly[0], poly[poly.size() - 1], query, outside, dummy)) intersects++;
	//std::cout << "Testing against side (" << poly[0].real() << ", " << poly[0].imag() << ") and (" << poly[poly.size() - 1].real() << ", " << poly[poly.size() - 1].imag() << ")\n";
	//std::cout << intersects << " intersects for point(" << query.real() << ", " << query.imag() << ")\n";
	return intersects % 2;
}

#endif