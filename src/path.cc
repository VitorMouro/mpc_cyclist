#include "path.h"

#include <cstdio>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

#include "_deps/mujoco-src/src/engine/engine_util_errmem.h"

Path::Path(unsigned int n_segments) : n_segments_(n_segments) {

}

Path::~Path()
= default;

void Path::addPoint(const double p[9])
{
    Point point(p);
    points_.push_back(point);

    if(points_.size() < 2)
        return;

    for(unsigned int i = 0; i <= n_segments_; i++) {
        double t0 = (double)i/n_segments_ + points_.size()-2;
        double a[3];
        getPoint(a, t0);
        curve_.push_back(a[0]);
        curve_.push_back(a[1]);
        curve_.push_back(a[2]);
    }
}

std::vector<double> Path::getCurve() const {
    return curve_;
}

int Path::loadFromFile(std::string& path)
{
	std::ifstream file(path);
	if (file.is_open())
	{
		std::string line;
		while (std::getline(file, line))
		{
			std::istringstream iss(line);
			double p[9];
			for (int i = 0; i < 9; i++)
			{
				std::string val;
				std::getline(iss, val, ',');
				p[i] = std::stod(val);
			}

			addPoint(p);
		}
		file.close();
		std::cout << "Loaded " << getNumAnchors() << " points from " << path << std::endl;
		return 0;
	} else {
		std::cerr << "Unable to open file " << path << std::endl;
		return 1;
	}
}

void print3d(const double p[3]) {
    printf("%f %f %f\n", p[0], p[1], p[2]);
}

void Path::getPoint(double p[3], double t) const
{
    double index;
    double t0 = std::modf(t, &index);
    int i = (int)index;
    if(i >= points_.size() - 1)
    {
        i = points_.size() - 2;
        t0 = 1;
    }

    double p0[3], p1[3], a0[3], a1[3];
    getAnchor(p0, i);
    getAnchor(p1, i + 1);
    getRightControl(a0, i);
    getLeftControl(a1, i+1);

    double p0_a0[3], a1_p1[3], a0_a1[3];
    lerp(p0_a0, p0, a0, t0);
    lerp(a0_a1, a0, a1, t0);
    lerp(a1_p1, a1, p1, t0);

    double p0a0_a0a1[3], a0a1_a1p1[3];
    lerp(p0a0_a0a1, p0_a0, a0_a1, t0);
    lerp(a0a1_a1p1, a0_a1, a1_p1, t0);

    lerp(p, p0a0_a0a1, a0a1_a1p1, t0);
}

void Path::getAnchor(double p[3], int i) const
{
    const Point &point = points_[i];
    p[0] = point.x;
    p[1] = point.y;
    p[2] = point.z;
}

void Path::getLeftControl(double a[3], int i) const
{
    const Point &point = points_[i];
    a[0] = point.ax;
    a[1] = point.ay;
    a[2] = point.az;
}

void Path::getRightControl(double a[3], int i) const
{
    const Point &point = points_[i];
    a[0] = point.bx;
    a[1] = point.by;
    a[2] = point.bz;
}

void Path::lerp(double p[3], const double p0[3], const double p1[3], const double t)
{
    p[0] = p0[0] + t * (p1[0] - p0[0]);
    p[1] = p0[1] + t * (p1[1] - p0[1]);
    p[2] = p0[2] + t * (p1[2] - p0[2]);
}
