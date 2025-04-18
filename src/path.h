#ifndef PATH_H
#define PATH_H

#include <vector>
#include <string>

// This class represents a path (or trajectory) that must be followed
// It is defined by a sequence of points in 3D space
// That are interpolated using a cubic spline

class Path {

public:
    Path(unsigned int n_segments);
    ~Path();
    void addPoint(const double p[9]);
    void getPoint(double p[3], double t) const;
    void getAnchor(double p[3], int i) const;
    void getLeftControl(double a[3], int i) const;
    void getRightControl(double a[3], int i) const;
    int getNumAnchors() const { return points_.size(); }
    int getNumSegments() const { return n_segments_; }
    std::vector<double> getCurve() const;
	int loadFromFile(std::string &path);

private:
    class Point {
    public:
        Point(const double d[9]) : x(d[0]), y(d[1]), z(d[2]), ax(d[3]), ay(d[4]), az(d[5]), bx(d[6]), by(d[7]), bz(d[8]) {}
        double x, y, z;
        double ax, ay, az;
        double bx, by, bz;
    };
    static void lerp(double p[3], const double p0[3], const double p1[3], double t) ;
    std::vector<Point> points_;
    unsigned int n_segments_;
    std::vector<double> curve_;

};

#endif // PATH_H
