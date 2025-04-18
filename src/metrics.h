//
// Created by vitor on 11/27/24.
//

#ifndef METRICS_H
#define METRICS_H
#include <chrono>
#include <iostream>
#include <absl/strings/str_format.h>

struct Point {
    double x;
    double y;
    double z;
};

typedef std::vector<Point> Points;
typedef std::vector<double> Scalars;

using namespace std::chrono;

class Metrics {

public:

    explicit Metrics(const size_t n_points) {
        _closest_distance.resize(n_points);
        std::ranges::fill(_closest_distance, 0);
    }

    ~Metrics()= default;

    bool updateTrajectoryError(const std::vector<double> &path, const int current_point_i, const double current_distance) {
        const double best_distance = _closest_distance[current_point_i];
        if (current_distance < best_distance || best_distance == 0) {
            _closest_distance[current_point_i] = current_distance;
            return true;
        }
        return false;
    }

    void updateTrajectoryTime(const time_point<steady_clock> startTime, const time_point<steady_clock> endTime) {
        _start_time = startTime;
        _end_time = endTime;
    }

    void updateSuccessRate(const int current_point_i, const int max_i) {
        _final_point_i = current_point_i;
        _max_i = max_i;
    }

    void reset() {
        std::ranges::fill(_closest_distance, 0);
        _start_time = time_point<steady_clock>::min();
        _end_time = time_point<steady_clock>::min();
    }

    void printPoints(Points &points) {
        printf("\"");
        for (int i = 0; i < points.size(); i++) {
            Point p = points[i];
            printf("(%e,%e,%e)", p.x, p.y, p.z);
            if (i < points.size() - 1)
                printf(",");
        }
        printf("\"");
    }

    void print() {
        // printf("metrics: TrajectoryError, TrajectoryTime, FinalPoint, TotalPoints, SiteTrajectory, CentreOfMassTrajectory, EulerAngles, LinearVelocity, AngularVelocity\n");
        printf("metrics: TrajectoryError, TrajectoryTime, FinalPoint, TotalPoints\n");
        printf("data: ");
        printf("%e,", getTrajectoryError());
        printf("%e,", getTrajectoryTime());
        printf("%d,%d", _final_point_i, _max_i);

        // printPoints(_siteTrajectory);
        // printPoints(_centreOfMassTrajectory);
        // printPoints(_eulerAngles);
        // printPoints(_linearVelocity);
        // printPoints(_angularVelocity);

        printf("\n");
    }

    double getTrajectoryError() const {
        double res = 0;
        for (const double dist : _closest_distance) {
           res += dist;
        }
        return res;
    }

    double getTrajectoryTime() const {
        const auto elapsed = _end_time - _start_time;
        return duration_cast<microseconds>(elapsed).count() / 1e6;
    }

    double getSuccessRate() const {
        return static_cast<double>(_final_point_i) / static_cast<double>(_max_i);
    }

    void updateTimeSeriesData(Point site, Point com, Point euler, Point linear, Point angular, Point targetPoint, double control_efford, double time) {
        _siteTrajectory.emplace_back(site);
        _centreOfMassTrajectory.emplace_back(com);
        _eulerAngles.emplace_back(euler);
        _linearVelocity.emplace_back(linear);
        _angularVelocity.emplace_back(angular);
        _targetPoint.emplace_back(targetPoint);
        _controlEfford.emplace_back(control_efford);
        _time.emplace_back(time);
    }

    // Write all the sensor data to a binary ostream and returns the size of each array
    void writeSensorData(std::ostream &os) const {
        unsigned long size = _siteTrajectory.size();
        os.write(reinterpret_cast<const char *>(&size), sizeof(size));
        unsigned long point_size = sizeof(Point);
        os.write(reinterpret_cast<const char *>(&point_size), sizeof(point_size));

        unsigned long data_size = sizeof(Point) * size;
        os.write(reinterpret_cast<const char *>(_siteTrajectory.data()), data_size);
        os.write(reinterpret_cast<const char *>(_centreOfMassTrajectory.data()), data_size);
        os.write(reinterpret_cast<const char *>(_eulerAngles.data()), data_size);
        os.write(reinterpret_cast<const char *>(_linearVelocity.data()), data_size);
        os.write(reinterpret_cast<const char *>(_angularVelocity.data()), data_size);
        os.write(reinterpret_cast<const char *>(_targetPoint.data()), data_size);

        os.write(reinterpret_cast<const char *>(_controlEfford.data()), sizeof(double) * size);
        os.write(reinterpret_cast<const char *>(_time.data()), sizeof(double) * size);
    }

private:

    // Trajectory Error
    std::vector<double> _closest_distance;

    // Trajectory Time
    steady_clock::time_point _start_time = time_point<steady_clock>::min();
    steady_clock::time_point _end_time = time_point<steady_clock>::min();

    // SuccessRate
    int _final_point_i = -1;
    int _max_i = -1;

    // Time series
    Points _siteTrajectory;
    Points _centreOfMassTrajectory;
    Points _eulerAngles;
    Points _linearVelocity;
    Points _angularVelocity;
    Points _targetPoint;

    std::vector<double> _controlEfford;
    std::vector<double> _time;
};



#endif //METRICS_H
