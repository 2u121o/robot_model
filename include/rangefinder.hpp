#ifndef RANGEFINDER_H
#define RANGEFINDER_H

#include <iostream>
#include <math.h>

#include <Eigen/Core>

#include "opencv2/opencv.hpp"


class RangeFinder
{

public:
    RangeFinder() = default;
    RangeFinder(const cv::Mat &map, double angle_min, double angle_max, double angle_increment, double range_min, double range_max);

    void takeMeasurements(const Eigen::Vector2d &sensor_pose, const double orientation,  Eigen::VectorXd &ranges);

    void getPoints(std::vector<Eigen::Vector2d> &min_points) const;

    //create a typekit this is part of the typekit
    void setAngleMin(double angle_min);
    void setAngleMax(double angle_max);
    void setAngleIncrement(double angle_increment);
    void setRangeMin(double range_min);
    void setRangeMax(double range_max);

private:
    cv::Mat map_;
    double angle_min_;
    double angle_max_;
    double angle_increment_;
    double range_min_;
    double range_max_;

    std::vector<Eigen::Vector2d> circ_points_;

    Eigen::VectorXd ranges_;
    //auto stamp_;                  //timestamp

    int num_meas_;

    std::vector<Eigen::Vector2d> min_points_;

};




#endif