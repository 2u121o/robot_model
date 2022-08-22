#ifndef RANGEFINDER_H
#define RANGEFINDER_H

#include <iostream>
#include <math.h>

#include <Eigen/Core>

#include "opencv2/opencv.hpp"


class RangeFinder
{

public:
    RangeFinder();
    RangeFinder(double angle_min, double angle_max, double angle_increment, double range_min, double range_max);

    void takeMeasurements(cv::Mat &map, Eigen::Vector2d sensor_pose, double orientation,  Eigen::VectorXd &ranges);

    ~RangeFinder();
private:
    double angle_min_;
    double angle_max_;
    double angle_increment_;
    double range_min_;
    double range_max_;

    Eigen::VectorXd ranges_;
    int seq_;                    //incremental value for each meas
    //auto stamp_;                  //timestamp

    int num_meas_;

};




#endif