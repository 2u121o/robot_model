#ifndef RANGEFINDER_H
#define RANGEFINDER_H

#include <iostream>
#include <math.h>

#include <Eigen/Core>

#include "opencv2/opencv.hpp"

class RangeFinder
{

public:

    RangeFinder(double angle_min, double angle_max, double angle_increment, double range_min, double range_max);

    void takeMeasurements(cv::Mat &map, Eigen::Vector2d sensor_pose, Eigen::VectorXd &ranges);

    ~RangeFinder();
private:
    const double angle_min_;
    const double angle_max_;
    const double angle_increment_;
    const double range_min_;
    const double range_max_;

    Eigen::VectorXd ranges_;
    int seq_;                    //incremental value for each meas
    //auto stamp_;                  //timestamp

    int num_meas_;

};



#endif