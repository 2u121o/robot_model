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
    RangeFinder(cv::Mat &map, double angle_min, double angle_max, double angle_increment, double range_min, double range_max);


    void takeMeasurements(Eigen::Vector2d sensor_pose, double orientation,  Eigen::VectorXd &ranges);

    void getPoints(std::vector<Eigen::Vector2d> &min_points);

    void setAngleMin(double angle_min);
    void setAngleMax(double angle_max);
    void setAngleIncrement(double angle_increment);
    void setRangeMin(double range_min);
    void setRangeMax(double range_max);


    ~RangeFinder();
private:
    cv::Mat map_;
    double angle_min_;
    double angle_max_;
    double angle_increment_;
    double range_min_;
    double range_max_;

    std::vector<Eigen::Vector2d> circ_points_;

    Eigen::VectorXd ranges_;
    int seq_;                    //incremental value for each meas
    //auto stamp_;                  //timestamp

    int num_meas_;

    std::vector<Eigen::Vector2d> min_points_;

};




#endif