#ifndef RANGEFINDER_H
#define RANGEFINDER_H

#include <iostream>
#include <math.h>

#include <Eigen/Core>

#include "opencv2/opencv.hpp"
#include "typekit.hpp"


class RangeFinder
{

public:
    RangeFinder() = default;
    RangeFinder(const cv::Mat &map, const SensorSettings &sensor_settings);

    void takeMeasurements(const SensorPose &sensor_pose,  Eigen::VectorXd &ranges);

    void getPoints(std::vector<Eigen::Vector2d> &min_points) const;

    void setSensorSettings(const SensorSettings &sensor_settings);

private:
    cv::Mat map_;
    
    SensorSettings sensor_settings_;

    std::vector<Eigen::Vector2d> circ_points_;

    Eigen::VectorXd ranges_;

    int num_meas_;

    std::vector<Eigen::Vector2d> min_points_;

};




#endif