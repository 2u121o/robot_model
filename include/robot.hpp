#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>

#include <Eigen/Dense>

#include "opencv2/opencv.hpp"

class Robot
{

public:
    Robot(const Eigen::Vector3d initial_state, int radius);
    void moveRobot(cv::Mat &map, int direction);
    void takeMeasurements();
    void drawRobot(cv::Mat &map);

    ~Robot();

private:
    Eigen::Vector3d state_;   //[x z theta]
    const double dt_ = 1;
    int radius_;

    bool isCollided(cv::Mat &map, Eigen::Vector3d state);
};

#endif

