#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>
#include <random>

#include <Eigen/Dense>

#include "opencv2/opencv.hpp"

#include "rangefinder.hpp"

class Robot
{

public:
    Robot(const Eigen::Vector3d initial_state, int radius, bool add_noise);
    void moveRobot(cv::Mat &map, int direction);
    void takeMeasurementsRange(cv::Mat &map);

    // cv::Point getPointRange(cv::Mat &map, int radius, double &range, bool show_beam);
    // double getBearing(cv::Point nearest_point);
    void drawRobot(cv::Mat &map);

    Eigen::Vector3d getStates();

    ~Robot();

private:
    Eigen::Vector3d state_;   //[x z theta]
    Eigen::Vector3d state_noise_;
    bool with_noise_;
    const double dt_ = 0.2;
    int radius_;
    const int thickness = 1;

    Eigen::VectorXd ranges_;

    std::default_random_engine generator_;
    std::normal_distribution<double> distribution_;

    bool isCollided(cv::Mat &map, Eigen::Vector3d state);

    void drawSensorLine();
};

#endif

