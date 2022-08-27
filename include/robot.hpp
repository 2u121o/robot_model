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
    Robot(cv::Mat &initiail_map, const Eigen::Vector3d initial_state, int radius, bool add_noise);
    void moveRobot(cv::Mat &map, int direction);
    void takeMeasurementsRange(cv::Mat &map, Eigen::VectorXd &ranges);

    void drawRobot(cv::Mat &map);

    Eigen::Vector3d getStates();
    void getMinPoints(std::vector<Eigen::Vector2d> &min_points);

    ~Robot();

private:
    Eigen::Vector3d state_;   //[x z theta]
    Eigen::Vector3d state_noise_;
    bool with_noise_;
    const double dt_ = 0.2;
    int radius_;
    const int thickness = 1;

    double angle_min_;
    double angle_increment_;

    RangeFinder rangefinder_;
    Eigen::VectorXd ranges_;

    std::default_random_engine generator_;
    std::normal_distribution<double> distribution_;

    std::vector<Eigen::Vector2d> min_points_;

    bool isCollided(cv::Mat &map, Eigen::Vector3d state);

    void drawSensorLine(cv::Mat &map);
};

#endif

