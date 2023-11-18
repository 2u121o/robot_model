#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>
#include <random>

#include <Eigen/Dense>

#include "opencv2/opencv.hpp"

#include "RangeFinder.hpp"

class Robot
{

public:
    Robot(const cv::Mat &initiail_map, const RobotState &initial_state, int radius, bool add_noise);
    void moveRobot(const cv::Mat &map, int direction);
    void takeMeasurementsRange(cv::Mat &map, Eigen::VectorXd &ranges);

    void drawRobot(cv::Mat &map);

    RobotState getStates() const;

    void getMinPoints(std::vector<Eigen::Vector2d> &min_points) const;

private:

    RobotState robot_state_;
    RobotState state_before_collision_check_;
    Eigen::Vector3d state_noise_;
    bool with_noise_;
    const double dt_ = 0.2;
    int radius_;
    const int thickness = 1;

    SensorPose sensor_pose_;
    SensorSettings sensor_settings_;
    RangeFinder rangefinder_;
    Eigen::VectorXd ranges_;

    std::default_random_engine generator_;
    std::normal_distribution<double> distribution_;

    std::vector<Eigen::Vector2d> min_points_;

    bool isCollided(const cv::Mat &map, const RobotState &robot_state);

    void drawSensorLine(cv::Mat &map);
};

#endif

