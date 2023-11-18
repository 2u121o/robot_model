#ifndef WORLD_HPP
#define WORLD_HPP

#include <iostream>

#include <opencv2/opencv.hpp>

#include "Robot.hpp"

class World
{
    public:
        World() = default;

        void drawWorld();

        void setMap(const cv::Mat &map);
        void setRobot(const Robot &robot);

        int getK() const;

    private:
        cv::Mat map_;
        Robot robot_;
        RobotState robot_state_;

        Eigen::VectorXd ranges_;
        SensorSettings sensor_settings_;

        int key_;

        void drawRobot();
        void drawSensorLines();

};


#endif