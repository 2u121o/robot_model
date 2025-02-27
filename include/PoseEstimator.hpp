#ifndef POSE_ESTIMATOR_HPP
#define POSE_ESTIMATOR_HPP

#include <iostream>

#include <Eigen/Dense>

#include "opencv2/opencv.hpp"

#include "typekit.hpp"

class PoseEstimator
{
    public:

        PoseEstimator(const RobotState &initial_state);
        void estimatePose(const int input_k, const Eigen::VectorXd& ranges);

        const RobotState& getStates();

        void setSensorSettings(const SensorSettings &sensor_settings);



    private:

        cv::Mat map_;
        Eigen::VectorXd current_state_;
        RobotState current_state_robot_state_;
        Eigen::VectorXd ranges_;

        SensorSettings sensor_settings_;

        //! Integration step size use to compute the kinematics of the robot.
        const double STEP_SIZE = 0.2;

        void stateTransition(Eigen::VectorXd& state, const int input_k);
        void outputTransition(Eigen::VectorXd& output, const Eigen::VectorXd& state, const int input_k);
};

#endif