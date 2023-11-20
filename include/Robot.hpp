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

    /**
     * @brief Construct a new empty Robot object
     * 
     */
    Robot() = default;

    /**
     * @brief Construct a new Robot object initializing the map, the initial state and the characteristic of the robot.
     * 
     * @param initiail_map is the basic map where the robot lives.
     * @param initial_state is a struct containing the Cartesian position and orientation of the robot.
     * @param radius is a scalar representing the radius of the robot. 
     * @param add_noise is a flag to check whether the noise is present or not. Can be used to 
     *                  make the simulation more realistic.
     */
    Robot(const cv::Mat &initial_map, const RobotState &initial_state, int radius, bool add_noise);

    /**
     * @brief Move the robot based on the direction. To move the robot the arrows on the keyboard can be
     *        used or simulated just passing the respective number.
     *        - 81 counterclockwise rotation 
     *        - 82 move forward
     *        - 83 clockwise rotation
     *        - 84 move backward
     * 
     * @param map is the current map.
     * @param direction is an integer representing the robot direction.
     */
    void moveRobot(const cv::Mat &map, int direction);

    /**
     * @brief Take the range measurements using the simulated range finder.  
     * 
     * @param map is the current map.
     * @param ranges is a vector containing the current measurements, one for
     *               each laser beam.
     */
    void takeMeasurementsRange(cv::Mat &map, Eigen::VectorXd &ranges);

    /**
     * @brief Get the state of the robot.
     * 
     * @return RobotState is a struct containing the current Cartesian position and 
     *         orientation of the robot.
     */
    RobotState getStates() const;
    
    /**
     * @brief Get the radius of the robot.
     * 
     * @return int representing the radius of the robot.
     */
    int getRadius() const;

    /**
     * @brief Get the vector containing 2D points representing the Cartesian
     *        position of the obstacle.
     * 
     * @param min_points vector of 2D points representing the Cartesian position
     *                    of the obstacle. 
     */
    void getMinPoints(std::vector<Eigen::Vector2d> &min_points) const;

    /**
     * @brief Get the current sensor settings. 
     * 
     * @param sensor_settings is a struct containing the basic settings of the sensor.
     */
    void getSensorSettings(SensorSettings &sensor_settings) const;

    /**
     * @brief Get the vector containing the measurements. 
     * 
     * @param ranges is a vector containing the distance between the sensor and the obstacle. 
     *               if the laser beam does not hit any obstacle it returns the maximum distance,
     *               otherwise it returns the distance between the obstacle and the sensor.
     */
    void getRanges(Eigen::VectorXd &ranges) const;

    Robot& operator=(const Robot &robot);

private:

    //! Current state of the robot. (Cartesian position and orientation)
    RobotState robot_state_;

    //! State of the robot before the map update.
    RobotState state_before_collision_check_;

    //! Noise added to the state in case the flag with_noise_ is set on true.
    Eigen::Vector3d state_noise_;

    //! Flag to check whether the noise is added to the robot state or not.
    bool with_noise_;

    //! Integration step size use to compute the kinematics of the robot.
    const double STEP_SIZE = 0.2;

    //! Robot radius.
    int radius_;
    
    //! Current pose of the sensor.
    SensorPose sensor_pose_;

    //! Current settings of the sensor.
    SensorSettings sensor_settings_;

    //! Object representing the sensor.
    RangeFinder rangefinder_;

    //! Vector containing the measurements.
    Eigen::VectorXd ranges_;

    //! Random number generator used to generate the noise.
    std::default_random_engine generator_;

    //! Norma distribution used for the noise.
    std::normal_distribution<double> distribution_;

    //! Vector containing the minimum position between the sensor and the obstacles.
    std::vector<Eigen::Vector2d> min_points_;

    //! Check whether the robot collided with the environment or not.
    bool isCollided(const cv::Mat &map, const RobotState &robot_state);

};

#endif

