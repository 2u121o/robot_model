#ifndef WORLD_HPP
#define WORLD_HPP

#include <iostream>

#include <opencv2/opencv.hpp>

#include "Robot.hpp"

class World
{
    public:

        /**
         * @brief Construct a new empty World.
         * 
         */
        World() = default;

        /**
         * @brief Draw the World with the robot and the relative sensors.
         * 
         */
        void drawWorld();

        /**
         * @brief Set the Map object.
         * 
         * @param map is a matrix containing the pixel of the map. 
         */
        void setMap(const cv::Mat &map);

        /**
         * @brief Set the Robot object visualized in the map.
         * 
         * @param robot
         */
        void setRobot(const Robot &robot);

        /**
         * @brief Get the keyboard key pressed from the user.
         * 
         * @return int representing the values associated to a specific key.
         */
        int getK() const;

    private:

        //! Current map where the robot lives.
        cv::Mat map_;

        //! Robot visualized in the map.
        Robot robot_;

        //! State of the robot visualized in the map.
        RobotState robot_state_;

        //! Vector containing the measurements.
        Eigen::VectorXd ranges_;

        //! Settings of the sensor.
        SensorSettings sensor_settings_;

        //! Pressed key from the keyboard.
        int key_;

        //! Utility function to draw the robot on the map.
        void drawRobot();

        //! Utility function to draw the sensor line on the map.
        void drawSensorLines();

};


#endif