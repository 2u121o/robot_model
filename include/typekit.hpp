#ifndef TYPEKIT_HPP
#define TYPEKIT_HPP

/**
 * @brief Basic settings of the Range finder.
 * 
 */
struct SensorSettings
{
    //! Minimum angle between the robot direction and the sensor beam.
    double angle_min;

    //! Maximum angle between the robot direction and the sensor beam.
    double angle_max;

    //! Angle increment between two laser beams.
    double angle_increment;

    //! Minimum measurable range.
    double range_min;

    //! Maximum measurable range.
    double range_max;
};

/**
 * @brief State of the robot
 * 
 */
struct RobotState
{
    //! x Cartesian position wrt the World frame.
    double x;

    //! y Cartesian position wrt the World frame.
    double y;

    //! Orientation of the robot wrt to x axis of the World frame.
    double theta;
};

/**
 * @brief Position of the sensor.
 * 
 */
struct SensorPose
{
    //! x Cartesian position.
    double x;

    //! y Cartesian position.
    double y;

    //! Orientation of the sensor.
    double theta;
};



#endif