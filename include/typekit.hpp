#ifndef TYPEKIT_HPP
#define TYPEKIT_HPP

struct SensorSettings
{
    double angle_min;
    double angle_max;
    double angle_increment;
    double range_min;
    double range_max;
};

struct RobotState
{
    double x;
    double y;
    double theta;
};

struct SensorPose
{
    double x;
    double y;
    double theta;
};



#endif