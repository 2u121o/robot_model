#include "World.hpp"

void World::drawWorld(const int visualization_time)
{
    drawRobot();
    drawSensorLines();

    cv::imshow("Map", map_);
    if(visualization_time == 0)
    {
        key_ = cv::waitKey(visualization_time); 
    }
    else
    {
        cv::waitKey(visualization_time); 
    }
}


void World::drawRobot()
{
    cv::Point center(robot_state_.x,robot_state_.y);
    cv::Scalar color(100, 0, 0);
    cv::circle(map_, center, robot_.getRadius(), color, 1);

    int radius = robot_.getRadius();

    double l0_x  = robot_state_.x;
    double l0_y = robot_state_.y;
    double end_line_x = l0_x + radius*std::cos(robot_state_.theta);
    double end_line_y = l0_y - radius*std::sin(robot_state_.theta);
    cv::Point end_line(end_line_x, end_line_y);
    cv::Point start_line(l0_x, l0_y);
    cv::line(map_, start_line, end_line, color, 1);
}

void World::drawSensorLines()
{
    cv::Point current_pt(robot_state_.x, robot_state_.y);
    cv::Scalar color(255, 224, 20);

    double orientation = robot_state_.theta;
    int num_ranges = ranges_.size();    
    for(int i=0; i<num_ranges; i++)
    {
        double angle =  sensor_settings_.angle_min+(i*sensor_settings_.angle_increment);
        double x = (ranges_[num_ranges-i-1]*std::cos(angle));
        double y = (-ranges_[num_ranges-i-1]*std::sin(angle));

        Eigen::Vector2d meas_point(x,y);
        Eigen::Matrix2d R;
        R << std::cos(orientation), std::sin(orientation),
             -std::sin(orientation), std::cos(orientation);

        Eigen::Vector2d t;
        t << robot_state_.x, robot_state_.y;
        meas_point = R*meas_point + t;

        cv::Point meas_point_pt((int)meas_point[0],(int)meas_point[1]);
        cv::line(map_, current_pt, meas_point_pt, color, 1);
    }
}

void World::setMap(const cv::Mat &map)
{
    map_ = map;
}

void World::setRobot(const Robot &robot)
{
    robot_ = robot;
    robot_state_ = robot_.getStates();
    robot_.getRanges(ranges_);
    robot_.getSensorSettings(sensor_settings_);

}

int World::getK() const
{
    return key_;
}