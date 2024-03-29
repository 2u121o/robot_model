#include "Robot.hpp"

Robot::Robot(const cv::Mat &initial_map, const RobotState &initial_state, int radius, bool add_noise): 
robot_state_{initial_state}, 
radius_{radius}, 
with_noise_{add_noise}
{
    std::cout << "[ROBOT] constructed" << std::endl;
    state_noise_.setZero();
    distribution_ = std::normal_distribution<double>(0,0.01);

    sensor_settings_.angle_min = -2.0944;
    sensor_settings_.angle_max = 2.0944;
    sensor_settings_.angle_increment = 0.0061;
    sensor_settings_.range_min = 0.0010;
    sensor_settings_.range_max = 50.0;
    rangefinder_= RangeFinder(initial_map, sensor_settings_);
    sensor_pose_.x = robot_state_.x;
    sensor_pose_.y = robot_state_.y;
    sensor_pose_.theta = robot_state_.theta;
    
}

void Robot::moveRobot(const cv::Mat &map, int direction)
{
    double u_v = 0;
    double u_t = 0;

    switch (direction)
    {
        case 81:
            u_t = 0.054532925;
            break;
        case 82:   
            u_v = 1.5;
            break;
        case 83:
            u_t = -0.054532925;
            break;
        case 84:
            u_v = -1.5;
            break;
        default:
            break;
    }

    if(with_noise_){
        for(int k=0; k<3; k++)
            state_noise_(k) = distribution_(generator_);
    }

    state_before_collision_check_.x = robot_state_.x + STEP_SIZE*5*std::cos(robot_state_.theta)*u_v + state_noise_(0);
    state_before_collision_check_.y = robot_state_.y - STEP_SIZE*5*std::sin(robot_state_.theta)*u_v + state_noise_(1);
    robot_state_.theta = robot_state_.theta + STEP_SIZE*2*u_t + state_noise_(2);
    
    if(!isCollided(map, state_before_collision_check_))
    {
        robot_state_.x = state_before_collision_check_.x;
        robot_state_.y = state_before_collision_check_.y;
    }
}

bool Robot::isCollided(const cv::Mat &map, const RobotState &robot_state)
{
    int x_c = static_cast<int>(robot_state.x);
    int y_c = static_cast<int>(robot_state.y);
    bool is_collided = false;
    for(int x=-radius_; x<radius_; x++){
        for(int y=-radius_; y<radius_; y++){
            if(std::pow(x,2)+std::pow(y,2)-std::pow(radius_,2)+0.1 <= 0){ 
                const cv::Vec3b &color = map.at<cv::Vec3b>(y+y_c, x+x_c);
                if(color[0]+color[1]+color[2] == 0){
                    is_collided = true;
                }
            }
        }
    }
    return is_collided;
}

void Robot::takeMeasurementsRange(cv::Mat &map, Eigen::VectorXd &ranges)
{
    sensor_pose_.x = robot_state_.x;
    sensor_pose_.y = robot_state_.y;
    sensor_pose_.theta = robot_state_.theta;
    rangefinder_.takeMeasurements(sensor_pose_, ranges_);
    ranges = ranges_;
    rangefinder_.getPoints(min_points_);
}

RobotState Robot::getStates() const
{
    return robot_state_;
}

int Robot::getRadius() const
{
    return radius_;    
}

 void Robot::getMinPoints(std::vector<Eigen::Vector2d> &min_points) const
 {
    min_points = min_points_;
}

void Robot::getSensorSettings(SensorSettings &sensor_settings) const
{
    sensor_settings = sensor_settings_;
}

void Robot::getRanges(Eigen::VectorXd &ranges) const
{
    ranges = ranges_;
}

Robot& Robot::operator=(const Robot &robot)
{
    if(this==&robot)
    {
        return *this;
    }
    
    robot_state_ = robot.robot_state_;
    with_noise_ = robot.with_noise_;
    radius_ = robot.radius_;
    sensor_settings_ = robot.sensor_settings_;
    rangefinder_ = robot.rangefinder_;
    distribution_ = robot.distribution_;
    ranges_  = robot.ranges_;
    return *this;
}