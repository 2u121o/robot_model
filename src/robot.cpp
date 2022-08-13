#include "robot.hpp"

Robot::Robot(const Eigen::Vector3d initial_state, int radius, bool add_noise): 
state_(initial_state), 
radius_(radius),  
with_noise_(add_noise)
{
    std::cout << "[ROBOT] constructed" << std::endl;
    state_noise_.setZero();
    distribution_ = std::normal_distribution<double>(0,0.01);

}

void Robot::moveRobot(cv::Mat &map, int direction){
    double u_v = 0;
    double u_t = 0;
    state_noise_.setZero();

    switch (direction)
    {
    case 81:
        u_t = 0.054532925;
        break;
    case 82:   
        u_v = 1;
        break;
    case 83:
        u_t = -0.054532925;
        break;
    case 84:
        u_v = -1;
        break;
    default:
        break;
    }

    Eigen::Vector3d state_tmp;
    
    if(with_noise_){
        for(int k=0; k<3; k++)
            state_noise_(k) = distribution_(generator_);
    }

    //kineamtic model
    state_tmp(0) = state_(0) + dt_*5*std::cos(state_(2))*u_v + state_noise_(0);
    state_tmp(1) = state_(1) - dt_*5*std::sin(state_(2))*u_v + state_noise_(1);
    state_(2) = state_(2) + dt_*u_t + state_noise_(2);
    
    if(!isCollided(map, state_tmp)){
        state_(0) = state_tmp(0);
        state_(1) = state_tmp(1);
    }
    
    
}

bool Robot::isCollided(cv::Mat &map, Eigen::Vector3d state){

    double x_c = state(0);
    double y_c = state(1);
    bool is_collided = false;
    for(int x=-radius_; x<radius_; x++){
        for(int y=-radius_; y<radius_; y++){
            //this if remove the square border
            if(std::pow(x,2)+std::pow(y,2)-std::pow(radius_,2)+0.1 <= 0){ 
                cv::Vec3b &color = map.at<cv::Vec3b>(y+y_c, x+x_c);
                if(color[0]+color[1]+color[2] == 0){
                    is_collided = true;
                }

            }
        }
    }
    return is_collided;
}

cv::Point Robot::getPointRange(cv::Mat &map, int radius, double &range, bool show_beam){

    cv::Point meas_point(-1, -1);
    double min_distance = 10000;
    bool is_measured = false;

    
    //with this first if it avoid core dump in case the robot reach the upper
    //part of the window, in this way the radius is adapted
    double robot_x = state_(0);
    double robot_y = state_(1);
    int radius_x = robot_x<=radius ? robot_x:radius;
    int radius_y = robot_y<=radius ? robot_y:radius;
    

    for(int y=-radius_y; y<radius; y++){
        for(int x=-radius_x; x<radius; x++){
            cv::Vec3b pixel_color = map.at<cv::Vec3b>(robot_y+y, robot_x+x);
            if(pixel_color[0]+pixel_color[1]+pixel_color[2] == 0){
                double tmp_distance = std::sqrt(std::pow(x,2)+std::pow(y,2));
                if(min_distance>tmp_distance){
                    meas_point.x = robot_x+x;
                    meas_point.y = robot_y+y;
                    min_distance = tmp_distance;
                    is_measured = true;
                }
            }
        }
    }

    range = is_measured && with_noise_ ? min_distance+distribution_(generator_):min_distance;

    if(range != 10000 && show_beam){
        cv::Scalar color(255, 255, 0);
        cv::Point start_line(robot_x, robot_y);
        cv::line(map, start_line, meas_point, color, thickness);
    }

    return meas_point;
}

double Robot::getBearing(cv::Point nearest_point){

    double robot_x = state_(0);
    double robot_y = state_(1);
    cv::Point robot_pose(robot_x, robot_y);

    //nearest point in the robot RF
    cv::Point nearest_point_in_robot = nearest_point - robot_pose;

    double bearing = state_(2) + std::atan2(nearest_point_in_robot.y, nearest_point_in_robot.x);
    if(with_noise_){
        double ni = distribution_(generator_);
        bearing += ni;
    }
    return bearing;
}

void Robot::drawRobot(cv::Mat &map){

    cv::Point center(state_(0), state_(1));
    cv::Scalar color(50, 0, 0);
    cv::circle(map, center, radius_, color, thickness);

    double l0_x  = state_(0);
    double l0_y = state_(1);
    double end_line_x = l0_x + radius_*std::cos(state_(2));
    double end_line_y = l0_y - radius_*std::sin(state_(2));
    cv::Point end_line(end_line_x, end_line_y);
    cv::Point start_line(l0_x, l0_y);
    cv::line(map, start_line, end_line, color, thickness);

}

Eigen::Vector3d Robot::getStates(){
    return state_;
}

Robot::~Robot()
{
}