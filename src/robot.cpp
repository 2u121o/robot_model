#include "robot.hpp"

Robot::Robot(const Eigen::Vector3d initial_state, int radius): state_(initial_state), radius_(radius)
{
    std::cout << "[ROBOT] constructed" << std::endl;

}

void Robot::moveRobot(cv::Mat &map, int direction){
    double u_v = 0;
    double u_t = 0;

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

    //kineamtic model
    state_tmp(0) = state_(0) + dt_*5*std::cos(state_(2))*u_v;
    state_tmp(1) = state_(1) - dt_*5*std::sin(state_(2))*u_v;
    state_(2) = state_(2) + dt_*u_t;
    
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

void Robot::drawRobot(cv::Mat &map){

    cv::Point center(state_(0), state_(1));
    cv::Scalar color(50, 0, 0);
    int thickness = 1;
    cv::circle(map, center, radius_, color, thickness);

    double l0_x  = state_(0);
    double l0_y = state_(1);
    double end_line_x = l0_x + radius_*std::cos(state_(2));
    double end_line_y = l0_y - radius_*std::sin(state_(2));
    cv::Point end_line(end_line_x, end_line_y);
    cv::Point start_line(l0_x, l0_y);
    cv::line(map, start_line, end_line, color, thickness);

}

Robot::~Robot()
{
}