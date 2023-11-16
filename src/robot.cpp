#include "robot.hpp"

Robot::Robot(cv::Mat &initial_map, const Eigen::Vector3d initial_state, int radius, bool add_noise): 
state_{initial_state}, 
radius_{radius}, 
with_noise_{add_noise},
map_{initial_map}
{
    std::cout << "[ROBOT] constructed" << std::endl;
    state_noise_.setZero();
    distribution_ = std::normal_distribution<double>(0,0.01);

    //TODO: add the possibility to set the sensor parameters
    angle_min_ = -2.0944;
    double angle_max = 2.0944;
    // angle_min_ = -M_PI;
    // double angle_max = M_PI;
    angle_increment_ = 0.0061;
    double range_min = 0.0010;
    double range_max = 50.0;
    rangefinder_= RangeFinder(initial_map, angle_min_, angle_max, angle_increment_,
                                  range_min, range_max);
    
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
        u_v = 1.0;
        break;
    case 83:
        u_t = -0.054532925;
        break;
    case 84:
        u_v = -1.0;
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
    state_(2) = state_(2) + dt_*2*u_t + state_noise_(2);
    
    if(!isCollided(map, state_tmp)){
        state_(0) = state_tmp(0);
        state_(1) = state_tmp(1);
    }
    
    
}

bool Robot::isCollided(cv::Mat &map, Eigen::Vector3d state){

    int x_c = (int)state(0);
    int y_c = (int)state(1);
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

void Robot::takeMeasurementsRange(cv::Mat &map, Eigen::VectorXd &ranges){
   
    Eigen::Vector2d sensor_pose(state_[0], state_[1]);
    rangefinder_.takeMeasurements(sensor_pose, state_[2], ranges_);
    ranges = ranges_;
    rangefinder_.getPoints(min_points_);
    drawSensorLine(map);

}

void Robot::drawSensorLine(cv::Mat &map){

    cv::Point current_pt(state_[0], state_[1]);
    cv::Scalar color(255, 224, 20);

    double orientation = state_[2];
    int num_ranges = ranges_.size();    
    for(int i=0; i<num_ranges; i++){
        double angle =  angle_min_+(i*angle_increment_);
        double x = (ranges_[num_ranges-i-1]*std::cos(angle));
        double y = (-ranges_[num_ranges-i-1]*std::sin(angle));

        Eigen::Vector2d meas_point(x,y);
        Eigen::Matrix2d R;
        R << std::cos(orientation), std::sin(orientation),
             -std::sin(orientation), std::cos(orientation);

        Eigen::Vector2d t;
        t << state_[0], state_[1];
        meas_point = R*meas_point + t;

        cv::Point meas_point_pt((int)meas_point[0],(int)meas_point[1]);
        cv::line(map, current_pt, meas_point_pt, color, thickness);

    }
}


void Robot::drawRobot(cv::Mat &map){

    //draws the robot
    cv::Point center(state_(0), state_(1));
    cv::Scalar color(100, 0, 0);
    cv::circle(map, center, radius_, color, thickness);

    //draws the orientation line
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

 void Robot::getMinPoints(std::vector<Eigen::Vector2d> &min_points){
    min_points = min_points_;
}

Robot::~Robot()
{
}