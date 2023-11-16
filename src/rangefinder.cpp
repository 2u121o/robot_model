#include "rangefinder.hpp"


RangeFinder::RangeFinder(){
    std::cout << "[rangefinder] constructed" << std::endl;
}

RangeFinder::RangeFinder(cv::Mat &map, double angle_min, double angle_max, double angle_increment, double range_min, double range_max):
map_(map),
angle_min_(angle_min), angle_max_(angle_max), angle_increment_(angle_increment),
range_min_(range_min), range_max_(range_max)
{
    std::cout << "[rangefinder] constructed" << std::endl;

    num_meas_ = (int)std::floor(std::abs(angle_max_-angle_min_)/angle_increment_);
    
    ranges_.resize(num_meas_);
    seq_ = 0;

    
    circ_points_.resize(num_meas_);
    for(int i=0; i<num_meas_; i++){
        double angle = angle_min_+i*angle_increment_;
        int x = range_max_*std::cos(angle);
        int y = range_max_*std::sin(angle);
        circ_points_[i][0] = x;
        circ_points_[i][1] = y;
    }

    min_points_.resize(num_meas_);


}


void RangeFinder::takeMeasurements(Eigen::Vector2d sensor_pose, double orientation,  Eigen::VectorXd &ranges){
   
    //                     meas_point
    //                     /| meas_point_y
    //          range_max / |  theta->angle between r and b
    //                   /  |
    //                  /___|
    //       sensor_pose    meas_point_x

    
    for(int k=0; k<num_meas_; k++){

        Eigen::Vector2d meas_point = circ_points_[k];
        Eigen::Matrix2d R;
        R << std::cos(orientation), std::sin(orientation),
             -std::sin(orientation), std::cos(orientation);

        Eigen::Vector2d t;
        t << sensor_pose[0], sensor_pose[1];

        meas_point = R*meas_point + t;

        cv::Point meas_point_pt(meas_point[0], meas_point[1]);
        cv::Point sensor_pose_pt(sensor_pose[0], sensor_pose[1]);

        cv::LineIterator it(map_,sensor_pose_pt,  meas_point_pt,  8); 
        int num_points = it.count;
        

        //points along the line
        int x;
        int y;
        bool is_measured = false;
        //bool is_black = false;
        for(int i=0; i<num_points; i++, ++it){
            x = it.pos().x;
            y = it.pos().y;
           
            cv::Vec3b &px_color = map_.at<cv::Vec3b>(y,x);
            
            //whit this 2 if is faster because it starts to count from the end 
            //end i avoid that the sensor line goes on to of the wall 
            //but just at the beginning from the robot side and on the opposite side
            if(px_color[0]+px_color[1]+px_color[2] == 0){
                //is_black = true;
                is_measured = true;
                break;
            }
            // if(is_black && px_color[0]+px_color[1]+px_color[2] == 765){
            //     is_measured = true;
            //     is_black = false;
            //     break;
            // }
        }

        Eigen::Vector2d min_point(x, y);
        min_points_[k] = min_point;
        

        //at this point i have the x and y for which the point is detected
        double new_range = std::sqrt(std::pow(sensor_pose[0]-x,2)+std::pow(sensor_pose[1]-y,2));
        ranges_[k] = is_measured ? new_range:range_max_;

    }
    ranges = ranges_;
}

void RangeFinder::getPoints(std::vector<Eigen::Vector2d> &min_points){
    min_points = min_points_;
}

void RangeFinder::setAngleMin(double angle_min){
    angle_min_ = angle_min;
}
void RangeFinder::setAngleMax(double angle_max){
    angle_max_ = angle_max;
}
void RangeFinder::setAngleIncrement(double angle_increment){
    angle_increment_ = angle_increment;
}
void RangeFinder::setRangeMin(double range_min){
    range_min_ = range_min;
}
void RangeFinder::setRangeMax(double range_max){
    range_max_ = range_max;
}

RangeFinder::~RangeFinder()
{
}