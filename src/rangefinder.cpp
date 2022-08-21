#include "rangefinder.hpp"


RangeFinder::RangeFinder(double angle_min, double angle_max, double angle_increment, double range_min, double range_max):
angle_min_(angle_min), angle_max_(angle_max), angle_increment_(angle_increment),
range_min_(range_min), range_max_(range_max)
{
    std::cout << "[rangefinder] constructed";

    num_meas_ = (int)std::floor(std::abs(angle_max_-angle_min_)/angle_increment_);
    ranges_.resize(num_meas_);
    seq_ = 0;
}

void RangeFinder::takeMeasurements(cv::Mat &map, Eigen::Vector2d sensor_pose, Eigen::VectorXd &ranges){
   
    //                     meas_point
    //                     /| meas_point_y
    //          range_max / |  theta->angle between r and b
    //                   /  |
    //                  /___|
    //       sensor_pose    meas_point_x

    double theta = angle_min_;
    for(int k=0; k<num_meas_; k++){
        double meas_point_y = std::sqrt(range_max_*std::tan(theta)/(1+std::tan(theta)));
        double meas_point_x = std::sqrt(range_max_-std::pow(meas_point_y,2));

        cv::Point meas_point(meas_point_x, meas_point_y);
        cv::Point sensor_pose_pt(sensor_pose[0], sensor_pose[1]);
        cv::LineIterator it(map, sensor_pose_pt, meas_point, 8);
        int num_points = it.count;
        std::vector<cv::Point> points_line(num_points);

        //points along the line
        int x;
        int y;
        bool is_measured = false;
        for(int i=0; i<num_points; i++, ++it){
            x = it.pos().x;
            y = it.pos().y;

            cv::Vec3b px_color = map.at<cv::Vec3b>(x,-y);
            if(px_color[0]+px_color[1]+px_color[2] == 0){
                is_measured = true;
                break;
            }
        }

        //at this point i have the x and y for which the point is detected
        double new_range = std::sqrt(std::pow(sensor_pose[0]-x,2)+std::pow(sensor_pose[1]-y,2));
        ranges_[k] = is_measured ? new_range:range_max_;
        theta += angle_increment_;
    }
    ranges = ranges_;
}

RangeFinder::~RangeFinder()
{
}