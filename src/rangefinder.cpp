#include "rangefinder.hpp"


RangeFinder::RangeFinder(const cv::Mat &map, const SensorSettings &sensor_settings):
map_{map},
sensor_settings_{sensor_settings}
{
    std::cout << "[rangefinder] constructed" << std::endl;

    num_meas_ = static_cast<int>(std::floor(
                                 std::abs(sensor_settings.angle_max-sensor_settings.angle_min)/sensor_settings.angle_increment));
    
    ranges_.resize(num_meas_);

    circ_points_.resize(num_meas_);
    for(int i=0; i<num_meas_; i++)
    {
        double angle = sensor_settings.angle_min+i*sensor_settings.angle_increment;
        int x = sensor_settings.range_max*std::cos(angle);
        int y = sensor_settings.range_max*std::sin(angle);
        circ_points_[i][0] = x;
        circ_points_[i][1] = y;
    }

    min_points_.resize(num_meas_);
}


void RangeFinder::takeMeasurements(const SensorPose &sensor_pose, Eigen::VectorXd &ranges)
{
   
    //                     meas_point
    //                     /| meas_point_y
    //          range_max / |  theta->angle between r and b
    //                   /  |
    //                  /___|
    //       sensor_pose    meas_point_x

    
    for(int k=0; k<num_meas_; k++)
    {

        Eigen::Vector2d meas_point = circ_points_[k];
        Eigen::Matrix2d R;
        R << std::cos(sensor_pose.theta), std::sin(sensor_pose.theta),
             -std::sin(sensor_pose.theta), std::cos(sensor_pose.theta);

        Eigen::Vector2d t;
        t << sensor_pose.x, sensor_pose.y;

        meas_point = R*meas_point + t;

        cv::Point meas_point_pt(meas_point[0], meas_point[1]);
        cv::Point sensor_pose_pt(sensor_pose.x, sensor_pose.y);

        cv::LineIterator it(map_,sensor_pose_pt,  meas_point_pt,  8); 
        int num_points = it.count;
    
        //points along the line
        int x;
        int y;
        bool is_measured = false;
        //bool is_black = false;
        for(int i=0; i<num_points; i++, ++it)
        {
            x = it.pos().x;
            y = it.pos().y;
           
            const cv::Vec3b &px_color = map_.at<cv::Vec3b>(y,x);
            
            //whit this 2 if is faster because it starts to count from the end 
            //end i avoid that the sensor line goes on to of the wall 
            //but just at the beginning from the robot side and on the opposite side
            if(px_color[0]+px_color[1]+px_color[2] == 0)
            {
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
        double new_range = std::sqrt(std::pow(sensor_pose.x-x,2)+std::pow(sensor_pose.y-y,2));
        ranges_[k] = is_measured ? new_range:sensor_settings_.range_max;

    }
    ranges = ranges_;
}

void RangeFinder::getPoints(std::vector<Eigen::Vector2d> &min_points) const
{
    min_points = min_points_;
}

void RangeFinder::setSensorSettings(const SensorSettings &sensor_settings)
{
    sensor_settings_ = sensor_settings;
}