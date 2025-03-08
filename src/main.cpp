#include <iostream>
#include <random>
#include <unistd.h>

#include <Eigen/Dense>

#include "Robot.hpp"
#include "typekit.hpp"
#include "World.hpp"
#include "PoseEstimator.hpp"

int main(){
    double radius = 10.0;
    RobotState initial_state;
    initial_state.x = 50.0;
    initial_state.y = 100.0;
    initial_state.theta = 0.0;

    RobotState initial_state_estimation;
    initial_state_estimation.x = 1150.0;
    initial_state_estimation.y = 800.0;
    initial_state_estimation.theta = 0.0;

    std::string paht_map = "../map_new_obs.png";

    cv::Mat map = cv::imread(paht_map);
    Robot robot(map, initial_state, radius, true);

    Robot robot_estimation(map, initial_state_estimation, radius, false);

    std::vector<Robot> robots = {robot, robot_estimation};

    int k;

    World world;
    world.setMap(map);
    world.setRobot(robots);
    world.drawWorld(0);
    k = world.getK();

    PoseEstimator pose_estimator(initial_state_estimation);
    SensorSettings sensor_settings;
    robot.getSensorSettings(sensor_settings);
    pose_estimator.setSensorSettings(sensor_settings);
    
    Eigen::VectorXd ranges; 
    Eigen::VectorXd measurements; 
    Eigen::VectorXd ranges_estimation;
    std::vector<Eigen::Vector2d> min_points;
    while(1)
    {
       
        map = cv::imread(paht_map);
        if(k==27 || k==-1) return 0;
        robots.at(0).moveRobot(map, k);
        robots.at(0).takeMeasurementsRange(map, ranges);
        RobotState robot_state = robots.at(0).getStates();
        // std::cout << ranges.transpose() << std::endl;

        measurements.setZero(2*ranges.size());
    
        for(int i=0, k=0; i<2*ranges.size(); i+=2)
        {
            
            double angle = sensor_settings.angle_min+(k*sensor_settings.angle_increment);
            double d = ranges(k);
            if(i%2==0) ++k;
            double x_d = (d*std::cos(angle));
            double y_d = (d*std::sin(angle));

            Eigen::Vector2d pose_d(x_d, y_d);
            Eigen::Vector2d pose_d_robot_frame;
            Eigen::Matrix2d R;
            R << std::cos(robot_state.theta), std::sin(robot_state.theta),
                -std::sin(robot_state.theta), std::cos(robot_state.theta);
            pose_d_robot_frame = R*pose_d + Eigen::Vector2d(robot_state.x, robot_state.y);

            measurements(i) = pose_d_robot_frame(0);
            measurements(i+1) = pose_d_robot_frame(1);
        }
        
        
        robots.at(1).takeMeasurementsRange(map, ranges_estimation);
   
        pose_estimator.estimatePose(k, ranges_estimation, measurements);
        RobotState robot_state_estimation = pose_estimator.getStates();
        if(!std::isnan(robot_state_estimation.x) && !std::isnan(robot_state_estimation.y) && !std::isnan(robot_state_estimation.theta))
        {
            robots.at(1).setStates(robot_state_estimation);
        }
        
        std::cout << "Estimation: " << pose_estimator.getStates().x << " " << pose_estimator.getStates().y << " " << pose_estimator.getStates().theta << std::endl;
        

        robots.at(0).getMinPoints(min_points);
        world.setMap(map);
        world.setRobot(robots);
        world.drawWorld(0);
        k = world.getK();
    }

    return 0;
}