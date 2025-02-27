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
    initial_state.x = 100.0;
    initial_state.y = 100.0;
    initial_state.theta = 0.0;

    RobotState initial_state_estimation;
    initial_state_estimation.x = 200.0;
    initial_state_estimation.y = 250.0;
    initial_state_estimation.theta = 0.0;

    std::string paht_map = "../map_new_obs.png";

    cv::Mat map = cv::imread(paht_map);
    Robot robot(map, initial_state, radius, false);

    Robot robot_estimation(map, initial_state_estimation, radius, false);

    std::vector<Robot> robots = {robot, robot_estimation};

    int k;

    World world;
    world.setMap(map);
    world.setRobot(robots);
    world.drawWorld(0);
    k = world.getK();

    PoseEstimator pose_estimator(initial_state_estimation);
    
    Eigen::VectorXd ranges; 
    Eigen::VectorXd ranges_estimation;
    std::vector<Eigen::Vector2d> min_points;
    while(1)
    {
        map = cv::imread(paht_map);
        if(k==27 || k==-1) return 0;
        robots.at(0).moveRobot(map, k);
        robots.at(0).takeMeasurementsRange(map, ranges);
        // std::cout << ranges.transpose() << std::endl;
        
        robots.at(1).takeMeasurementsRange(map, ranges_estimation);
        pose_estimator.estimatePose(k, ranges_estimation);
        robots.at(1).setStates(pose_estimator.getStates());
        

        robots.at(0).getMinPoints(min_points);
        world.setMap(map);
        world.setRobot(robots);
        world.drawWorld(0);
        k = world.getK();
    }

    return 0;
}