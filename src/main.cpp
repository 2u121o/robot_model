#include <iostream>
#include <random>
#include <unistd.h>

#include <Eigen/Dense>

#include "Robot.hpp"
#include "typekit.hpp"
#include "World.hpp"

int main(){
    
    RobotState initial_state;
    initial_state.x = 100.0;
    initial_state.y = 100.0;
    initial_state.theta = 0.0;

    std::string paht_map = "../map_new_obs.png";

    cv::Mat map = cv::imread(paht_map);
    Robot robot(map, initial_state, 10, false);

    int k;

    World world;
    world.setMap(map);
    world.setRobot(robot);
    world.drawWorld();
    k = world.getK();
    
    Eigen::VectorXd ranges; 
    std::vector<Eigen::Vector2d> min_points;
    while(1){
        
        map = cv::imread(paht_map);
        if(k==27 || k==-1) return 0;
        robot.moveRobot(map, k);
        robot.takeMeasurementsRange(map, ranges);
        std::cout << ranges.transpose() << std::endl;
        robot.getMinPoints(min_points);
        // robot.drawRobot(map);
        world.setMap(map);
        world.setRobot(robot);
        world.drawWorld();
        k = world.getK();
        
    }

    return 0;
}