#include <iostream>
#include <random>

#include <Eigen/Dense>

#include "robot.hpp"

int main(){

    

    
    Eigen::Vector3d initial_state;
    initial_state << 100, 100, 0;
    

    std::string paht_map = "../map.png";

    cv::Mat map = cv::imread(paht_map);
    Robot robot(map, initial_state, 20, false);


    robot.drawRobot(map);
    cv::imshow("Map", map);
    int k = cv::waitKey(0);
    
    Eigen::VectorXd ranges; 
    while(1){
   
        // std::cout << distribution(generator) << std::endl;
        map = cv::imread(paht_map);
        if(k==27 || k==-1) return 0;
        robot.moveRobot(map, k);
        robot.takeMeasurementsRange(map, ranges);
        std::cout << ranges.transpose() << std::endl;
        robot.drawRobot(map);
        cv::imshow("Map", map);
        k = cv::waitKey(0);
        
    }

    return 0;
}