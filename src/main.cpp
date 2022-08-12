#include <iostream>

#include <Eigen/Dense>

#include "robot.hpp"

int main(){
    
    Eigen::Vector3d initial_state;
    initial_state << 100, 100, 3.14/2;
    Robot robot(initial_state, 20);

    cv::Mat map = cv::imread("/home/dario/Workspace/robot_model/map.png");

    robot.drawRobot(map);
    cv::imshow("Map", map);
    int k = cv::waitKey(0);

    while(1){
        map = cv::imread("/home/dario/Workspace/robot_model/map.png");
        if(k==27 || k==-1) return 0;
        robot.moveRobot(map, k);
        robot.drawRobot(map);
        cv::imshow("Map", map);
        k = cv::waitKey(0);
        
    }

    return 0;
}