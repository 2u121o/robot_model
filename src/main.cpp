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


    // for(int i = 0; i<800; i++){
    //     cv::Vec3b &px_color = map.at<cv::Vec3b>(100,i);
    //     //std::cout << px_color[0]+px_color[1]+px_color[2] << std::endl;
    //     //if(px_color[0]+px_color[1]+px_color[2]  == 0){
    //         px_color[2] = 255;
    //     //}
    // }

    robot.drawRobot(map);
    cv::imshow("Map", map);
    int k = cv::waitKey(0);
    

    while(1){
   
        // std::cout << distribution(generator) << std::endl;
        map = cv::imread(paht_map);
        if(k==27 || k==-1) return 0;
        robot.moveRobot(map, k);
        double range;
        // cv::Point min_point = robot.getPointRange(map, 50, range, true);
        // double bearing = robot.getBearing(min_point);
        robot.takeMeasurementsRange(map);
        robot.drawRobot(map);
        cv::imshow("Map", map);
        k = cv::waitKey(0);
        
    }

    return 0;
}