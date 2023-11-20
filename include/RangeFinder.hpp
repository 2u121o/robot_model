#ifndef RANGE_FINDER_H
#define RANGE_FINDER_H

#include <iostream>
#include <math.h>

#include <Eigen/Core>

#include "opencv2/opencv.hpp"
#include "typekit.hpp"

/**
 * @class It is a object that simulate a possible 2D lidar. It generates a 
 *        certain number of beams used to measure the the distance between the 
 *        sensor and a possible obstacle. If no obstacle is hit by the beam the
 *        entry associated to that beam return the maximum distance otherwise
 *        it return the distance between the sensor and the obstacle. 
 * 
 */

class RangeFinder
{

public:

    /**
     * @brief Construct a new empty Range Finder
     * 
     */
    RangeFinder() = default;

    /**
     * @brief Construct a new Range Finder object given the map and the sensor settings.
     * 
     * @param map is an opencv object containing a matrix of pixels. Only black pixels are
     *            recognized ad obstacles.
     * @param sensor_settings is a struct containing the basic settings of a range finder. 
     *                         There are the beams angles min and max, the ranges min and max
     *                         and the angle increments between beams. In particular this last
     *                         setting defines the number of beams. 
     */
    RangeFinder(const cv::Mat &map, const SensorSettings &sensor_settings);

    /**
     * @brief it computes the measurements based on the sensor pose and the provided map.
     *        Each beam generates a measurement that it is stored in the ranges vector.
     * 
     * @param sensor_pose is a structure representing the 2D Cartesian pose of the sensor.
     * @param ranges is a num_meas_-dimensional vector containing the measurements. Each entry
     *               corresponds to a laser beam.
     */
    void takeMeasurements(const SensorPose &sensor_pose,  Eigen::VectorXd &ranges);
    
    /**
     * @brief Get the coordinate of the obstacles.
     * 
     * @param min_points is a num_meas_-dimensional vector containing 2D point representing the
     *                   position of the obstacle.
     */
    void getPoints(std::vector<Eigen::Vector2d> &min_points) const;

    /**
     * @brief Set the Sensor Settings structure.
     * 
     * @param sensor_settings is a structure containing the basic sensor settings.
     */
    void setSensorSettings(const SensorSettings &sensor_settings);

private:

    //! OpenCv map representing the World where the robot live.
    cv::Mat map_;
    
    //! Current sensor setting, which are used to specify the beams angles, number of beams
    //! and the max and min range of the beams.
    SensorSettings sensor_settings_;

    std::vector<Eigen::Vector2d> circ_points_;

    //! num_meas_-dimensional vector containing the distances from the sensor and the obstacles. 
    //! Each entry is associated a one different beam.
    Eigen::VectorXd ranges_;

    //! Number of measurement computed using the sensor settings. In particular, based on the 
    //! max and min bound and the increments between beams. 
    int num_meas_;

    //! is a num_meas_-dimensional vector containing 2D point representing the
    //! position of the obstacle.
    std::vector<Eigen::Vector2d> min_points_;

};




#endif