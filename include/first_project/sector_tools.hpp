#pragma once
#include "ros/ros.h" //include le API di ROS
#include <Eigen/Geometry> // Per Quaterniond e AngleAxisd
#include "sensor_msgs/NavSatFix.h" //messaggio che il nodo riceve

#include "first_project/gps_odometer_tools.hpp"
#include "first_project/odometry_tools.hpp"


namespace sector_tools{

    typedef struct{
        Eigen::Vector2d A;
        Eigen::Vector2d B;
    } border;

    //Constants
    //barrier between sector 1 and 2
    const border SECTOR_1_2 = {{45.63005992259639, 9.289493322674346}, {45.63014854640358, 9.289485276047794}};

    //barrier between sector 2 and 3
    const border SECTOR_2_3 = {{45.62360610880583, 9.287260626835694}, {45.62353576425271, 9.287329693713597}};

    //barrier between sector 3 and 1
    const border SECTOR_3_1 = {{45.616046529711056, 9.280680036793406}, {45.61603152085702, 9.280910706754558}};


    //tollerance in meters
    constexpr int TOLLERANCE = 5;


    int getSector(gps_odometer_tools::positionGPS gps,gps_odometer_tools::positionGPS reference_gps, int last_sector);

    border gpsBorderToEnu(border gps_boundary, gps_odometer_tools::positionGPS reference_gps);

    bool sectorChanged(int new_sector, int& last_sector);



}