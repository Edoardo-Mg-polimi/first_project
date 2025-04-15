#pragma once

#include "ros/ros.h" //include le API di ROS
#include "sensor_msg/NavSatFix.h" //messaggio che il nodo riceve
#include "nav_msgs/Odometry.h" //definisce il tipo di messaggio che il nodo invia

namespace gps_odometer_tools{
    //Constants
    constexpr double EQUATORIAL_RADIUS = 6378137.0; // in meters
    constexpr double POLAR_RADIUS = 6356752; // in meters


    struct positionGPS{
        double latitude;
        double longitude;
        double altitude;
    };

    struct positionECEF{
        double x;
        double y;
        double z;
    };

    struct positionENU{
        double x;
        double y;
        double z;
    };

    //Conversioni
    positionECEF gpsToECEF(positionGPS gps);

}
