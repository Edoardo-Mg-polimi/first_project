#pragma once

#include "ros/ros.h" //include le API di ROS
#include "sensor_msgs/NavSatFix.h" //messaggio che il nodo riceve
#include "nav_msgs/Odometry.h" //definisce il tipo di messaggio che il nodo invia

#include "first_project/odometry_tools.hpp"

namespace gps_odometer_tools{
    //Constants
    constexpr double EQUATORIAL_RADIUS = 6378137.0; // in meters
    constexpr double POLAR_RADIUS = 6356752; // in meters


    struct positionGPS{
        double latitude;
        double longitude;
        double altitude;
    };

    struct position{
        double x;
        double y;
        double z;
    };

    //Conversioni
    position gpsToEcef(positionGPS gps);

    position ecefToEnu(position ecef, positionGPS gps, positionGPS reference_gps);

}
