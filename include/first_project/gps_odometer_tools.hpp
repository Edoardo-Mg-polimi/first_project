#pragma once
#include <cmath>
#include <Eigen/Geometry> // For Quaterniond and AngleAxisd
#include "ros/ros.h" // Includes ROS APIs
#include "sensor_msgs/NavSatFix.h" // Message received by the node
#include "nav_msgs/Odometry.h" // Defines the type of message sent by the node

#include "first_project/odometry_tools.hpp"

namespace gps_odometer_tools {

    /**
     * @brief Earth's equatorial radius in meters.
     */
    constexpr double EQUATORIAL_RADIUS = 6378137.0; // in meters

    /**
     * @brief Earth's polar radius in meters.
     */
    constexpr double POLAR_RADIUS = 6356752; // in meters

    /**
     * @brief Sensitivity threshold for orientation calculation.
     */
    constexpr double EPSILON = 0.5;
    
    /**
     * @brief Structure representing a GPS position.
     * 
     * Contains latitude, longitude, altitude, and a ROS timestamp.
     */
    struct positionGPS {
        double latitude; ///< Latitude in degrees.
        double longitude; ///< Longitude in degrees.
        double altitude; ///< Altitude in meters.
        ros::Time time; ///< ROS timestamp.
    };

    /**
     * @brief Structure representing a Cartesian position.
     * 
     * Contains x, y, z coordinates and a ROS timestamp.
     */
    struct position {
        double x; ///< x coordinate in meters.
        double y; ///< y coordinate in meters.
        double z; ///< z coordinate in meters.
        ros::Time time; ///< ROS timestamp.
    };

    /**
     * @brief Converts a GPS position to ECEF (Earth-Centered, Earth-Fixed) coordinates.
     * 
     * @param gps The GPS position in DEGREES to convert.
     * @return position The position in ECEF coordinates.
     */
    position gpsToEcef(positionGPS gps);

    /**
     * @brief Converts ECEF coordinates to ENU (East-North-Up) coordinates.
     * 
     * @param ecef The position in ECEF coordinates.
     * @param gps The current GPS position.
     * @param reference_gps The reference GPS position.
     * @return position The position in ENU coordinates.
     */
    position ecefToEnu(position ecef, positionGPS gps, positionGPS reference_gps);

    /**
     * @brief Calculates the sampling time between two ENU positions.
     * 
     * @param enu The current ENU position.
     * @param old_enu The previous ENU position.
     * @return double The sampling time in seconds.
     */
    double getSampleTime(position enu, position& old_enu);

    /**
     * @brief Converts Euler angles (roll, pitch, yaw) to a ROS quaternion.
     * 
     * @param roll Roll angle in radians.
     * @param pitch Pitch angle in radians.
     * @param yaw Yaw angle in radians.
     * @return geometry_msgs::Quaternion The corresponding quaternion.
     */
    geometry_msgs::Quaternion eulerToQuaternion(double roll, double pitch, double yaw);

}
