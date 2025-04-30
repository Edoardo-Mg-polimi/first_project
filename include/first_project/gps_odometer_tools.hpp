#pragma once
#include <cmath>
#include <Eigen/Geometry> // Per Quaterniond e AngleAxisd
#include "ros/ros.h" // Include le API di ROS
#include "sensor_msgs/NavSatFix.h" // Messaggio che il nodo riceve
#include "nav_msgs/Odometry.h" // Definisce il tipo di messaggio che il nodo invia

#include "first_project/odometry_tools.hpp"

namespace gps_odometer_tools {

    /**
     * @brief Raggio equatoriale della Terra in metri.
     */
    constexpr double EQUATORIAL_RADIUS = 6378137.0; // in meters

    /**
     * @brief Raggio polare della Terra in metri.
     */
    constexpr double POLAR_RADIUS = 6356752; // in meters


    /**
     * @brief Struttura che rappresenta una posizione GPS.
     * 
     * Contiene latitudine, longitudine, altitudine e un timestamp ROS.
     */
    struct positionGPS {
        double latitude; ///< Latitudine in gradi.
        double longitude; ///< Longitudine in gradi.
        double altitude; ///< Altitudine in metri.
        ros::Time time; ///< Timestamp ROS.
    };

    /**
     * @brief Struttura che rappresenta una posizione cartesiana.
     * 
     * Contiene le coordinate x, y, z e un timestamp ROS.
     */
    struct position {
        double x; ///< Coordinata x in metri.
        double y; ///< Coordinata y in metri.
        double z; ///< Coordinata z in metri.
        ros::Time time; ///< Timestamp ROS.
    };


    /**
     * @brief Converte una posizione GPS in coordinate ECEF (Earth-Centered, Earth-Fixed).
     * 
     * @param gps La posizione GPS  in GRADI da convertire.
     * @return position La posizione in coordinate ECEF.
     */
    position gpsToEcef(positionGPS gps);

    /**
     * @brief Converte coordinate ECEF in coordinate ENU (East-North-Up).
     * 
     * @param ecef La posizione in coordinate ECEF.
     * @param gps La posizione GPS corrente.
     * @param reference_gps La posizione GPS di riferimento.
     * @return position La posizione in coordinate ENU.
     */
    position ecefToEnu(position ecef, positionGPS gps, positionGPS reference_gps);


    /**
     * @brief Calcola il tempo di campionamento tra due posizioni ENU.
     * 
     * @param enu La posizione ENU corrente.
     * @param old_enu La posizione ENU precedente.
     * @return double Il tempo di campionamento in secondi.
     */
    double getSampleTime(position enu, position& old_enu);

    /**
     * @brief Converte angoli di Eulero (roll, pitch, yaw) in un quaternion ROS.
     * 
     * @param roll Angolo di rollio in radianti.
     * @param pitch Angolo di beccheggio in radianti.
     * @param yaw Angolo di imbardata in radianti.
     * @return geometry_msgs::Quaternion Il quaternion corrispondente.
     */
    geometry_msgs::Quaternion eulerToQuaternion(double roll, double pitch, double yaw);

}
