#pragma once
#include "ros/ros.h" // Includes ROS APIs
#include <Eigen/Geometry> // For Quaterniond and AngleAxisd
#include "sensor_msgs/NavSatFix.h" // Message received by the node

#include "first_project/gps_odometer_tools.hpp"
#include "first_project/odometry_tools.hpp"

namespace sector_tools {

    /**
     * @brief Structure representing a boundary between two sectors.
     * 
     * Contains two points (A and B) represented as 2D vectors.
     */
    typedef struct {
        Eigen::Vector2d A; ///< Point A of the boundary.
        Eigen::Vector2d B; ///< Point B of the boundary.
    } border;

    // Constants
    /**
     * @brief Boundary between sector 1 and sector 2.
     */
    const border SECTOR_1_2 = {{45.63005992259639, 9.289493322674346}, {45.63014854640358, 9.289485276047794}};

    /**
     * @brief Boundary between sector 2 and sector 3.
     */
    const border SECTOR_2_3 = {{45.62360610880583, 9.287260626835694}, {45.62353576425271, 9.287329693713597}};

    /**
     * @brief Boundary between sector 3 and sector 1.
     */
    const border SECTOR_3_1 = {{45.616046529711056, 9.280680036793406}, {45.61603152085702, 9.280910706754558}};

    /**
     * @brief Tolerance in meters for sector calculations.
     */
    constexpr int TOLLERANCE = 5;

    /**
     * @brief Determines the sector in which a GPS position is located.
     * 
     * @param gps The current GPS position.
     * @param reference_gps The reference GPS position.
     * @param last_sector The previous sector.
     * @return int The current sector (1, 2, or 3).
     */
    int getSector(gps_odometer_tools::positionGPS gps, gps_odometer_tools::positionGPS reference_gps, int last_sector);

    /**
     * @brief Converts a GPS boundary to ENU (East-North-Up) coordinates.
     * 
     * @param gps_boundary The GPS boundary to convert.
     * @param reference_gps The reference GPS position.
     * @return border The boundary converted to ENU coordinates.
     */
    border gpsBorderToEnu(border gps_boundary, gps_odometer_tools::positionGPS reference_gps);

    /**
     * @brief Checks if the sector has changed.
     * 
     * @param new_sector The newly calculated sector.
     * @param last_sector The previous sector (passed by reference and updated if necessary).
     * @return bool True if the sector has changed, False otherwise.
     */
    bool sectorChanged(int new_sector, int& last_sector);

}