#pragma once
#include "ros/ros.h" // Include le API di ROS
#include <Eigen/Geometry> // Per Quaterniond e AngleAxisd
#include "sensor_msgs/NavSatFix.h" // Messaggio che il nodo riceve

#include "first_project/gps_odometer_tools.hpp"
#include "first_project/odometry_tools.hpp"

namespace sector_tools {

    /**
     * @brief Struttura che rappresenta un confine tra due settori.
     * 
     * Contiene due punti (A e B) rappresentati come vettori 2D.
     */
    typedef struct {
        Eigen::Vector2d A; ///< Punto A del confine.
        Eigen::Vector2d B; ///< Punto B del confine.
    } border;


    /**
     * @brief Confine tra il settore 1 e il settore 2.
     */
    const border SECTOR_1_2 = {{45.63005992259639, 9.289493322674346}, {45.63014854640358, 9.289485276047794}};

    /**
     * @brief Confine tra il settore 2 e il settore 3.
     */
    const border SECTOR_2_3 = {{45.62360610880583, 9.287260626835694}, {45.62353576425271, 9.287329693713597}};

    /**
     * @brief Confine tra il settore 3 e il settore 1.
     */
    const border SECTOR_3_1 = {{45.616046529711056, 9.280680036793406}, {45.61603152085702, 9.280910706754558}};


    /**
     * @brief Tolleranza in metri per il calcolo dei settori.
     */
    constexpr int TOLLERANCE = 5;

    /**
     * @brief Determina il settore in cui si trova una posizione GPS.
     * 
     * @param gps La posizione GPS corrente.
     * @param reference_gps La posizione GPS di riferimento.
     * @param last_sector Il settore precedente.
     * @return int Il settore corrente (1, 2 o 3).
     */
    int getSector(gps_odometer_tools::positionGPS gps, gps_odometer_tools::positionGPS reference_gps, int last_sector);

    /**
     * @brief Converte un confine GPS in coordinate ENU (East-North-Up).
     * 
     * @param gps_boundary Il confine GPS da convertire.
     * @param reference_gps La posizione GPS di riferimento.
     * @return border Il confine convertito in coordinate ENU.
     */
    border gpsBorderToEnu(border gps_boundary, gps_odometer_tools::positionGPS reference_gps);

    /**
     * @brief Verifica se il settore è cambiato.
     * 
     * @param new_sector Il nuovo settore calcolato.
     * @param last_sector Il settore precedente (passato per riferimento e aggiornato se necessario).
     * @return bool True se il settore è cambiato, False altrimenti.
     */
    bool sectorChanged(int new_sector, int& last_sector);

}