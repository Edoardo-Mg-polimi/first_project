#pragma once

#include "ros/ros.h" // Include le API di ROS
#include "geometry_msgs/PointStamped.h" // Definisce il tipo di messaggio che il nodo riceve
/* geometry_msgs/PointStamped:
    y = speed [km/h]
    x = steer at the steering wheel [deg]
*/
#include "nav_msgs/Odometry.h" // Definisce il tipo di messaggio che il nodo invia
#include <cmath> // Per le funzioni matematiche
#include <tf2/LinearMath/Quaternion.h>

namespace odometer_tools {

    /**
     * @brief Distanza tra le ruote (asse anteriore e posteriore) in metri.
     */
    constexpr int WHEEL_BASELINE = 1.30; // m

    /**
     * @brief Distanza tra l'asse anteriore e quello posteriore in metri.
     */
    constexpr float FRONT_TO_REAR_AXIS = 1.765; // m

    /**
     * @brief Fattore di conversione per l'angolo di sterzata.
     */
    constexpr int STEERING_FACTOR = 32;


    /**
     * @brief Struttura che rappresenta lo stato della posizione.
     * 
     * Contiene le coordinate cartesiane (x, y), l'angolo di orientamento (theta)
     * e il timestamp ROS.
     */
    struct positionState {
        double x; ///< Coordinata x in metri.
        double y; ///< Coordinata y in metri.
        double theta; ///< Angolo di orientamento in radianti.
        ros::Time time; ///< Timestamp ROS.
    };


    /**
     * @brief Converte la velocità da km/h a m/s.
     * 
     * @param speed Velocità in km/h.
     * @return double Velocità in m/s.
     */
    double speedConvert(double speed);

    /**
     * @brief Converte l'angolo di sterzata dal volante in radianti.
     * 
     * @param steer Angolo di sterzata in gradi.
     * @return double Angolo di sterzata in radianti.
     */
    double steerConvert(double steer);

    /**
     * @brief Converte un angolo da gradi a radianti.
     * 
     * @param deg Angolo in gradi.
     * @return double Angolo in radianti.
     */
    double degToRad(double deg);

    /**
     * @brief Calcola la velocità angolare utilizzando l'approssimazione del modello Ackermann.
     * 
     * @param steering_angle Angolo di sterzata in radianti.
     * @param speed Velocità in m/s.
     * @return double Velocità angolare in radianti al secondo.
     */
    double getAngularSpeed(double steering_angle, double speed);


    /**
     * @brief Calcola il campionamento temporale tra due timestamp.
     * 
     * @param state Stato della posizione precedente.
     * @param current_time Timestamp corrente.
     * @return double Campionamento temporale in secondi.
     */
    double timeSample(positionState& state, ros::Time current_time);


    /**
     * @brief Esegue l'integrazione di Eulero per aggiornare lo stato della posizione.
     * 
     * @param state Stato della posizione corrente.
     * @param speed Velocità in m/s.
     * @param angular_speed Velocità angolare in radianti al secondo.
     * @param current_time Timestamp corrente.
     * @return positionState Stato della posizione aggiornato.
     */
    positionState eulerIntegration(positionState state, double speed, double angular_speed, ros::Time current_time);

    /**
     * @brief Esegue l'integrazione di Runge-Kutta per aggiornare lo stato della posizione.
     * 
     * @param state Stato della posizione corrente.
     * @param speed Velocità in m/s.
     * @param angular_speed Velocità angolare in radianti al secondo.
     * @param current_time Timestamp corrente.
     * @return positionState Stato della posizione aggiornato.
     */
    positionState rungeKuttaIntegration(positionState state, double speed, double angular_speed, ros::Time current_time);

    /**
     * @brief Converte lo stato della posizione in un quaternion ROS.
     * 
     * Questa funzione prende uno stato di posizione rappresentato da coordinate
     * cartesiane (x, y) e un angolo di orientamento (theta) e lo converte in un
     * messaggio `geometry_msgs::Quaternion` utilizzato in ROS.
     * 
     * @param state Lo stato della posizione, contenente x, y, theta e il timestamp.
     * @return geometry_msgs::Quaternion Il quaternion corrispondente all'orientamento.
     */
    geometry_msgs::Quaternion quaternionConversion(positionState state);

}