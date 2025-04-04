#pragma once

#include "ros/ros.h" //include le API di ROS
#include "geometry_msgs/PointStamped.h" //definisce il tipo di messaggio che il nodo riceve
/*geometry_msgs/PointStamped:
    y = speed [km/h]
    x = steer at the steering wheel [deg]
*/
#include "nav_msgs/Odometry.h" //definisce il tipo di messaggio che il nodo invia
#include <cmath> //per le funzioni matematiche
#include <tf2/LinearMath/Quaternion.h>

namespace odometer_tools{
    //Constants
    constexpr int WHEEL_BASELINE = 1.30;//m
    constexpr float FRONT_TO_REAR_AXIS = 1.765;//m
    constexpr int STEERING_FACTOR = 32;
    
    struct positionState{
        double x;
        double y;
        double theta;
        ros::Time time;
    };

    //Conversions
    double speedConvert(double speed);
    double steerConvert(double steer);
    double degToRad(double deg);

    //Ackermann steering bicyle approximation
    double getAngularSpeed(double steering_angle, double speed);

    //Euler integration
    positionState eulerIntegration(positionState state, double speed, double angular_speed, ros::Time current_time);

    //Quaternion conversion
    geometry_msgs::Quaternion quaternionConversion(positionState state);

}