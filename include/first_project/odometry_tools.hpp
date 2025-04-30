#pragma once

#include "ros/ros.h" // Includes ROS APIs
#include "geometry_msgs/PointStamped.h" // Defines the type of message received by the node
/* geometry_msgs/PointStamped:
    y = speed [km/h]
    x = steer at the steering wheel [deg]
*/
#include "nav_msgs/Odometry.h" // Defines the type of message sent by the node
#include <cmath> // For mathematical functions
#include <tf2/LinearMath/Quaternion.h>

namespace odometer_tools {

    /**
     * @brief Distance between the wheels (front and rear axle) in meters.
     */
    constexpr int WHEEL_BASELINE = 1.30; // m

    /**
     * @brief Distance between the front and rear axles in meters.
     */
    constexpr float FRONT_TO_REAR_AXIS = 1.765; // m

    /**
     * @brief Conversion factor for the steering angle.
     */
    constexpr int STEERING_FACTOR = 32;

    /**
     * @brief Structure representing the position state.
     * 
     * Contains Cartesian coordinates (x, y), orientation angle (theta),
     * and the ROS timestamp.
     */
    struct positionState {
        double x; ///< x coordinate in meters.
        double y; ///< y coordinate in meters.
        double theta; ///< Orientation angle in radians.
        ros::Time time; ///< ROS timestamp.
    };

    /**
     * @brief Converts speed from km/h to m/s.
     * 
     * @param speed Speed in km/h.
     * @return double Speed in m/s.
     */
    double speedConvert(double speed);

    /**
     * @brief Converts the steering angle from the steering wheel to radians.
     * 
     * @param steer Steering angle in degrees.
     * @return double Steering angle in radians.
     */
    double steerConvert(double steer);

    /**
     * @brief Converts an angle from degrees to radians.
     * 
     * @param deg Angle in degrees.
     * @return double Angle in radians.
     */
    double degToRad(double deg);

    /**
     * @brief Calculates the angular speed using the Ackermann model approximation.
     * 
     * @param steering_angle Steering angle in radians.
     * @param speed Speed in m/s.
     * @return double Angular speed in radians per second.
     */
    double getAngularSpeed(double steering_angle, double speed);

    /**
     * @brief Calculates the time sample between two timestamps.
     * 
     * @param state Previous position state.
     * @param current_time Current timestamp.
     * @return double Time sample in seconds.
     */
    double timeSample(positionState& state, ros::Time current_time);

    /**
     * @brief Performs Euler integration to update the position state.
     * 
     * @param state Current position state.
     * @param speed Speed in m/s.
     * @param angular_speed Angular speed in radians per second.
     * @param current_time Current timestamp.
     * @return positionState Updated position state.
     */
    positionState eulerIntegration(positionState state, double speed, double angular_speed, ros::Time current_time);

    /**
     * @brief Performs Runge-Kutta integration to update the position state.
     * 
     * @param state Current position state.
     * @param speed Speed in m/s.
     * @param angular_speed Angular speed in radians per second.
     * @param current_time Current timestamp.
     * @return positionState Updated position state.
     */
    positionState rungeKuttaIntegration(positionState state, double speed, double angular_speed, ros::Time current_time);

    /**
     * @brief Converts the position state to a ROS quaternion.
     * 
     * This function takes a position state represented by Cartesian coordinates
     * (x, y) and an orientation angle (theta) and converts it into a
     * `geometry_msgs::Quaternion` message used in ROS.
     * 
     * @param state The position state, containing x, y, theta, and the timestamp.
     * @return geometry_msgs::Quaternion The quaternion corresponding to the orientation.
     */
    geometry_msgs::Quaternion quaternionConversion(positionState state);

}