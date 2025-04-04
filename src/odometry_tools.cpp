#include "first_project/odometry_tools.hpp"


namespace odometer_tools{

    //Conversions
    double speedConvert(double speed){
        return speed / 3.6; // km/h to m/s
    }

    double steerConvert(double steer){
        return steer / STEERING_FACTOR; // volante to rotazione ruota
    }

    double degToRad(double deg){
        return deg * (M_PI / 180.0);
    }

    //Ackermann steering bicyle approximation
    double getAngularSpeed(double steering_angle, double speed){
        double radius = (FRONT_TO_REAR_AXIS / tan(steering_angle));
        double angular_speed = speed / radius;
        return angular_speed;
    }


    //Euler integration
    positionState eulerIntegration(positionState state, double speed, double angular_speed, ros::Time current_time){
        double dt = 0.0;
        if (!state.time.isZero()) {
            dt = (current_time - state.time).toSec();
        }
        state.time = current_time;

        state.x += speed * cos(state.theta) * dt;
        state.y += speed * sin(state.theta) * dt;
        state.theta += angular_speed * dt;

        return state;
    }

    //Quaternion conversion
    geometry_msgs::Quaternion quaternionConversion(positionState state){
        geometry_msgs::Quaternion orientation;
        tf2::Quaternion q;

        q.setRPY(0, 0, state.theta);

        orientation.x = q.x();
        orientation.y = q.y();
        orientation.z = q.z();
        orientation.w = q.w();

        return orientation;
    }

}