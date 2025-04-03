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
    void eulerIntegration(double& x, double& y, double& theta, double speed, double angular_speed, double dt){
        x += speed * cos(theta) * dt;
        y += speed * sin(theta) * dt;
        theta += angular_speed * dt;
    }

}