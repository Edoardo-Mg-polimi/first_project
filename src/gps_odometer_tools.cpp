#include "first_project/gps_odometer_tools.hpp"

namespace gps_odometer_tools{

    position gpsToEcef(positionGPS gps){
        double eccentricity = sqrt(1 - (POLAR_RADIUS * POLAR_RADIUS) / (EQUATORIAL_RADIUS * EQUATORIAL_RADIUS));
        double N = EQUATORIAL_RADIUS / sqrt(1 - (eccentricity * eccentricity) * sin(gps.latitude) * sin(gps.latitude));

        position ecef;
        ecef.x = (N + gps.altitude) * cos(gps.latitude) * cos(gps.longitude);
        ecef.y = (N + gps.altitude) * cos(gps.latitude) * sin(gps.longitude);
        ecef.z = ((1 - eccentricity * eccentricity) * N + gps.altitude) * sin(gps.latitude);

        ecef.time = gps.time;

        return ecef;
    }

    position ecefToEnu(position ecef, positionGPS gps, positionGPS reference_gps){
        //Confertire la reference in GPS in radianti
        reference_gps = {odometer_tools::degToRad(reference_gps.latitude), 
                        odometer_tools::degToRad(reference_gps.longitude), 
                        reference_gps.altitude};
        
        //Convertire la reference in ECEF
        position reference_pos = gpsToEcef(reference_gps);

        position enu;
        enu.x = -sin(reference_gps.longitude) * (ecef.x - reference_pos.x) + cos(reference_gps.longitude) * (ecef.y - reference_pos.y);
        enu.y = -sin(reference_gps.latitude) * cos(reference_gps.longitude) * (ecef.x - reference_pos.x) - sin(reference_gps.latitude) * sin(reference_gps.longitude) * (ecef.y - reference_pos.y) + cos(reference_gps.latitude) * (ecef.z - reference_pos.z);
        enu.z = cos(reference_gps.latitude) * cos(reference_gps.longitude) * (ecef.x - reference_pos.x) + cos(reference_gps.latitude) * sin(reference_gps.longitude) * (ecef.y - reference_pos.y) + sin(reference_gps.latitude) * (ecef.z - reference_pos.z);

        enu.time = gps.time;

        return enu;
    }

    double getSampleTime(position enu, position& old_enu){
        double dt = (enu.time - old_enu.time).toSec();
        if (dt <= 0) {
            ROS_WARN_STREAM("Tempo non valido, non posso calcolare la velocità");
            return 0;
        }
        old_enu = enu; // aggiorna la vecchia posizione ENU
        return dt;
    }
    
    geometry_msgs::Quaternion eulerToQuaternion(double roll, double pitch, double yaw){
        Eigen::AngleAxisd yawAngle(yaw,   Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd rollAngle(roll,  Eigen::Vector3d::UnitZ());

        Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;

        geometry_msgs::Quaternion q_msg;
        q_msg.x = q.x();
        q_msg.y = q.y();
        q_msg.z = q.z();
        q_msg.w = q.w();

        return q_msg;
    }

}