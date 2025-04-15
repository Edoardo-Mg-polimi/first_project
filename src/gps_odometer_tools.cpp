#include "gps_odometer_tools.hpp"

namespace gps_odometer_tools{

    positionECEF gpsToECEF(positionGPS gps){
        double eccentricity = sqrt(1 - (POLAR_RADIUS^2) / (EQUATORIAL_RADIUS^2));
        double N = EQUATORIAL_RADIUS / sqrt(1 - (eccentricity^2) * sin(gps.latitude)^2);

        positionECEF ecef;
        ecef.x = (N + gps.altitude) * cos(gps.latitude) * cos(gps.longitude);
        ecef.y = (N + gps.altitude) * cos(gps.latitude) * sin(gps.longitude);
        ecef.z = ((1 - eccentricity^2) * N + gps.altitude) * sin(gps.latitude);

        return ecef;
    }

    

}