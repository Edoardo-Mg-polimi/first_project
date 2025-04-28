#include "first_project/sector_tools.hpp"

namespace sector_tools{

    border gpsBorderToEnu(border gps_boundary, gps_odometer_tools::positionGPS reference_gps){
        //Punto A del confine
        gps_odometer_tools::positionGPS A_gps = {gps_boundary.A.x(), gps_boundary.A.y(), 0.0, ros::Time(0)};
        gps_odometer_tools::position ecef_A = gps_odometer_tools::gpsToEcef(A_gps);
        gps_odometer_tools::position enu_A = gps_odometer_tools::ecefToEnu(ecef_A, A_gps, reference_gps);

        //Punto B del confine
        gps_odometer_tools::positionGPS B_gps = {gps_boundary.B.x(), gps_boundary.B.y(), 0.0, ros::Time(0)};
        gps_odometer_tools::position ecef_B = gps_odometer_tools::gpsToEcef(B_gps);
        gps_odometer_tools::position enu_B = gps_odometer_tools::ecefToEnu(ecef_B, B_gps, reference_gps);
        
        //Creazione del confine in ENU
        border enu_border = {{enu_A.x, enu_A.y}, {enu_B.x, enu_B.y}};

        return enu_border;
    }


    int getSector(gps_odometer_tools::positionGPS gps, gps_odometer_tools::positionGPS reference_gps, int last_sector){
        // 1 - Trasformo le coordinate GPS in ENU
        gps_odometer_tools::position ecef = gps_odometer_tools::gpsToEcef(gps);
        gps_odometer_tools::position enu = gps_odometer_tools::ecefToEnu(ecef, gps, reference_gps);
        Eigen::Vector2d P = {enu.x, enu.y};

        switch (last_sector){
        case 1:{
            // 2 - Trasformo le coordinate dei 2 punti del border 1-2 in ENU
            border border_1_2 = gpsBorderToEnu(SECTOR_1_2, reference_gps);
            
            // 3 - Calcola da quale lato si trova il punto point rispetto al border
            double orientation = (border_1_2.B - border_1_2.A).x() * (P - border_1_2.A).y() -
                                 (border_1_2.B - border_1_2.A).y() * (P - border_1_2.A).x();

            ROS_INFO_STREAM("P: " << P.transpose());
            ROS_INFO_STREAM("A: " << border_1_2.A.transpose());
            ROS_INFO_STREAM("B: " << border_1_2.B.transpose());
            ROS_INFO_STREAM("Orientation: " << orientation);
                                 
            
            if (orientation >= 0)
                return 1;
            
            else if (orientation < 0)
                return 2;
        break;
        }

        case 2:{
            // 2 - Trasformo le coordinate dei 2 punti del border 2-3 in ENU
            border border_2_3 = gpsBorderToEnu(SECTOR_2_3, reference_gps);

            // 3 - Calcola da quale lato si trova il punto point rispetto al border
            double orientation = (border_2_3.B - border_2_3.A).x() * (P - border_2_3.A).y() -
                                 (border_2_3.B - border_2_3.A).y() * (P - border_2_3.A).x();

            ROS_INFO_STREAM("P: " << P.transpose());
            ROS_INFO_STREAM("A: " << border_2_3.A.transpose());
            ROS_INFO_STREAM("B: " << border_2_3.B.transpose());
            ROS_INFO_STREAM("Orientation: " << orientation);
            
            if (orientation >= 0)
                return 2;
            
            else if (orientation < 0)
                return 3;
        break;
        }
        
        case 3:{
            // 2 - Trasformo le coordinate dei 2 punti del border 3-1 in ENU
            border border_3_1 = gpsBorderToEnu(SECTOR_3_1, reference_gps);

            // 3 - Calcola da quale lato si trova il punto point rispetto al border
            double orientation = (border_3_1.B - border_3_1.A).x() * (P - border_3_1.A).y() -
                                 (border_3_1.B - border_3_1.A).y() * (P - border_3_1.A).x();

            ROS_INFO_STREAM("P: " << P.transpose());
            ROS_INFO_STREAM("A: " << border_3_1.A.transpose());
            ROS_INFO_STREAM("B: " << border_3_1.B.transpose());
            ROS_INFO_STREAM("Orientation: " << orientation);

            
            if (orientation >= 0)
                return 3;
            
            else if (orientation < 0)
                return 1;
        break;
        }

        default:
            ROS_ERROR_STREAM("Settore non valido: " << last_sector);
            return last_sector;
        break;
        }

        return -1; // Se non si trova nessun settore, ritorna -1
        
    }


    bool sectorChanged(int new_sector, int& last_sector){
        if (last_sector != new_sector){
            ROS_INFO_STREAM("Settore cambiato: " << last_sector << " -> " << new_sector);
            last_sector = new_sector;
            return true;
        }
        
        return false;
    }

}