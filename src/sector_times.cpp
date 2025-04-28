#include "ros/ros.h" //include le API di ROS
#include "geometry_msgs/PointStamped.h" //messaggio input 1
/*geometry_msgs/PointStamped:
    y = speed [km/h]
    x = steer at the steering wheel [deg]
*/
#include "sensor_msgs/NavSatFix.h" //messaggio input 2
// sincronizzazione dei messaggi in ingresso
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "first_project/gps_odometer_tools.hpp"
#include "first_project/sector_tools.hpp" 

#include "first_project/Sector_times.h" //messaggio in uscita

//SETTORI
int current_sector = 1; // variabile globale per il settore corrente
int last_sector = 0; // variabile globale per il settore precedente

//TEMPI
double start_sector_time = 0; // tempo di inizio del settore
double last_time = 0; // tempo di sample precedente


//VELOCITÁ
double summed_weighted_speed = 0; // somma delle velocità
double mean_speed = 0; // velocità media del settore

void callback(const geometry_msgs::PointStampedConstPtr& msg1, 
              const sensor_msgs::NavSatFixConstPtr& msg2,
              const gps_odometer_tools::positionGPS& reference_position,
              ros::Publisher& sector_times_pub) {

    // 0 - Controllo se il messaggio è valido
    if (msg2->latitude == 0.0 && msg2->longitude == 0.0 && msg2->altitude == 0.0) {
        ROS_WARN_STREAM("Messaggio GPS ignorato: coordinate nulle (0,0,0)");
        return;
    }

    // 1 - raccolta dati
    gps_odometer_tools::positionGPS gps{ msg2->latitude,
                                         msg2->longitude,
                                         msg2->altitude,
                                         msg2->header.stamp};
    
    ros::Time current_time = msg1->header.stamp;
    double current_speed = msg1->point.y; // velocità in km/h

    // 2 - Calcolo del settore
    current_sector = sector_tools::getSector(gps, reference_position, current_sector);
    ROS_INFO_STREAM("Settore: " << current_sector);

    // 3 - Calcolo del tempo di percorrenza del settore
    if (sector_tools::sectorChanged(current_sector, last_sector)){ //quando cambio settore
        start_sector_time = current_time.toSec();//resetto il tempo di inizio del settore
        ROS_INFO_STREAM("Inizio settore " << current_sector << " al tempo: " << start_sector_time); 
        ROS_INFO_STREAM("Resetto velocità media e cronometro");
        
        summed_weighted_speed = 0; // resetto la somma delle velocità
        last_time = current_time.toSec();

        mean_speed = current_speed;
    }

    double sector_time = current_time.toSec() - start_sector_time;
    ROS_INFO_STREAM("Cronometro: " << sector_time);

    // 4 - Velocità media del settore
    double dt = current_time.toSec() - last_time; //calcolo il tempo di campionamento
    last_time = current_time.toSec();

    summed_weighted_speed += current_speed * dt;// somma delle velocità pesate
    mean_speed = summed_weighted_speed / sector_time;
    
    // 5 - Creazione del messaggio
    first_project::Sector_times msg_out;
    msg_out.current_sector = current_sector;
    msg_out.current_sector_time = sector_time;
    msg_out.current_sector_mean_speed = mean_speed;

    // 6 - Pubblicazione del messaggio
    sector_times_pub.publish(msg_out);
}




int main(int argc, char **argv){
	ros::init(argc, argv, "sector_times");//inizializza il nodo sector_times

	ros::NodeHandle n;//crea publisher, subscriber...

    //aggiunta parametri inizio percorso
	gps_odometer_tools::positionGPS reference_position;

	if (!n.getParam("reference_latitude", reference_position.latitude) ||
		!n.getParam("reference_longitude", reference_position.longitude) ||
		!n.getParam("reference_altitude", reference_position.altitude)) {
		ROS_ERROR_STREAM("Parametri di riferimento GPS non trovati!");
	}
	else {
    ROS_INFO_STREAM("Reference GPS: lat=" << reference_position.latitude
                    << ", lon=" << reference_position.longitude
                    << ", alt=" << reference_position.altitude);
	}

	ros::Publisher sector_times_pub = n.advertise<first_project::Sector_times>("sector_times", 10); // inizializzazione publisher

    message_filters::Subscriber<geometry_msgs::PointStamped> sub1(n, "speedsteer", 1);
    message_filters::Subscriber<sensor_msgs::NavSatFix> sub2(n, "/swiftnav/front/gps_pose", 1);

    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PointStamped, sensor_msgs::NavSatFix> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub1, sub2);
    sync.registerCallback(boost::bind(&callback, _1, _2, reference_position, boost::ref(sector_times_pub)));

  	ros::spin();

  return 0;
}
