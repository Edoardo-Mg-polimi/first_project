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


void callback(const geometry_msgs::PointStampedConstPtr& msg1, const sensor_msgs::NavSatFixConstPtr& msg2){
    ROS_INFO ("Received two messages");

    // 0 - Controllo se il messaggio è valido
    if (msg2->latitude == 0.0 && msg2->longitude == 0.0 && msg2->altitude == 0.0) {
        ROS_WARN_STREAM("Messaggio GPS ignorato: coordinate nulle (0,0,0)");
        return;
    }

    // 1 - raccolta dati
    gps_odometer_tools::positionGPS gps{ odometer_tools::degToRad(msg2->latitude),
                                         odometer_tools::degToRad(msg2->longitude),
                                         msg2->altitude,
                                         msg2->header.stamp};

    // 2 - Calcolo del settore

    // 3 - Calcolo del tempo di percorrenza del settore

    // 4 - Velocità media del setore

    // 5 - Creazione del messaggio

    // 6 - Pubblicazione del messaggio

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

	//ros::Publisher sector_times_pub = n.advertise</*** */>("sector_times", 10); // inizializzazione publisher

    message_filters::Subscriber<geometry_msgs::PointStamped> sub1(n, "speedsteer", 1);
    message_filters::Subscriber<sensor_msgs::NavSatFix> sub2(n, "/swiftnav/front/gps_pose", 1);

    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PointStamped, sensor_msgs::NavSatFix> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub1, sub2);
    sync.registerCallback(boost::bind(&callback, _1, _2));

  	ros::spin();

  return 0;
}
