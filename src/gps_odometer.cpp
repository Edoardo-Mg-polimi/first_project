#include "ros/ros.h" //include le API di ROS
#include "sensor_msgs/NavSatFix.h" //messaggio che il nodo riceve
#include "nav_msgs/Odometry.h" //definisce il tipo di messaggio che il nodo invia

 
#include "first_project/gps_odometer_tools.hpp"


void speedSteerCallback(const sensor_msgs::NavSatFix::ConstPtr& msg, 
						ros::Publisher& gps_odom_pub, 
						const gps_odometer_tools::positionGPS& reference_position){
    
	// 1 - raccolta dati 
	gps_odometer_tools::positionGPS gps{ odometer_tools::degToRad(msg->latitude), odometer_tools::degToRad(msg->longitude), odometer_tools::degToRad(msg->altitude) };
	ROS_INFO_STREAM("Messaggio GPS ricevuto");

	// 2 - Conversione da posizione GPS (latitudine, longitudine, altitudine) -> cartesiano ECEF
	gps_odometer_tools::position ecef = gps_odometer_tools::gpsToEcef(gps);

	// 3 - Conversione da cartesiano ECEF -> Cartesiano ENU
	gps_odometer_tools::position enu = gps_odometer_tools::ecefToEnu(ecef, gps, reference_position);

	// 4 - Calcolo dell'orientazione

	// ??? 5 va convertito in quaternione ?

	// 6 - Creazione del messaggio di odometria
	
	// 7 - invio del messaggio di odometry

}



int main(int argc, char **argv){
	ros::init(argc, argv, "gps_odometer");//inizializza il nodo listener

	ros::NodeHandle n;//crea publisher, subscriber...

	gps_odometer_tools::positionGPS reference_position;

	n.getParam("reference_latitude", reference_position.latitude);
	n.getParam("reference_longitude", reference_position.longitude);
	n.getParam("reference_altitude", reference_position.altitude);

	ros::Publisher gps_odom_pub = n.advertise<nav_msgs::Odometry>("gps_odom", 10); // inizializzazione publisher

  	ros::Subscriber sub = n.subscribe<sensor_msgs::NavSatFix>("/swiftnav/front/gps_pose", 1,
        [&gps_odom_pub, &reference_position](const sensor_msgs::NavSatFix::ConstPtr& msg) {
            speedSteerCallback(msg, gps_odom_pub, reference_position);//sottosrive il nodo al topic e ogni volta che riceve un messaggi richiama la funzione chatterCallback
		});


  	ros::spin();

  return 0;
}