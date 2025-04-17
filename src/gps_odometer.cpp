#include "ros/ros.h" //include le API di ROS
#include "sensor_msgs/NavSatFix.h" //messaggio che il nodo riceve
#include "nav_msgs/Odometry.h" //definisce il tipo di messaggio che il nodo invia

#include "first_project/odometry_tools.hpp" //include le funzioni di conversione
#include "first_project/gps_odometer_tools.hpp"

#include <cmath> //per le funzioni matematiche
#include <Eigen/Geometry> // Per Quaterniond e AngleAxisd

gps_odometer_tools::position old_enu = {0, 0, 0}; // inizializzazione della posizione ENU


void speedSteerCallback(const sensor_msgs::NavSatFix::ConstPtr& msg, 
						ros::Publisher& gps_odom_pub, 
						const gps_odometer_tools::positionGPS& reference_position){
	// 0 - Controllo se il messaggio è valido
	if (msg->latitude == 0.0 && msg->longitude == 0.0 && msg->altitude == 0.0) {
		ROS_WARN_STREAM("Messaggio GPS ignorato: coordinate nulle (0,0,0)");
		return;
	}
	
    
	// 1 - raccolta dati 
	gps_odometer_tools::positionGPS gps{ odometer_tools::degToRad(msg->latitude),
										 odometer_tools::degToRad(msg->longitude),
										 msg->altitude,
										 msg->header.stamp};
	ROS_INFO_STREAM("Messaggio GPS ricevuto");

	// 2 - Conversione da posizione GPS (latitudine, longitudine, altitudine) -> cartesiano ECEF
	gps_odometer_tools::position ecef = gps_odometer_tools::gpsToEcef(gps);

	// 3 - Conversione da cartesiano ECEF -> Cartesiano ENU
	gps_odometer_tools::position enu = gps_odometer_tools::ecefToEnu(ecef, gps, reference_position);

	ROS_INFO_STREAM("ENU: x=" << enu.x << ", y=" << enu.y << ", z=" << enu.z);


	// 4 - Calcolo dell'orientazione
	gps_odometer_tools::position direction = {enu.x - old_enu.x, enu.y - old_enu.y, enu.z - old_enu.z};
	double yaw = atan2(direction.z, direction.x);
    double pitch = atan2(direction.y, sqrt(direction.x * direction.x + direction.z * direction.z));
	double roll = 0.0; // opzionale

	// 5 - Conversione in quaternione
	geometry_msgs::Quaternion quaternion = gps_odometer_tools::eulerToQuaternion(roll, pitch, yaw);

	// 6 - Calcolo velocità lineare e angolare
	double dt = getSampleTime(enu, old_enu);
	geometry_msgs::Vector3 linear_velocity;
	linear_velocity.x = direction.x / dt;
	linear_velocity.y = direction.y / dt;
	linear_velocity.z = direction.z / dt;

	geometry_msgs::Vector3 angular_velocity;
	angular_velocity.x = 0;
	angular_velocity.y = pitch / dt;
	angular_velocity.z = yaw / dt;

	// 7 - creazione del messaggio Odometry
	nav_msgs::Odometry odom_msg;
	odom_msg.header.stamp = gps.time;
	odom_msg.header.frame_id = "gps_odom";
	odom_msg.child_frame_id = "base_link";
	//posizione in Sistema di riferimento del mondo "odom"
	odom_msg.pose.pose.position.x = enu.x;
	odom_msg.pose.pose.position.y = enu.y;
	odom_msg.pose.pose.position.z = enu.z;
	odom_msg.pose.pose.orientation = quaternion;
	//velocità in Sistema di riferimento del robot "base_link"
	odom_msg.twist.twist.linear = linear_velocity;
	odom_msg.twist.twist.angular = angular_velocity;


	//7 - pubblicazione del messaggio odometry
	gps_odom_pub.publish(odom_msg);
}



int main(int argc, char **argv){
	ros::init(argc, argv, "gps_odometer");//inizializza il nodo listener

	ros::NodeHandle n;//crea publisher, subscriber...

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

	ros::Publisher gps_odom_pub = n.advertise<nav_msgs::Odometry>("gps_odom", 10); // inizializzazione publisher

  	ros::Subscriber sub = n.subscribe<sensor_msgs::NavSatFix>("/swiftnav/front/gps_pose", 1,
        [&gps_odom_pub, &reference_position](const sensor_msgs::NavSatFix::ConstPtr& msg) {
            speedSteerCallback(msg, gps_odom_pub, reference_position);//sottosrive il nodo al topic e ogni volta che riceve un messaggi richiama la funzione chatterCallback
		});


  	ros::spin();

  return 0;
}