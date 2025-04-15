#include "ros/ros.h" //include le API di ROS
#include "sensor_msg/NavSatFix.h" //messaggio che il nodo riceve
#include "nav_msgs/Odometry.h" //definisce il tipo di messaggio che il nodo invia

#include "gps_odometer_tools.hpp"


void speedSteerCallback(const geometry_msgs::PointStamped::ConstPtr& msg, ros::Publisher& gps_odom_pub){
    // 1 - raccolta dati 
	gps_odometer_tools::positionGPS gps{ msg->point.x, msg->point.y, msg->point.z };
	ROS_INFO_STREAM("Messaggio GPS ricevuto");

	// 2 - Conversione da posizione GPS (latitudine, longitudine, altitudine) -> cartesiano ECEF
	gps_odometer_tools::positionECEF ecef = gps_odometer_tools::gpsToECEF(gps);

	// 3 - Conversione da cartesiano ECEF -> Cartesiano ENU

	


}



int main(int argc, char **argv){
	ros::init(argc, argv, "gps_odometer");//inizializza il nodo listener

	ros::NodeHandle n;//crea publisher, subscriber...

	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("gps_odom", 10); // inizializzazione publisher
  	ros::Subscriber sub = n.subscribe<geometry_msgs::PointStamped>("gps_pose", 1,
        [&odom_pub](const geometry_msgs::PointStamped::ConstPtr& msg) {
            speedSteerCallback(msg, gps_odom_pub);//sottosrive il nodo al topic e ogni volta che riceve un messaggi richiama la funzione chatterCallback
		});


  	ros::spin();

  return 0;
}