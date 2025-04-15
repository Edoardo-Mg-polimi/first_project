#include "ros/ros.h" //include le API di ROS
#include "sensor_msg/NavSatFix.h" //messaggio che il nodo riceve
#include "nav_msgs/Odometry.h" //definisce il tipo di messaggio che il nodo invia


void speedSteerCallback(const geometry_msgs::PointStamped::ConstPtr& msg, ros::Publisher& gps_odom_pub){
    // 1 - raccolta dati 

	// 2 - Conversione posizione GPS (latitudine, longitudine) -> cartesiano ECEF
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