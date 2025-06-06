#include "ros/ros.h" //include le API di ROS
#include "geometry_msgs/PointStamped.h" //definisce il tipo di messaggio che il nodo riceve
/*geometry_msgs/PointStamped:
    y = speed [km/h]
    x = steer at the steering wheel [deg]
*/
#include "nav_msgs/Odometry.h" //definisce il tipo di messaggio che il nodo invia
#include <tf/transform_broadcaster.h>


#include "first_project/odometry_tools.hpp" //include il file di intestazione che contiene le dichiarazioni delle funzioni


odometer_tools::positionState state{};

//funzione di callback che viene richiamata
void speedSteerCallback(const geometry_msgs::PointStamped::ConstPtr& msg, ros::Publisher& odom_pub){
	ROS_INFO_STREAM("Messaggio ricevuto");

	//1 - raccolta i dati dal messaggio
	ros::Time current_time = msg->header.stamp;
	double steer_wheel = msg->point.x;// angolo volante in gradi
	double speed = msg->point.y;// velocità in km/h

	if(steer_wheel<10 && steer_wheel>-10){
		steer_wheel=0;
	}

	//2 - conversione unità di misura
	double steering_angle = odometer_tools::steerConvert(steer_wheel);// angolo volante in gradi

	steering_angle = odometer_tools::degToRad(steering_angle);// angolo volante in radianti
	speed = odometer_tools::speedConvert(speed);// velocità in m/s


	//3 - Ackermann steering bicyle approximation
	double angular_speed = odometer_tools::getAngularSpeed(steering_angle, speed);// velocità angolare in rad/s

	//4 - Integrazione nel tempo - POSITION
	state = rungeKuttaIntegration(state, speed, angular_speed, current_time);// integrazione nel tempo

	//5 - conversione in quaternione - ORIENTATION
	geometry_msgs::Quaternion orientation = odometer_tools::quaternionConversion(state);// conversione in quaternione

	//6 - creazione del messaggio
	nav_msgs::Odometry odom_msg;
	odom_msg.header.stamp = current_time;
	odom_msg.header.frame_id = "odom";
	odom_msg.child_frame_id = "base_link";
	//posizione in Sistema di riferimento del mondo "odom"
	odom_msg.pose.pose.position.x = -state.y;
	odom_msg.pose.pose.position.y = state.x;
	odom_msg.pose.pose.position.z = 0.0;
	odom_msg.pose.pose.orientation = orientation;
	//velocità in Sistema di riferimento del robot "baes_link"
	odom_msg.twist.twist.linear.x = speed;
	odom_msg.twist.twist.linear.y = 0.0;
	odom_msg.twist.twist.linear.z = 0.0;
	odom_msg.twist.twist.angular.x = 0.0;
	odom_msg.twist.twist.angular.y = 0.0;
	odom_msg.twist.twist.angular.z = angular_speed;

	//7 - pubblicazione del messaggio odometry
	odom_pub.publish(odom_msg);

	//8 - pubblicazione tf
	static tf::TransformBroadcaster odom_broadcaster;
	tf::Transform odom_trans;
	odom_trans.setOrigin(tf::Vector3(-state.y, state.x, 0.0));
	tf::Quaternion quat;
	tf::quaternionMsgToTF(orientation, quat);
	odom_trans.setRotation(quat);

	odom_broadcaster.sendTransform(tf::StampedTransform(
		odom_trans,
		current_time,
		"odom",
		"base_link"
	));
	
}




int main(int argc, char **argv){
	ros::init(argc, argv, "odometer");//inizializza il nodo listener

	ros::NodeHandle n;//crea publisher, subscriber...

	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10); // inizializzazione publisher
  	ros::Subscriber sub = n.subscribe<geometry_msgs::PointStamped>("speedsteer", 1,
        [&odom_pub](const geometry_msgs::PointStamped::ConstPtr& msg) {
            speedSteerCallback(msg, odom_pub);//sottosrive il nodo al topic e ogni volta che riceve un messaggi richiama la funzione chatterCallback
		});


  	ros::spin();

  return 0;
}
