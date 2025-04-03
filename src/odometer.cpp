#include "ros/ros.h" //include le API di ROS
#include "geometry_msgs/PointStamped.h" //definisce il tipo di messaggio che il nodo riceve
/*geometry_msgs/PointStamped:
    y = speed [km/h]
    x = steer at the steering wheel [deg]
*/
#include "nav_msgs/Odometry.h" //definisce il tipo di messaggio che il nodo invia


#include "first_project/odometry_tools.hpp" //include il file di intestazione che contiene le dichiarazioni delle funzioni



//funzione di callback che viene richiamata
void speedSteerCallback(const geometry_msgs::PointStamped::ConstPtr& msg){
	ROS_INFO_STREAM("Messaggio ricevuto");

	//1 - raccolta i dati dal messaggio
	ros::Time time = msg->header.stamp;
	double steer_wheel = msg->point.x;// angolo volante in gradi
	double speed = msg->point.y;// velocità in km/h

	//2 - conversione unità di misura
	double steering_angle = odometer_tools::steerConvert(steer_wheel);// angolo volante in gradi
	steering_angle = odometer_tools::degToRad(steering_angle);// angolo volante in radianti
	speed = odometer_tools::speedConvert(speed);// velocità in m/s

	//3 - Ackermann steering bicyle approximation
	double angular_speed = odometer_tools::getAngularSpeed(steering_angle, speed);// velocità angolare in rad/s

   //4 - Integrazione nel tempo

   //5 - conversione in quaternione
}




int main(int argc, char **argv){
	ros::init(argc, argv, "odometer");//inizializza il nodo listener

	ros::NodeHandle n;//crea publisher, subscriber...
  	ros::Subscriber sub = n.subscribe("speedsteer", 1, speedSteerCallback);//sottosrive il nodo al topic e ogni volta che riceve un messaggi richiama la funzione chatterCallback

  	ros::spin();

  return 0;
}