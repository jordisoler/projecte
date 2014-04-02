/*
*	Node per dirigir l'escaner 3D desde ROS
*
*
*	Esquema principal V1
*	
*	Provat sense PAP.
*	
*	2-04-2014
*/

#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "tf/transform_broadcaster.h"
//#include "lidar_scan/CodiGir.h"
#include <math.h>

#define coorx 0.0865
#define coory 0.08
#define coorz 0.0 //Modificar
#define angleInicial 0.0
#define tempsOffset 0.0
#define frec 800
using namespace std;

const int inca = frec*M_PI/64000;
class estatPap
{
public:
	bool publicable, preparada;
	
	estatPap()
	{
		angle = angleInicial;
		publicable = false;
		preparada = false;
		dAngle = inca;
	}
	
	estatPap(double delta)
	{
		angle = angleInicial;
		publicable = false;
		preparada = false;
		dAngle = delta;
	}
	
	void resetejar()
	{
		publicable = false;
		preparada = false;
	}
	
	void preparar()
	{
		angle = angle + dAngle;
		preparada = true;
	}
	
	tf::Transform transform()
	{
		tf::Transform tr;
		tf::Quaternion q;
		q.setRPY(angle*2, -M_PI/2, 0);
		tr.setRotation(q);
		tr.setOrigin( tf::Vector3(coorx, coory, coorz) );
		return tr;
	}
	
	void timeOut(const std_msgs::Empty& msg)
	{
		publicable = true;
	}
private:
	
	double angle, dAngle;
};

void publicar(tf::Transform tr)
{
	static tf::TransformBroadcaster emisor;
	emisor.sendTransform(tf::StampedTransform(tr, ros::Time::now() + ros::Duration(tempsOffset), "base_laser", "hokuyo"));
}

int main(int argc, char **argv)
{
	double increment = 800*M_PI/64000;
	ROS_INFO("He enviat: %f", increment);
	estatPap estat = estatPap(increment);
	ros::init(argc, argv, "lidar_scan_node");
	ros::NodeHandle n;
	//ros::Publisher gir_pub = n.advertise<lidar_scan::CodiGir>("/gir", 10);
	ros::Subscriber bot_sub = n.subscribe("/syncPap", 5, &estatPap::timeOut, &estat);
	//ros::Subscriber flanc_sub;
	tf::Transform t;
	while (ros::ok())
	{
		if(!estat.preparada)
		{
			estat.preparar();
			t = estat.transform();
		}
		if(estat.publicable)
		{
			publicar(t);
			estat.resetejar();
		}
		
		ros::spinOnce();
	}
	
	
	return 0;
}










