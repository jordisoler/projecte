/*
*	Node per dirigir l'escaner 3D desde ROS
*
*
*	27-03-2014
*/

#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "tf/transform_broadcaster.h"
#include "panell_control/CodiGir.h"
#include <math.h>

#define coorx 0.0865
#define coory 0.08
#define coorz 0.0
#define angleInicial 0.0
#define tempsOffset 0.0

using namespace std;

class ControlEscaneig
{
public:
	ControlEscaneig()
	{
		gir_pub = n.advertise<panell_control/CodiGir>("/gir", 10);
		bot_pub = n.advertise<std_msgs/Empty>("/controlBot", 5);
		t_pub = n.advertise<tf/Transform>
		flanc_sub = n.subscribe("/home", 100, &ControlEscaneig::resetejar);
		ros::Timer timer = n.createTimer(ros::Duration(0.025), &ControlEscaneig::publica);
		angle = angleInicial;
		incAngle = M_PI/100;
		tllest = false;
		t.setOrigin( tf::Vector3(coorx, coory, coorz) );
		posicionar();
	}

	void iniciar()
	{
		while(ros::ok())
		{
			if (!tllest)
			{
				angle = angle + incAngle;
				posicionar();
			}
			ros::spinOnce();
		}
	}

	void posicionar()
	{
		tf::Quaternion q;
		q.setRPY(0, 0, angle);
		t.setRotation(q);
		tllest = true;
	}

	void publica(const ros::TimerEvent&)
	{
		std_msgs::Empty bot_msg;
		panell_control::CodiGir gir_msg;
		tf::TransformBroadcaster emisor;
		bot_pub.publish(bot_msg);
		emisor.sendTransform(tf::StampedTransform(t, ros::Time::now() + ros::Duration(tempsOffset), "base_laser", "hokuyo"));
		if (!tllest)
		{
			ROS_ERROR("S'ha produit un error en l'enviament dels frames. La informació enviada no està actualitzada.")
		}
		tllest = false;
	}

	void resetejar(const std_msgs::Empty& entrada)
	{

	}
private:
	ros::NodeHandle n;
	ros::Publisher gir_pub;
	ros::Publisher bot_pub;
	ros::Publisher t_pub;
	ros::Subscriber flanc_sub;
	ros::Transform t;
	ros::Timer 
	int stepping;
	bool tllest;
	double angle;
};
int main(int argc, char **argv)
{
	ros::init(argc, argv, "panell_node");	

	ControlEscaneig controlador;
	controlador.iniciar();

	return 0;
}