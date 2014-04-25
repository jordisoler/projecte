/*
*	Node per dirigir l'escaner 3D desde ROS
*
*
*	Esquema principal V2
*	
*	Poques desenes de microsegons per a publicar transformada que arriba.
*	Límit de freqüència de gir.
*	Paràmetres:
*		-Frequència de gir.
*		
*	Provat sense PAP.
*	
*	2-04-2014
*/

#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "tf/transform_broadcaster.h"
#include "lidar_scan/CodiGir.h"
#include <math.h>

#define coorx 0.0865
#define coory 0.08
#define coorz 0.0
#define angleInicial 0.0
#define tempsOffset 0.0
#define periodeDefecte 1500
#define maxPeriode 25000	// hokuyo: 40s/volta -> un escaneig per step
#define minPeriode 1250		// hokuyo: 2s/volta
#define blinding 0.7
using namespace std;

const int inca = 15.625*M_PI/periodeDefecte;
static const ros::Duration aCegues(blinding);

class estatPap
{
public:
	bool publicable, preparada;

	estatPap()	{
		angle = angleInicial;
		publicable = false;
		preparada = false;
		dAngle = inca;
	}

	estatPap(double delta)	{
		angle = angleInicial;
		publicable = false;
		preparada = false;
		dAngle = delta;
	}

	void resetejar()	{
		publicable = false;
		preparada = false;
	}

	void preparar()	{
		angle = angle + dAngle;
		preparada = true;
	}

	tf::Transform transform()	{
		tf::Transform tr;
		tf::Quaternion q;
		q.setRPY(-angle*2, -M_PI/2, 0);
		tr.setRotation(q);
		tr.setOrigin( tf::Vector3(coorx, coory, coorz) );
		return tr;
	}

	void timeOut(const std_msgs::Empty& msg)	{
		publicable = true;
	}
	
	void casa(const std_msgs::Empty& msg)	{
		if(ros::Time::now() > darrerFlanc+aCegues){
			angle = angleInicial;
			darrerFlanc = ros::Time::now();
		}
	}
private:

	double angle, dAngle;
	ros::Time darrerFlanc;
};


void publicar(tf::Transform tr){
	static tf::TransformBroadcaster emisor;
	emisor.sendTransform(tf::StampedTransform(tr, ros::Time::now() + ros::Duration(tempsOffset), "base_laser", "hokuyo"));
}

int main(int argc, char **argv){
	int periode;
	int periodeNou;
	
	ros::init(argc, argv, "lidar_scan_node");
	ros::NodeHandle n;
	ros::NodeHandle nh("~");
	ros::Publisher gir_pub = n.advertise<lidar_scan::CodiGir>("/girConfig", 10);
	
	tf::Transform t;
	lidar_scan::CodiGir gir_msg;
	

	if (nh.getParam("periode", periode)){
		if(periode < minPeriode){
			periode = minPeriode;
			nh.setParam("periode", minPeriode);
			ROS_INFO("El periode de stepping ha de ser, com a minim: %u. Aquest ha estat el que s'ha establert.", minPeriode);
		}else if(periode > maxPeriode){
			periode = maxPeriode;
			nh.setParam("periode", maxPeriode);
			ROS_INFO("El periode de stepping ha de ser, com a maxim: %u. Aquest ha estat el que s'ha establert.", maxPeriode);
		}else{
			ROS_INFO("S'ha inicilitzat l'escaneig al periode: %u", periode);
		}
		
	}else	{
		periode = periodeDefecte;
		ROS_INFO("S'ha inicilitzat l'escaneig al periode per defecte: %u", periode);
	}
	
	double increment = 15.625*M_PI/periode;
	estatPap estat = estatPap(increment);
	ros::Subscriber bot_sub = n.subscribe("/syncPap", 5, &estatPap::timeOut, &estat);
	ros::Subscriber flanc_sub = n.subscribe("/home", 5, &estatPap::casa, &estat) ;
	gir_msg.per = periode;
	
	int nio=0;
	while(ros::ok() && nio<=1){
		sleep(1);
		gir_pub.publish(gir_msg);
		nio++;
	}
	
	while (ros::ok())	{
		if(!estat.preparada){
			estat.preparar();
			t = estat.transform();
		}
		if(estat.publicable){
			publicar(t);
			estat.resetejar();
		}

		if(nh.getParam("periode", periodeNou)){
			if(periodeNou != periode){
				if(periodeNou < minPeriode){
					periode = minPeriode;
					nh.setParam("periode", minPeriode);
					ROS_INFO("El periode de stepping ha de ser, com a minim: %u. Aquest ha estat el que s'ha establert.", minPeriode);
				}else if(periodeNou > maxPeriode){
					periode = maxPeriode;
					nh.setParam("periode", maxPeriode);
					ROS_INFO("El periode de stepping ha de ser, com a maxim: %u. Aquest ha estat el que s'ha establert.", maxPeriode);
				}else{
					periode = periodeNou;
					ROS_INFO("S'ha canviat el periode d'escaneig a: %u", periode);
				}
				double increment = 15.625*M_PI/periode;
				estat = estatPap(increment);
					
				gir_msg.per = periode;
				
				int nio=0;
				while(ros::ok() && nio<=1){
					sleep(1);
					gir_pub.publish(gir_msg);
					nio++;
				}
			}
		}
		ros::spinOnce();
	}


	return 0;
}
