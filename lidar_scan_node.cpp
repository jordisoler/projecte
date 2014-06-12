/*
*	
*/

//#include <cstdio>
#include <math.h>

#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "std_msgs/UInt32.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/LaserScan.h"
#include "lidar_scan/CodiGir.h"

#include "tf/transform_broadcaster.h"
#include "laser_assembler/AssembleScans.h"


#define coorx 0.0865
#define coory 0.08
#define coorz 0.0
#define angleInicial 0.0
#define tempsOffset 0.0		//Temps que passa entre l'instant en que el motor duu a terme un 'n' steps fins que aquests són publicats
#define periodeDefecte 1500
#define maxPeriode 25000	// hokuyo: 40s/volta -> un escaneig per step
#define minPeriode 1250		// hokuyo: 2s/volta
#define blinding 5.7		// Periode de temps en que les senyals del sensor reflectiu son ignorades

using namespace std;

static const ros::Duration aCegues(blinding); //variable de classe en funcio de periode

class estatPap{
public:
	estatPap(int periode)	{
		gir_pub = n_.advertise<lidar_scan::CodiGir>("/girConfig", 10, true);
		scan3d_pub = n_.advertise<sensor_msgs::PointCloud> ("pc_most_intense", 1);
		sclient = n_.serviceClient<laser_assembler::AssembleScans>("assemble_scans");
		
		angle = angleInicial;
		primera_vegada = false;
		viu = true;
		//aCegues(blinding);
		periodeActual = periode;
		incAngle=125*M_PI/4/periode;
		compte = 0;
		
		
		gir_msg.per = periode;
		gir_msg.header.stamp = ros::Time::now();
		gir_pub.publish(gir_msg);
	}

	void casa(const std_msgs::Empty& msg)	{			
		if(primera_vegada){
			angle = angleInicial;
			primera_vegada=false;
		}
	}
	
	//void syncCallback(const std_msgs::UInt32& msg){
	//	
	//}
	
	void laserCallback(const sensor_msgs::LaserScan& scan){
		if (!primera_vegada){
			ros::Time temps = scan.header.stamp;
			publicarTf(temps);
		}
	}
	
	void publicarTf(ros::Time temps){
		angle = angle + incAngle;
		
		tf::Transform tr;
		tf::Quaternion q;
		q.setRPY(-angle*2, -M_PI/2, 0);
		tr.setRotation(q);
		tr.setOrigin( tf::Vector3(coorx, coory, coorz) );
				
		static tf::TransformBroadcaster emisor;
		emisor.sendTransform(tf::StampedTransform(tr, temps + ros::Duration(tempsOffset), "base_laser", "hokuyo"));
	}
	
	void publicarScan(const ros::TimerEvent& e){
		if (primera_vegada){
		  primera_vegada = false;
		  return;
		}
		if (compte > 20){
			acaba();
			return;
		}else{
			compte ++;
	
			laser_assembler::AssembleScans srv;
			srv.request.begin = e.last_real;
			srv.request.end   = e.current_real;
	
			if (sclient.call(srv)){
			  ROS_INFO("S'ha publicat l'escaneig %u amb %u punts", compte, (uint32_t)(srv.response.cloud.points.size())) ;
			  scan3d_pub.publish(srv.response.cloud);
			}else{
			  ROS_ERROR("Error en sol.licitar dades a l'acumulador\n") ;
			}
		}
	}
	
	void acaba(){
		gir_msg.per = 0;
		gir_msg.header.stamp = ros::Time::now();
		gir_pub.publish(gir_msg);
		viu = false;
	}
	
	bool viu;
	
private:	
	double angle, incAngle;
	bool primera_vegada;
	int compte;
	int passos;
	int periodeActual;
	ros::Time darrerFlanc;
	ros::Duration aCegues;
	ros::Timer escaneigPeriodic;
	ros::NodeHandle n_;
	ros::ServiceClient sclient;
	ros::Publisher gir_pub, scan3d_pub;
	lidar_scan::CodiGir gir_msg;
};

int main(int argc, char **argv){
	int periode;
	
	ros::init(argc, argv, "lidar_scan_node");
	ros::NodeHandle n;
	ros::NodeHandle nh("~");

	ROS_INFO("Esperant a l'acumulador");
	ros::service::waitForService("build_cloud");
	ROS_INFO("Sistema preparat per a comencar l'escaneig");
	
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
		
	}else {
		periode = periodeDefecte;
		ROS_INFO("S'ha inicilitzat l'escaneig al periode per defecte: %u", periode);
	}
	
	estatPap estat = estatPap(periode);
	
	//ros::Subscriber bot_sub = n.subscribe("/syncPap", 5, &estatPap::syncCallback, &estat);
	ros::Subscriber flanc_sub = n.subscribe("/home", 5, &estatPap::casa, &estat);
	ros::Subscriber laser_sub = n.subscribe("/most_intense", 5, &estatPap::laserCallback, &estat);

	while(estat.viu && ros::ok()){
		ros::spinOnce();
	}
	
	return 0;
}



