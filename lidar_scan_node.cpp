/*
*	Node per dirigir l'escaner 3D desde ROS
*
*
*	Esquema principal V2
*	
*	El frame va més lent que la realitat.
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
#include "laser_assembler/AssembleScans.h"
#include "sensor_msgs/PointCloud.h"

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
bool nodeMort;

class estatPap
{
public:
	estatPap(int periode)	{
		gir_pub = n_.advertise<lidar_scan::CodiGir>("/girConfig", 10, true);
		scan3d_pub = n_.advertise<sensor_msgs::PointCloud> ("assembled_cloud", 1);
		sclient = n_.serviceClient<laser_assembler::AssembleScans>("assemble_scans");
		gir_msg.per = periode;
		
		angle = angleInicial;
		dAngle = 15.625*M_PI/periode;
		gir_msg.header.stamp = ros::Time::now();
		gir_pub.publish(gir_msg);
	}

	void casa(const std_msgs::Empty& msg)	{
		if(ros::Time::now() > darrerFlanc+aCegues){
			angle = angleInicial;
			darrerFlanc = ros::Time::now();
		}
	}
	
	void publicarTf(const std_msgs::Empty& msg){
		angle = angle + dAngle;
		/*if(compte > 180/M_PI){
			compte = 0;
			publicarScan();
		}else{
			compte++;
		}*/
		tf::Transform tr;
		tf::Quaternion q;
		q.setRPY(-angle*2, -M_PI/2, 0);
		tr.setRotation(q);
		tr.setOrigin( tf::Vector3(coorx, coory, coorz) );
				
		static tf::TransformBroadcaster emisor;
		emisor.sendTransform(tf::StampedTransform(tr, ros::Time::now() + ros::Duration(tempsOffset), "base_laser", "hokuyo"));
	}
	
	void acaba(){
		gir_msg.per = 0;
		gir_msg.header.stamp = ros::Time::now();
		gir_pub.publish(gir_msg);
	}
	
private:

	void publicarScan(){
		ros::Time tempsActual = ros::Time::now();
		laser_assembler::AssembleScans srv;
		srv.request.begin = tempsDarrer;
		srv.request.end   = tempsActual;

		if (sclient.call(srv)){
		  ROS_INFO("S'ha publicat un escaneig amb %u punts", (uint32_t)(srv.response.cloud.points.size())) ;
		  scan3d_pub.publish(srv.response.cloud);
		}
		else{
		  ROS_ERROR("Error en sol.licitar dades a l'acumulador\n") ;
		}
		tempsDarrer = tempsActual;
	}
	int compte;
	double angle, dAngle;
	ros::Time darrerFlanc;
	ros::NodeHandle n_;
	ros::ServiceClient sclient;
	ros::Publisher gir_pub, scan3d_pub;
	ros::Time tempsDarrer;
	lidar_scan::CodiGir gir_msg;
};

void prepararSortida(const ros::TimerEvent&){
	
}
int main(int argc, char **argv){
	int periode;
	bool sortir = false;
	
	nodeMort = false;
	ros::init(argc, argv, "lidar_scan_node");
	ros::NodeHandle n;
	ros::NodeHandle nh("~");

	if (nh.getParam("periode", periode)){
		if(periode < minPeriode && periode != 0){
			periode = minPeriode;
			nh.setParam("periode", minPeriode);
			ROS_INFO("El periode de stepping ha de ser, com a minim: %u. Aquest ha estat el que s'ha establert.", minPeriode);
		}else if(periode == 0){
			sortir = true;
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
	
	estatPap estat = estatPap(periode);
	
		
	ros::Subscriber bot_sub = n.subscribe("/syncPap", 5, &estatPap::publicarTf, &estat);
	ros::Subscriber flanc_sub = n.subscribe("/home", 5, &estatPap::casa, &estat);

	if(!sortir){
		ros::spin();
	}else{
		ros::Timer timer = n.createTimer(ros::Duration(0.5), prepararSortida);
		while(!nodeMort){
			ros::spinOnce();
		}
	}
	
	
	return 0;
}
