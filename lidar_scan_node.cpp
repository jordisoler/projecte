/***********************************************************************
*	
*	Node de control del funcionament de l'escànner 3D.	
*	
************************************************************************/

#include <math.h>
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"

// Missatges
#include "std_msgs/Empty.h"
#include "sensor_msgs/LaserScan.h"
#include "lidar_scan/CodiGir.h"

/* Constants de la geometria de l'escànner i relacionades
 * amb el període del motor									 */
#define coorx 0.0865
#define coory 0.08
#define coorz 0.0
#define angleInicial 0.0
#define periodeDefecte 1500
#define maxPeriode 25000
#define minPeriode 1250

using namespace std;

// Classe encarregada del control de l'escaneig
class lidarScan{
public:
	
	// Inicialització dels atributs
	lidarScan(int periode)	{
		gir_pub = n_.advertise<lidar_scan::CodiGir>("/girConfig", 10, true);
		
		angle = angleInicial;
		pos_coneguda = false;
		viu = true;
		periodeActual = periode;
		incAngle=125*M_PI/4/periode;		
		
		gir_msg.per = periode;
		gir_msg.header.stamp = ros::Time::now();
		gir_pub.publish(gir_msg);
	}

	// Làser a la posició inicial
	void casa(const std_msgs::Empty& msg){			
		if(!pos_coneguda){
			angle = angleInicial;
			pos_coneguda=true;
		}
	}
		
	
	// Publicar la transformada entre la base de l'escànner i el sensor òptic
	void publicarTf(const sensor_msgs::LaserScan& scan){
		if (pos_coneguda){
			ros::Time temps = scan.header.stamp;
			angle = angle + incAngle;
			
			tf::Transform tr;
			tf::Quaternion q;
			q.setRPY(-angle*2, -M_PI/2, 0);
			tr.setRotation(q);
			tr.setOrigin( tf::Vector3(coorx, coory, coorz) );
					
			static tf::TransformBroadcaster emisor;
			emisor.sendTransform(tf::StampedTransform(tr, temps, "base_laser_e2", "hokuyo_e2")); //--------------CANVI
		}
	}
	
	// Finalitzar l'esnaceig
	void acaba(){
		gir_msg.per = 0;
		gir_msg.header.stamp = ros::Time::now();
		gir_pub.publish(gir_msg);
		viu = false;
	}
	
	bool viu;
	
private:
	ros::NodeHandle n_;
	ros::Publisher gir_pub;
	lidar_scan::CodiGir gir_msg;
	double angle, incAngle;
	bool pos_coneguda;
	int periodeActual;
};

int main(int argc, char **argv){
	int periode;
	
	ros::init(argc, argv, "lidar_scan_node");
	ros::NodeHandle n;
	ros::NodeHandle nh("~");

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
	
	lidarScan escanner = lidarScan(periode);
	
	ros::Subscriber flanc_sub = n.subscribe("/home", 5, &lidarScan::casa, &escanner);
	ros::Subscriber laser_sub = n.subscribe("/first", 5, &lidarScan::publicarTf, &escanner);

	while(escanner.viu && ros::ok()){
		ros::spinOnce();
	}
	return 0;
}



