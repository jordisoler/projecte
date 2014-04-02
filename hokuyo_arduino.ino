/*
* Rutina de control a baix nivell d'un escaner 3D a partir d'un LIDAR
*
* Esquema principal V1
*
* Funcionalitats:
*   No homing, paràmetres no implementats, 1:16 step, step a 800Hz (4s/volta), no senyal de volta.
*
* Compila, no provat amb PAP.
*
* 2-04-2014
*/


#include <ros.h>
#include <ros/time.h>
//#include <lidar_scan/CodiGir.h>
#include <std_msgs/Empty.h>
#include "TimerOne.h"

#define entradaOpb A0
#define duradaPols 2
#define periode 1250 //800Hz

const int bot = 13; //Pin de STEP
const int dir = 7;  //Pin de direccio
const int obp = 8;  //Pin del díode del sensor fotoelèctric.
//const int limlectura = 40;
//int lecturaOpb;

/*
void configurar(const lidar_scan::CodiGir& missatge){
  canviarStepping(missatge.bots);
}
*/

std_msgs::Empty bota_msg;
std_msgs::Empty flanc_msg;
ros::NodeHandle nh;
//ros::Subscriber<lidar_scan::CodiGir> config_sub("/girCongif", &configurar );
ros::Publisher flanc("/home", &flanc_msg);
ros::Publisher bota("/syncPap", &bota_msg);
int num = 0;


void setup(){
  pinMode(bot, OUTPUT);
  //pinMode(obp, OUTPUT);
  pinMode(dir, OUTPUT);
  digitalWrite(dir, LOW);
  nh.initNode();
  //nh.subscribe(config_sub);
  nh.advertise(bota);
  nh.advertise(flanc);
  Timer1.initialize(periode); 
  Timer1.attachInterrupt(performStep);
}

void performStep(){
  digitalWrite(bot, HIGH);
  delayMicroseconds(duradaPols);
  digitalWrite(bot, LOW);
  num++;
}

void loop(){
  if(num > 20){
      bota.publish(&bota_msg);
      num=0;
  }
  //lecturaOpb=analogRead(entradaOpb);
  //if(lecturaOpb > limlectura){
  //    flanc.publish(&flanc_msg);
  //}
  nh.spinOnce();
}
