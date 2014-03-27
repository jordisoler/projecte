/*
* Rutina de control a baix nivell d'un escaner 3D a partir d'un LIDAR
*
* Funcionalitats:
*   - Comunicació amb ros:
*       + Rep un missatge buit quan s'ha de fer un bot.
*       + Rep el missatge propi 'CodiGir' indicant el stepping.
*       + Envia un missatge buit quan el sensor fotoelèctric reconeix 
*         el punt singular.
*   - Enviar les senyals apropiades a les entrades 'step', 'dir', 'ms1',
*     'ms2' i 'ms3' segons les dades rebudes a través de ROS.
*   
* 26-03-2014
*/


#include <ros.h>
#include <ros/time.h>
#include <panell_control/CodiGir.h>
#include <std_msgs/Empty.h>

#define entradaOpb A0
#define duradaPols 50

const int bot = 13; //Pin de STEP
const int ms1 = 11; //Pin de MS1
const int ms2 = 10; //Pin de MS2
const int ms3 = 9;  //Pin de MS3
const int dir = 7;  //Pin de direccio
const int obp = 8;  //Pin del díode del sensor fotorlèctric.
const int limlectura = 40;
int lecturaOpb;


void resposta(const panell_control::CodiGir& missatge){
  canviarStepping(missatge.bots);
}

void actua(const std_msgs::Empty& missatge){
    digitalWrite(bot, HIGH);
    delayMicroseconds(duradaPols);
    digitalWrite(bot, LOW);
}

std_msgs::Empty msg;
ros::NodeHandle nh;
ros::Subscriber<panell_control::CodiGir> sub("/gir", &resposta );
ros::Subscriber<std_msgs::Empty> bota("/controlBot", &actua);
ros::Publisher flanc("/home", &msg);



void canviarStepping(int n){
  switch(n){
    case 2:      //Mig step
      digitalWrite(ms1, HIGH);
      digitalWrite(ms2, LOW);
      digitalWrite(ms3, LOW);
      break;
    case 3:      //Quart de step
      digitalWrite(ms1, LOW);
      digitalWrite(ms2, HIGH);
      digitalWrite(ms3, LOW);
      break;
    case 4:      //Octau de Step
      digitalWrite(ms1, HIGH);
      digitalWrite(ms2, HIGH);
      digitalWrite(ms3, LOW);
      break;
    case 5:      //Setzau de Step
      digitalWrite(ms1, HIGH);
      digitalWrite(ms2, HIGH);
      digitalWrite(ms3, HIGH);
      break;
    default:     //Full Step
      digitalWrite(ms1, LOW);
      digitalWrite(ms2, LOW);
      digitalWrite(ms3, LOW);
      break;
  }
}

void setup(){
  pinMode(bot, OUTPUT);
  pinMode(ms1, OUTPUT);
  pinMode(ms2, OUTPUT);
  pinMode(obp, OUTPUT);
  pinMode(dir, OUTPUT);
  digitalWrite(dir, LOW);
  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(bota);
  nh.advertise(flanc);
}

void loop(){
  lecturaOpb=analogRead(entradaOpb);
  if(lecturaOpb > limlectura){
      flanc.publish(&msg);
  }
  nh.spinOnce();
}