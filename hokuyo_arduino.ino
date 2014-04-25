/*
* Rutina de control a baix nivell d'un escaner 3D a partir d'un LIDAR
*
* Esquema principal V2
*
* Funcionalitats:
*   No homing, 1:16 step, frequència desde lidar_scan_node, no senyal de volta.
*
* Compila, provat en arduino, sense PAP.
*
* Vermell, blau, verd, negre
*
* 4-04-2014
*/


#include <ros.h>
#include <ros/time.h>
#include <lidar_scan/CodiGir.h>
#include <std_msgs/Empty.h>
#include <TimerOne.h>

#define entradaOpb A5
#define duradaPols 2
#define periodeDef 1250 //800Hz

//Representació simbòlica dels pins conectats al driver i al sensor òptic (obp).
const int p_dir = 13;
const int p_bot = 12;
const int p_sleep = 11;
const int p_reset = 10;
const int p_m2 = 9;
const int p_m1 = 8;
const int p_m0 = 7;
const int p_enable = 6;
const int p_fault = 5;
const int p_obp = 4;

const int limlectura = 240;
int lecturaOpb;
int num;
int perScan;
bool casa;

void performStep(){
    digitalWrite(p_bot, HIGH);
    delayMicroseconds(duradaPols);
    digitalWrite(p_bot, LOW);
    num++;
}

void configurar(const lidar_scan::CodiGir& missatge){
    Timer1.stop();
    Timer1.detachInterrupt();
    if(!casa){
        homing();
    }
    perScan = 25000/missatge.per;
    Timer1.initialize(missatge.per);
    Timer1.attachInterrupt(performStep);
    casa = false;
}

std_msgs::Empty bota_msg;
std_msgs::Empty flanc_msg;
ros::NodeHandle nh;
ros::Subscriber<lidar_scan::CodiGir> config_sub("girConfig", &configurar );
ros::Publisher flanc("/home", &flanc_msg);
ros::Publisher bota("/syncPap", &bota_msg);

void homing(){
    int i;
    lecturaOpb=analogRead(entradaOpb);
    while(lecturaOpb > limlectura && i < 40){
        performStep();
        lecturaOpb=analogRead(entradaOpb);
        delay(10);
        if(lecturaOpb > limlectura){
            i++;
        }
    }
    /*
    lecturaOpb=analogRead(entradaOpb);
    if(lecturaOpb>limlectura){
        for(int i=0; i<40; i++){
            performStep();
            delay(2);
        }
        
    }*/
    while(lecturaOpb < limlectura){
        performStep();
        lecturaOpb=analogRead(entradaOpb);
        delay(2);
    }
    
    num=0;
    casa = true;
    flanc.publish(&flanc_msg);
}

void setup(){
    pinMode(p_dir, OUTPUT);
    pinMode(p_bot, OUTPUT);
    pinMode(p_sleep, OUTPUT);
    pinMode(p_reset, OUTPUT);
    pinMode(p_m2, OUTPUT);
    pinMode(p_m1, OUTPUT);
    pinMode(p_m0, OUTPUT);
    pinMode(p_enable, OUTPUT);
    pinMode(p_fault, INPUT);
    pinMode(p_obp, OUTPUT);

    digitalWrite(p_dir, LOW);
    digitalWrite(p_sleep, HIGH);
    digitalWrite(p_reset, HIGH);
    digitalWrite(p_m2, HIGH);
    digitalWrite(p_m1, HIGH);
    digitalWrite(p_m0, HIGH);
    digitalWrite(p_enable, LOW);
    digitalWrite(p_obp, HIGH);

    nh.initNode();
    nh.subscribe(config_sub);
    nh.advertise(bota);
    nh.advertise(flanc);
    perScan = 25000/periodeDef;
    //homing();
    num=0;
}

void loop(){
    if(num > perScan){
        bota.publish(&bota_msg);
        num = 0;
        lecturaOpb=analogRead(entradaOpb);
        if(lecturaOpb > limlectura){
            flanc.publish(&flanc_msg);     
        }
    }
    
    nh.spinOnce();
}
