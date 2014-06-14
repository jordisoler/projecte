/**************************************************************
*
*   Programa per a controlar els dispositius físics 
*   de l'escànner 3D.
*   
**************************************************************/


#include <ros.h>
#include <ros/time.h>
#include <lidar_scan/CodiGir.h>
#include <TimerOne.h>

// Missatges
#include <std_msgs/UInt32.h>
#include <std_msgs/Empty.h>

// Definició de constants
#define duradaPols 2
#define periodeDef 1250 //Freqüència per defecte del motor: 800Hz

//Representació simbòlica dels pins conectats al driver i al sensor òptic (obp).
#define entradaOpb A5
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

const int limlectura = 24;
int lecturaOpb;
bool casa;

// Dur a terme un pas
void performStep(){
    digitalWrite(p_bot, HIGH);
    delayMicroseconds(duradaPols);
    digitalWrite(p_bot, LOW);
}

// Configurar la velocitat d'escaneig i inicialització del temporitzador
void configurar(const lidar_scan::CodiGir& missatge){
    Timer1.stop();
    Timer1.detachInterrupt();
    
    if(!casa){
        homing();
    }
    if(missatge.per != 0){
        Timer1.initialize(missatge.per);
        Timer1.attachInterrupt(performStep);
        casa = false;
    }
}

// Declaració dels objectes de ROS a utilitzar
std_msgs::Empty flanc_msg;
ros::NodeHandle nh;
ros::Subscriber<lidar_scan::CodiGir> config_sub("girConfig", &configurar );
ros::Publisher flanc("/home", &flanc_msg);

// Recerca del punt de referència
void homing(){
    int i;
    lecturaOpb=analogRead(entradaOpb);
    while(lecturaOpb > limlectura && i < 40){
        performStep();
        lecturaOpb=analogRead(entradaOpb);
        delay(5);
        if(lecturaOpb > limlectura){
            i++;
        }
    }
    while(lecturaOpb < limlectura){
        performStep();
        lecturaOpb=analogRead(entradaOpb);
        delay(2);
    }
    
    casa = true;
    flanc.publish(&flanc_msg);
}

// Inicialització dels d'entrades i sortides i del node de ROS
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
    nh.advertise(flanc);
}

// Bucle infinit a l'espera d'interrupcions
void loop(){
    nh.spinOnce();
}
