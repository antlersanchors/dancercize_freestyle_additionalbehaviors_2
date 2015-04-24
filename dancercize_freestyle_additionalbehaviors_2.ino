// CenterAB - both motors
// xA->Freqequency1, xB->Frequency2
//CenterA at xB, CenterB at xA
//feels like "Slave"

#include <spi4teensy3.h>
#include <EEPROM.h>
#include <M3T3.h>

int duty, count, fout;
int xA, xB;
int xA_old, xB_old; // for our new audio accents
boolean stepping = false;
int stepThreshold = 50;
int stepA_dist, stepB_dist;

long prevTime;

int xt, x, xold, F;
int K = 20;


void setup(){
  Serial.begin(9600);
  MotorA.init();
  MotorB.init();
  Music.init();
  Music.setWaveform1(0);//8bit default?
  Music.setWaveform2(0);
  Music.setGain1(1.0f);
  Music.setGain2(1.0f);
  
  analogReadAveraging(32);

  prevTime = millis();
}

void loop(){
  
  xA = analogRead(A1);
  Music.setFrequency1(map (xA, 0, 1023, 40, 2000)); 
  
  xB = analogRead(A9);
  Music.setFrequency2(map (xB, 0, 1023, 40, 2000));  

  dance();

  steppingSound();

  
  if(abs(xA-xB) >= (800)) {
//feel increasing heartbeat?
  awkward(); 
  Serial.println("xA-xB");
  Serial.println(xA-xB); 
  } else {
   dance(); 
  // Serial.println("Dancing!");
  }

}

void steppingSound(){
  stepA_dist = xA_old - xA;
  stepB_dist = xB_old - xB;

  if (stepA_dist >= stepThreshold) {

    stepping = true;

    Music.setWaveform1(FUZZ);
    Music.setWaveform2(NOISE);
    Music.setFrequency1(10000);
    Music.setGain1(0.001 / stepA_dist);
    Music.setGain2(0.001 / stepA_dist);
    Serial.println("stepping soundA");

  } else if (stepB_dist >= stepThreshold) {

    stepping = true;

    Music.setWaveform1(FUZZ);
    Music.setWaveform2(NOISE);
    Music.setFrequency1(12000);
    Music.setGain1(0.001);
    Music.setGain2(0.001);
    Serial.println("stepping soundB");

  } else {

    stepping = false;
    
  }

  if (stepping = false) {

    Music.setGain(0.9995f*Music.getGain());
  }
  
  if(millis() - prevTime >= 500){
    xA_old = xA;
    xB_old = xB;
    prevTime = millis();
  }
}

void dance() {
     
  int foutA = -0.3*(xA-xB); 
  MotorA.torque(foutA);  
 
  int foutB = -0.3*(xB-xA); 
  MotorB.torque(foutB);   
  
}

//void steps() {
// 
//  xold = xA;
//  xt = xA % 250; //same force for each 250 ranage
//  F = 0;
//  if (xt > 60) F = - K * (xt - 60);
//  if (xt > 80) F = - K * (100 - xt);
//  if (xt > 120) F =  K * (140 - xt);
//  if (xt > 140) F = 0;
//  MotorA.torque(F);
//    
//  
//}

//void wobble() {
//  float k = 15; // spring coefficient (stiffness)
//  float m = 50; // mass
//  float d = 2.0;  // damping coefficient
//
//  float x = analogRead(A1);
//  float v;
//  float f;
//
//  long tick = 0;
//  int last_pos = 0;
//
//  long tick_now = millis();  
//  float dt = (float)(tick_now - tick) / 100.0f;
//        
//  int current_pos = analogRead(A1);  
//  
//  f = k * (current_pos - x) - (d * v);
//  v += (f / m) * dt;
//  x += v * dt;
//      
//  tick = millis();
//  
//  int t = map(f, -5000, 5000, -512, 512);   // re-mapping the force to a (valid) torque value
//  MotorA.torque(t); 
//  
//}

void awkward() {

    xA = analogRead(A1);
    xB = analogRead(A9);  
    
    int p = 120;
    
    int foutA = -3*(xA-xB); 
    MotorA.torque(foutA/2);  
 
    int foutB = -3*(xB-xA); 
    MotorB.torque(foutB/2); 
  
    MotorA.start();
    MotorB.start();
    MotorA.torque(foutA + p);
//    Music.noteOn(map(analogRead(A1),0,1023,30,60));
    MotorB.torque(foutB + p);
    delay (10);
//    Music.noteOff();
    MotorA.torque(0);
    MotorB.torque(0);
    delay (175);
    MotorA.torque(foutA - p);
//    Music.noteOn(map(analogRead(A1),0,1023,35,65));
    MotorB.torque(foutB - p);
    delay (20);
//    Music.noteOff();
    MotorA.torque(0);
    MotorB.torque(0);
//    MotorA.stop();
//    MotorB.stop();
    delay (375);
  
  
}


