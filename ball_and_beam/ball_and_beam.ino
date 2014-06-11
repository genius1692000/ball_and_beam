#include <PID_v1.h>
#include <Servo.h> 
#include <NewPing.h>

#define TRIGGER_PIN  12
#define ECHO_PIN     11
#define MAX_DISTANCE 60

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

double Setpoint, Input, Output;
double kp, ki, kd;
float distance;

Servo myservo;
PID myPID(&Input, &Output, &Setpoint,kp,ki,kd, DIRECT);

void setup() {
  Serial.begin(115200);
  myservo.attach(3);
  kp = 0.8;
  ki = 0.07;
  kd = 0.12;
  Setpoint = 20;

  myPID.SetTunings(kp, ki, kd);
  myPID.SetOutputLimits	(62, 88);
  myPID.SetMode(AUTOMATIC);
}

void loop() {
  //Read the position of ball, the sensor ping 4 times 
  //and get the average ecoh time, 
  //US_ROUNDTRIP_CM is a constance set by the ping library 
 unsigned int uS = sonar.ping_median(5);
  distance = uS/ US_ROUNDTRIP_CM;  
  //distance = sonar.ping_cm();

  //print the output
  Serial.println(distance);

  //compute teh PID controller
  Input = distance;
  myPID.Compute();
  myservo.write(Output);

  //turn the PID controller if serial is available
  if (Serial.available() > 0) {
    kp  = Serial.parseFloat(); 
    ki = Serial.parseFloat(); 
    kd = Serial.parseFloat();  
    myPID.SetTunings(kp, ki, kd);
  }
}
