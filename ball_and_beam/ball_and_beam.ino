#include <PID_v1.h>
#include <Servo.h> 

double Setpoint, Input, Output;
#define trigPin 2
#define echoPin 3
long duration;
float distance, filtered_distance, sum;
double kp, ki, kd;
Servo myservo;
PID myPID(&Input, &Output, &Setpoint,kp,ki,kd, DIRECT);

float filter[] = {
  141667814, 30535347, 33525554, 36521970, 39499313, 42412875, 
  45222844, 47904896, 50434808, 52787330, 54946901, 56905861, 
  58644063, 60115628, 61271939, 62115584, 62741988, 63086021, 
  63086021, 62741988, 62115584, 61271939, 60115628, 58644063, 
  56905861, 54946901, 52787330, 50434808, 47904896, 45222844, 
  42412875, 39499313, 36521970, 33525554, 30535347, 141667814
};
float temp_data[sizeof(filter)/sizeof(filter[0])]; 

void setup() {
  Serial.begin (9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  myservo.attach(9);

  kp = 6;
  ki = 4.5;
  kd = 4.5;
  Setpoint = 25;

  myPID.SetTunings(kp, ki, kd);
  myPID.SetOutputLimits	(-45,45);
  myPID.SetMode(AUTOMATIC);

  for (int i =0; i<sizeof(filter)/sizeof(filter[0]); i++)
    sum+=filter[i];
}

void loop() {

  digitalWrite(trigPin, LOW);  
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); 
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  if ((duration/2) / 29.1 <=200 && (duration/2) / 29.1 >= 0) 
    distance = (duration/2) / 29.1;

  filtered_distance = 0;
  for (int i = 1; i< sizeof(filter)/sizeof(filter[0]); i++) 
    temp_data[i-1] = temp_data[i];

  temp_data[sizeof(filter)/sizeof(filter[0])-1] = distance;

  for (int i = 0; i < sizeof(filter)/sizeof(filter[0]); i++) 
    filtered_distance += temp_data[i]*filter[i]/sum;

  Serial.print(distance);
  Serial.print(", ");
  Serial.println(filtered_distance);

   Input = filtered_distance;
    myPID.Compute();
    myservo.write(1470-Output); 


  if (Serial.available() > 0) {
    kp  = Serial.parseFloat(); 
    ki = Serial.parseFloat(); 
    kd = Serial.parseFloat();  
    myPID.SetTunings(kp, ki, kd);
  }
}





















