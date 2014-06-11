/*

FIR filter designed with
 http://t-filter.appspot.com

sampling frequency: 2000 Hz

* 0 Hz - 21 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = 1.7689342482361 dB

* 100 Hz - 1000 Hz
  gain = 0
  desired attenuation = -80 dB
  actual attenuation = -87.32574811904081 dB

*/


#include <PID_v1.h>
#include <Servo.h> 
#include <NewPing.h>

#define TRIGGER_PIN  12
#define ECHO_PIN     11
#define MAX_DISTANCE 60

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

double Setpoint, Input, Output;
double kp, ki, kd;
float distance, filter_distance, sum, servo_begin_angle;
unsigned long timer, timer2;

Servo myservo;
PID myPID(&Input, &Output, &Setpoint,kp,ki,kd, DIRECT);
float filter[] = {
//  141667814, 30535347, 33525554, 36521970, 39499313, 42412875, 
//  45222844, 47904896, 50434808, 52787330, 54946901, 56905861, 
//  58644063, 60115628, 61271939, 62115584, 62741988, 63086021, 
//  63086021, 62741988, 62115584, 61271939, 60115628, 58644063, 
//  56905861, 54946901, 52787330, 50434808, 47904896, 45222844, 
//  42412875, 39499313, 36521970, 33525554, 30535347, 141667814
-0.000039630965436958505,
  -0.00004724482686166332,
  -0.00007221605645506774,
  -0.00010255983540136766,
  -0.000137213739661849,
  -0.00017413495343555406,
  -0.00021003858791914445,
  -0.00024022680823908242,
  -0.00025839208768997765,
  -0.00025649764927881713,
  -0.00022472214429758973,
  -0.000151461385885916,
  -0.000023429946975866753,
  0.00017412071807948484,
  0.0004570899723381397,
  0.0008421217972498352,
  0.0013460846916723429,
  0.001985453416936253,
  0.00277563407176192,
  0.0037302452725153385,
  0.004860357197405943,
  0.006173735518555742,
  0.007674153632263778,
  0.009360784929955193,
  0.011227674573016506,
  0.01326337071775564,
  0.015450832261072038,
  0.01776721663742429,
  0.020184396402525163,
  0.022669073120707418,
  0.025183636600444145,
  0.027686986008638257,
  0.03013538509639834,
  0.032483786554667694,
  0.03468703727647556,
  0.03670116027370157,
  0.038484716245868696,
  0.040000110693941666,
  0.04121475494905446,
  0.04210209748110933,
  0.04264252062610118,
  0.04282400895088053,
  0.04264252062610118,
  0.04210209748110933,
  0.04121475494905446,
  0.040000110693941666,
  0.038484716245868696,
  0.03670116027370157,
  0.03468703727647556,
  0.032483786554667694,
  0.03013538509639834,
  0.027686986008638257,
  0.025183636600444145,
  0.022669073120707418,
  0.020184396402525163,
  0.01776721663742429,
  0.015450832261072038,
  0.01326337071775564,
  0.011227674573016506,
  0.009360784929955193,
  0.007674153632263778,
  0.006173735518555742,
  0.004860357197405943,
  0.0037302452725153385,
  0.00277563407176192,
  0.001985453416936253,
  0.0013460846916723429,
  0.0008421217972498352,
  0.0004570899723381397,
  0.00017412071807948484,
  -0.000023429946975866753,
  -0.000151461385885916,
  -0.00022472214429758973,
  -0.00025649764927881713,
  -0.00025839208768997765,
  -0.00024022680823908242,
  -0.00021003858791914445,
  -0.00017413495343555406,
  -0.000137213739661849,
  -0.00010255983540136766,
  -0.00007221605645506774,
  -0.00004724482686166332,
  -0.000039630965436958505
};
int filter_length = sizeof(filter)/sizeof(filter[0]);
float temp_data[sizeof(filter)/sizeof(filter[0])]; 


void setup(){ 
  Serial.begin(115200);
  myservo.attach(3);

  kp = 0.8;
  ki = 0.07;
  kd = 0.12;
  Setpoint = 20;

  servo_begin_angle = 75;
  myPID.SetTunings(kp, ki, kd);
  myPID.SetOutputLimits	(-13,13);
  myPID.SetMode(AUTOMATIC);

  for (int i =0; i<filter_length; i++)
    sum+=filter[i];

}

void loop(){
  //get ultrasound sensor input
  //if (millis() - timer >= 50) {
    distance = sonar.ping_cm();
    //timer = millis();
 // }

  //filter the input
  filter_distance = 0;
  for (int i = 1; i< filter_length; i++) 
    temp_data[i-1] = temp_data[i];
  temp_data[filter_length-1] = distance;
  for (int i = 0; i < filter_length; i++) 
    filter_distance += temp_data[i]*filter[i]/sum;

  //print the output
  Serial.print(distance);
  Serial.print(", ");
  Serial.println(filter_distance);

  //compute the PID controller
  if (millis() - timer2 >= 100){
    Input = filter_distance;
    myPID.Compute();
    myservo.write(servo_begin_angle+ Output);
    timer2 = millis();
  }

  //turn the PID controller if need
  if (Serial.available() > 0) {
    kp  = Serial.parseFloat(); 
    ki = Serial.parseFloat(); 
    kd = Serial.parseFloat();  
    myPID.SetTunings(kp, ki, kd);
  }
}





