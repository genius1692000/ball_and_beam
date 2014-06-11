import processing.serial.*;
Serial myPort;
float x, kp, ki, kd, Setpoint, distance, filtered_distance, sum;


void setup()
{
  size (800, 600);
  //background(175);
  strokeWeight(3);
  textAlign(CENTER, CENTER);
  refresh();
  println(Serial.list());
  myPort = new Serial(this, "/dev/cu.usbmodem1421", 115200);
  myPort.bufferUntil('\n');

  kp = 0.8;
  ki = 0.07;
  kd = 0.12;
  Setpoint = 20;
}

void draw()
{
}

void refresh() {
  x = 0;
  noStroke();
  fill(255);
  rect(0, 0, width, height);
  stroke(100);
  strokeWeight(1);
  fill(0);
  textSize(12);
  for (int i = 0; i <=3; i++) {
    line (0, height/5 * (i+1), width, height/5 * (i+1));
    text(str((4-i)*10), 10, height/5 * (i+1));
  }

  stroke(100, 255, 100);
  line (0, height-Setpoint/50*(height), width, height-Setpoint/50*(height));
}

void serialEvent(Serial myPort) {
  float last_filtered_distance = filtered_distance;
  float last_distance = distance;
  //  filtered_distance = 0;
  String myString = myPort.readStringUntil('\n');

  if (myString != null) {
    myString = trim(myString);
    float data[] = float(split(myString, ","));
   // distance = data[0];
    filtered_distance = data[0];

    println(distance + "    " + filtered_distance);
    strokeWeight(3);
    stroke(255, 0, 0);
    line(x-1, height - last_filtered_distance /50 *(height), x, height - filtered_distance /50 *(height));

    strokeWeight(1);
    stroke(0, 0, 255);
    line(x-1, height - last_distance /50 *(height), x, height - distance /50 *(height));

    x++;
    if (x > width) refresh();
  }
  String display = "kp = " + kp + "     ki = " + ki + "     kd = " + kd;
  fill(175);
  noStroke();
  rect(0, height-28, width, height);
  fill(0);
  textSize(12);
  text(display, width/2, height-14);
}

void keyPressed() {
  if (key == 'a' || key == 'A')       kp+=0.01;
  else if (key == 'z' || key == 'Z')  kp-=0.01;
  else if (key == 's' || key == 'S')  ki+=0.01;
  else if (key == 'x' || key == 'X')  ki-=0.01;
  else if (key == 'd' || key == 'D')  kd+=0.01;
  else if (key == 'c' || key == 'C')  kd-=0.01;
  String out = kp + ", " + ki + ", " + kd;
  myPort.write(out);
}

