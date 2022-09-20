#include <MatrixMath.h>
#include <math.h>
#include "Wire.h"
#include <MPU6050_light.h>





MPU6050 mpu(Wire);

float L1, L2, L3, R1, R2, R3;
  float avgL = 0;
  float avgR = 0;

  float angaz_o = 0; //original angle, before start
  float angaz = 0; //real time azimuth angle
  float angel = 0; //real time elevation angle
  float maxavg = 0; //maximum average detected in YAW
  float maxaz = 0; //azimuth angle at max avg
  float maxael = 0; //elevation angle at avgL == avgR
  float elrot = 0; //elevation rotation angle to perform
  
char junk;
String MissionPlan; 
String SatNumberS, SunAzimuthS, SunElevationS, StarAzimuthS, StarElevationS, MagFieldStrengthS, DeclinationS, InclinationS, Task1_XS, Task1_YS, Task1_ZS, a_1S, a_2S, b_1S, b_2S;
double SatNumber, SunAzimuth, SunElevation, StarAzimuth, StarElevation, MagFieldStrength, Declination, Inclination, Task1_X, Task1_Y, Task1_Z, a_latitude, a_longitude, b_latitude, b_longitude;
double kP = 5, kI = 0.005, kD = 0.5; //Constants for PID
bool rotationcomplete;
String TaskChoice;
double Ra1, Ra2, Y;
double avgLM, avgRM;

long timer;
unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double input, output;
double Integral, Derivative;
double initialerror;
double turntime = 80000;
double setpointX, setpointY, setpointZ;
double Zalignment;
double starttime;
double firstoutput;
int SatNumberAddress, SunAzimuthAddress, SunElevationAddress, StarAzimuthAddress, StarElevationAddress, MagFieldStrengthAddress, DeclinationAddress, InclinationAddress, Task1_XAddress;
int Task1_YAddress, Task1_ZAddress, a_1Address, a_2Address, b_1Address, b_2Address;
double pi = 3.14159;
int countdown;
double a_lat, a_long, b_lat, b_long, SunAz, SunEl, StarAz, StarEl;

int motorpinX1 = 10; 
int motorpinX2 = 4;
int motorpinZ1 = 8;
int motorpinZ2 = 9;
int motorpinY1 = 7;
int motorpinY2 = 2;
int LEDinput = 35;
int Laseroutput = 23;

int motorpinX_PWM = 3; 
int motorpinY_PWM = 5;
int motorpinZ_PWM = 6;

void setup() {

  Serial1.begin(9600);
  Wire.begin();

  pinMode(motorpinX1, OUTPUT);
  pinMode(motorpinX2, OUTPUT);
  pinMode(motorpinY1, OUTPUT);
  pinMode(motorpinY2, OUTPUT);
  pinMode(motorpinZ1, OUTPUT);
  pinMode(motorpinZ2, OUTPUT);
  pinMode(LEDinput, INPUT);
  pinMode(Laseroutput, OUTPUT);

  Calibration();

  delay(2000);
}

void loop() {

  while (millis() < 5000) {
    mpu.update();
  }

  Calibration();
  Bluetooth_input();
  ExtractMissionPlan();

  while (1 != 2) {
    ChooseTask();
    if (TaskChoice.indexOf("1") == 0) {  //Input 1 into serial monitor to execute task 1
      ExecuteTask1();
    }
    else if (TaskChoice.indexOf("2") == 0) {
      ExecuteTask2();
    }
    else if (TaskChoice.indexOf("3") == 0) {ExecuteTask3();} }
  
}

void Calibration() {
  byte status = mpu.begin();
  Serial1.print(F("MPU6050 status: "));
  Serial1.println(status);
  while (status != 0) { } // stop everything if could not connect to MPU6050

  Serial1.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true, true); // gyro and accelero
  Serial1.println("Done!\n");
}

void ChooseTask() {

  Serial1.print("Please choose a task: ");

  while (Serial1.available() == 0)
  {}      // clear the serial buffer

  TaskChoice = Serial1.readString(); //read the input

  Serial1.println("You chose task: ");
  Serial1.println(TaskChoice);
}

String Bluetooth_input() {

  Serial1.println("Bluetooth connection established");
  Serial1.print("Please input your mission plan: ");

  while (Serial1.available() == 0)
  {
    junk = Serial1.read() ;  // clear the serial buffer
  }

  MissionPlan = Serial1.readString(); //read the input
}

void ExtractMissionPlan() {

  SatNumberAddress = MissionPlan.indexOf("!address") - 3;
  SatNumberS = FindValue(SatNumberAddress);
  SatNumber = SatNumberS.toDouble();
  if (SatNumber == 2) {
    SunAzimuthAddress = MissionPlan.indexOf("1-13") + 5;
    SunAzimuthS = FindValue(SunAzimuthAddress);
    SunElevationAddress = MissionPlan.indexOf("1-13") + SunAzimuthS.length() + 7;
    SunElevationS = FindValue(SunElevationAddress);
    StarAzimuthAddress = MissionPlan.indexOf("!Sun") + 24;
    StarAzimuthS = FindValue(StarAzimuthAddress);
    StarElevationAddress = MissionPlan.indexOf("!Sun") + 26 + StarAzimuthS.length();
    StarElevationS = FindValue(StarElevationAddress);
    MagFieldStrengthAddress = MissionPlan.indexOf("!Star") + 25;
    MagFieldStrengthS = FindValue(MagFieldStrengthAddress);
    DeclinationAddress = MissionPlan.indexOf("!Star") + 26 + MagFieldStrengthS.length();
    DeclinationS = FindValue(DeclinationAddress);
    InclinationAddress = MissionPlan.indexOf("!Star") + 27 + DeclinationS.length() + MagFieldStrengthS.length();
    InclinationS = FindValue(InclinationAddress);
    Task1_XAddress = MissionPlan.indexOf("inclination") + 18;
    Task1_XS = FindValue(Task1_XAddress);
    Task1_YAddress = MissionPlan.indexOf("inclination") + 19 + Task1_XS.length();
    Task1_YS = FindValue(Task1_YAddress);
    Task1_ZAddress = MissionPlan.indexOf("inclination") + 20 + Task1_XS.length() + Task1_YS.length();
    Task1_ZS = FindValue(Task1_ZAddress);
    a_1Address = MissionPlan.indexOf("XYZ") + 5;
    a_1S = FindValue(a_1Address);
    a_2Address = MissionPlan.indexOf("XYZ") + 6 + a_1S.length();
    a_2S = FindValue(a_2Address);
    b_1Address = MissionPlan.indexOf("3a") + 14;
    b_1S = FindValue(b_1Address);
    b_2Address = MissionPlan.indexOf("3a") + 15 + b_1S.length();
    b_2S = FindValue(b_2Address);

    SunAzimuth = SunAzimuthS.toDouble();
    SunElevation = SunElevationS.toDouble();
    StarAzimuth = StarAzimuthS.toDouble();
    StarElevation = StarElevationS.toDouble();
    setpointX = Task1_XS.toDouble(); //Assigning setpoints from mission plan values
    setpointY = Task1_YS.toDouble();
    setpointZ = Task1_ZS.toDouble();
    a_latitude = a_1S.toDouble();
    a_longitude = a_2S.toDouble();
    b_latitude = b_1S.toDouble();
    b_longitude = b_2S.toDouble();
    a_lat = a_latitude * (pi / 180);
    a_long = a_longitude * (pi / 180);
    b_lat = b_latitude * (pi / 180);
    b_long = b_longitude * (pi / 180);

    SunAz = SunAzimuth * (pi / 180);
    SunEl = SunElevation * (pi / 180);
    StarAz = StarAzimuth * (pi / 180);
    StarEl = StarElevation * (pi / 180);

  }
  else {
    Serial1.print("Error, mission plan not accepted ");
  }

  Serial1.print("SatNumber: ");
  Serial1.println(SatNumber);
  Serial1.print("SunAzimuth: ");
  Serial1.println(SunAzimuth);
  Serial1.print("SunElevation: ");
  Serial1.println(SunElevation);
  Serial1.print("StarAzimuth: ");
  Serial1.println(StarAzimuth);
  Serial1.print("StarElevation: ");
  Serial1.println(StarElevation);
  Serial1.print("MagFieldStrengthAddress: ");
  Serial1.println(MagFieldStrength);
  Serial1.print("Declination: ");
  Serial1.println(Declination);
  Serial1.print("Inclination: ");
  Serial1.println(Inclination);
  Serial1.print("Task1_X: ");
  Serial1.println(setpointX);
  Serial1.print("Task1_Y: ");
  Serial1.println(setpointY);
  Serial1.print("Task1_Z: ");
  Serial1.println(setpointZ);
  Serial1.print("a_longitude: ");
  Serial1.println(a_longitude);
  Serial1.print("a_latitude: ");
  Serial1.println(a_latitude);
  Serial1.print("b_longitude: ");
  Serial1.println(b_longitude);
  Serial1.print("b_latitude: ");
  Serial1.println(b_latitude);
}

String FindValue(int Address) {
  int L = 0;
  int pointer = Address;
  String StringAtAddress = "";

  while ((String)MissionPlan.charAt(pointer) != (String)" ") {
    pointer += 1;
  }
  L = pointer - Address;

  int EndAddress = pointer;
  pointer = Address;
  while (pointer < Address + L) {
    StringAtAddress += MissionPlan.charAt(pointer);
    pointer += 1;
  }
  return StringAtAddress;
}

double myPID(double inp, double currentvelocity, int point) {
  currentTime = millis();                //get current time
  elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation

  error = point - inp;                                // determine error

  Integral += error * elapsedTime / 1000;             // compute integral
  Derivative = currentvelocity;                       //Angular velocity using mpu.getGyro...   

  double out = kP * error + kI * Integral + kD * Derivative;        //  PID output

  previousTime = currentTime; //remember current time

  if (out > 255) {
    out = 255;
  }
  else if (out<(-255)) {
    out = -255;                  
  }
  return out;
}

void Xrotation(int setpoint) { //X
  input = (Xpos());              //read from IMU gyroscope
  output = myPID(input, mpu.getGyroX(), setpoint);
  delay(100);
  if (error>0){                                           
  if (output < 0) {                                       //for motor rotation direction
    digitalWrite(motorpinX1, HIGH);                       //Uses reaction wheel X
    digitalWrite(motorpinX2, LOW);
    analogWrite(motorpinX_PWM, -1 * output);              //analogWrite takes positive number from 0-255
  }        //control the motor based on PID value
  if (output > 0) {
    digitalWrite(motorpinX1, LOW);
    digitalWrite(motorpinX2, HIGH);
    analogWrite(motorpinX_PWM, output);
  }
}
else if (error < 0) {                                     //to account for a negative error in PID function
    if (output < 0) {
      digitalWrite(motorpinX1, HIGH);
      digitalWrite(motorpinX2, LOW);
      analogWrite(motorpinX_PWM, -1 * output);
    }        //control the motor based on PID value
    if (output > 0) {
      digitalWrite(motorpinX1, LOW);
      digitalWrite(motorpinX2, HIGH);
      analogWrite(motorpinX_PWM, output);
    }
  }

}

void Yrotation(int setpoint) { //Y
  input = (Ypos());                //read from IMU gyroscope
  output = myPID(input, mpu.getGyroY(), setpoint);
  delay(100);
  if (error > 0) {
    if (output < 0) {
      digitalWrite(motorpinY1, HIGH);
      digitalWrite(motorpinY2, LOW);
      analogWrite(motorpinY_PWM, -1 * output);
    }        //control the motor based on PID value
    if (output > 0) {
      digitalWrite(motorpinY1, LOW);
      digitalWrite(motorpinY2, HIGH);
      analogWrite(motorpinY_PWM, output);
    }
  }
  else if (error < 0) {
    if (output < 0) {
      digitalWrite(motorpinY1, HIGH);
      digitalWrite(motorpinY2, LOW);
      analogWrite(motorpinY_PWM, -1 * output);
    }        //control the motor based on PID value
    if (output > 0) {
      digitalWrite(motorpinY1, LOW);
      digitalWrite(motorpinY2, HIGH);
      analogWrite(motorpinY_PWM, output);
    }
  }
  Serial1.print("Ypos: ");
  Serial1.println(Ypos());
}

void YrotationTask3(int setpoint) { //Y
  input = (Ypos());                //read from IMU gyroscope
  output = myPID(-1 * input, mpu.getGyroY(), setpoint);
  delay(100);
  if (error > 0) {
    if (output < 0) {
      digitalWrite(motorpinY1, LOW);
      digitalWrite(motorpinY2, HIGH);
      analogWrite(motorpinY_PWM, -1 * output);
    }        //control the motor based on PID value
    if (output > 0) {
      digitalWrite(motorpinY1, HIGH);
      digitalWrite(motorpinY2, LOW);
      analogWrite(motorpinY_PWM, output);
    }
  }
  else if (error < 0) {
    if (output < 0) {
      digitalWrite(motorpinY1, LOW);
      digitalWrite(motorpinY2, HIGH);
      analogWrite(motorpinY_PWM, -1 * output);
    }        //control the motor based on PID value
    if (output > 0) {
      digitalWrite(motorpinY1, HIGH);
      digitalWrite(motorpinY2, LOW);
      analogWrite(motorpinY_PWM, output);
    }
  }
}


void FullYrotation(double setpoint) {                             //This uses Yrotation but defines when the rotation will stop
  Serial1.println("Executing Y rotation ");

  starttime = millis();
  int i = 0;
  rotationcomplete = false;
  double countdown2 = 0;
  delay(100);
  Integral = 0;

  while ((millis() < (starttime + turntime)) and (rotationcomplete == false)) {
    if (error < 4 and error > -4) {
      if (i == 0) {
        countdown2 = millis();                                //Start countdown if within error range
      }
      else if (countdown2 < (millis() - 300)) {               //Rotation is complete if within error range for 300ms                        
        rotationcomplete = true;
      }
      i++;
    }
    else {
      i = 0;
    }
    Yrotation(setpoint); //sweep
    Serial1.print("error: ");
    Serial1.println(error);
  }

  digitalWrite(motorpinY1, LOW);
  digitalWrite(motorpinY2, LOW);

  Serial1.print("Y rotation complete: ");
  Serial1.println(setpoint);
}

void FullYrotationTask3(double setpoint) {
  Serial1.println("Executing Y rotation ");

  starttime = millis();
  int i = 0;
  rotationcomplete = false;
  double countdown2 = 0;
  delay(100);
  Integral = 0;

  while ((millis() < (starttime + turntime)) and (rotationcomplete == false)) {
    if (error < 4 and error > -4) {
      if (i == 0) {
        countdown2 = millis();
      }
      else if (countdown2 < (millis() - 300)) {
        rotationcomplete = true;
      }
      i++;
    }
    else {
      i = 0;
    }
    YrotationTask3(setpoint); //sweep
    Serial1.print("error: ");
    Serial1.println(error);
  }

  digitalWrite(motorpinY1, LOW);
  digitalWrite(motorpinY2, LOW);

  Serial1.print("Y rotation complete: ");
  Serial1.println(setpoint);
}

void ExecuteTask1 () {                                  //Rotations about each axis

  FullYrotation(setpointY + Ypos());
  delay(1000);                                            
  FullXrotation(Xpos()+90);                             //Rotate 90deg to move Z reaction wheel to bottom
  FullZrotation(setpointZ + Zpos());
  FullXrotation(setpointX + Xpos());
  Serial.println("Task 1 completed.");
}

void ExecuteTask2() {

  double countdown1 = 0;

  Serial1.println("Executing Task 2. ");

  starttime = millis();
  int i = 0;
  rotationcomplete = false;
  double yposition = Ypos() + 360;
  angaz_o = analogRead(Ypos());
  delay(100);
  Integral = 0; //reset integral term to 0

  while ((millis() < (starttime + turntime)) and (rotationcomplete == false)) {
    if (error < 4 and error > -4) {
      if (i == 0) {
        countdown1 = millis();
      }
      else if (countdown1 < (millis() - 300)) {
        rotationcomplete = true;
      }
      i++;
    }
    else {
      i = 0;
    }
    Yrotation(yposition); //360 deg sweep
    Serial1.print("error: ");
    Serial1.println(error);

    angaz = Ypos();//angle to azimuth
    if (angaz <= (angaz_o + 180)) {
      L1 = analogRead(A3);                    //Pins for light sensors
      L1 = 0.1 * L1;
      L2 = analogRead(A8);
      L3 = analogRead(11);
      R1 = analogRead(A0);
      R2 = analogRead(A6);
      R3 = analogRead(A14);
      angaz = Ypos();//gyrYaw);
      avgL = favgL (L1, L2, L3);
      avgR = favgR (R1, R2, R3);
      if (avgR > maxavg) {
        maxavg = avgR;                  
        maxaz = angaz;
      }
      if (avgL > maxavg) {
        maxavg = avgL;
        maxaz = angaz; //position of max average of light from photodiodes
      }
    }
    angaz = Ypos();   
  }

  digitalWrite(motorpinY1, LOW);
  digitalWrite(motorpinY2, LOW);

  Serial1.println("Sweep completed ");
  delay(1000);

  angaz = Ypos();
  
  Serial1.print("maxaz: ");
  Serial1.println(maxaz);
  Serial1.print("angaz: ");
  Serial1.println(angaz);

  FullYrotation(maxaz); //rotate to maximum light

  Serial.println("maxaz: ");
  Serial.println(maxaz);
  Serial.println("Task 2 completed.");
}

void ExecuteTask3() {

  FullYrotation(maxaz);                                                
  transformations(a_latitude, a_longitude, SunAzimuth, SunElevation);
  FullXrotation(R1);
  FullYrotationTask3(Y+Ypos()+13);
  FullXrotation(R2);
  Laser(); //arm laser
  delay(10000);
  digitalWrite(Laseroutput,LOW);
  
  ExecuteTask2(); //To find the sun again in order to recalibrate the position
  transformations(b_latitude, b_longitude, SunAzimuth, SunElevation);
  FullXrotation(R1);
  FullYrotationTask3(Y+Ypos()+7);
  );
  FullXrotation(R2);
  Laser();
  delay(10000);
  digitalWrite(Laseroutput,LOW);
  
  Serial.println("Task 3 completed.");
}

void Laser() {
  if (digitalRead(LEDinput) == HIGH) {
    digitalWrite(Laseroutput, HIGH);
  }
}
void ReachZeroPosition() {            //Uses accelerometer and balancing of vectors to find a zero position and reset to it(wasn't used in final test)
  double AccX = mpu.getAccX();
  double AccY = mpu.getAccY();
  double G[] = {AccX, AccY};
  double g[] = {1, 0};
  Zalignment = arccos(g, G);
  starttime = millis();
  int i = 0;
  rotationcomplete = false;
  delay(3100);

  while ((millis() < (starttime + turntime)) and (rotationcomplete == false)) {
    if (error < 5 and error > -5) {
      if (i == 0) {
        countdown = millis();
      }
      else if (countdown < (millis() - 3000)) {
        rotationcomplete = true;
      }
      i++;
    }
    else {
      i = 0;
    }
    Xrotation(Zalignment);
    Serial1.print("Error: ");
    Serial1.println(error);
  }
  Serial1.println("Reach Zero Position Completed");
}

double transformations(double latitude, double longitude, double Az, double El) { //Conversion of coordinates and angle transformation maths
  double x, y;
  double A1 = 1.340264;
  double A2 = -0.081106;
  double A3 = 0.000893;
  double A4 = 0.003796;
  double Ra = 2191 / (2 * pi); //Cylinder radius of radius = equator length of map
  double beta = latitude; //authalic latitude
  double theta = asin(sqrt(3) / 2 * sin(beta));
  x = (Ra * 2 * sqrt(3) * longitude * cos(theta)) / (3 * (A1 + 3 * A2 * pow(theta, 2) + (7 * A3 + 9 * A4 * pow(theta, 2)) * pow(theta, 6)));
  y = Ra * theta * (A1 + A2 * pow(theta, 2) + pow(theta, 6) * (A3 + A4 * pow(theta, 2)));
  x = (x * 1095.5 / 943.8248283878921) / 1000;
  y = (y * 533 / 459.375565768491) / 1000; //1.16 correction factor

  double lightvector[] = {(sin(Az - (pi / 2))*cos(El)) * 4, ((cos(Az - pi / 2))*cos(El)) * 4, sin(El) * 4};
  double O[] = {0, 0, 0};
  double P[] = {3, -x, y};
  double pmap[] = {P[0] - O[0], P[1] - O[1], P[2] - O[2]};
  double yaxis[] = {0, 0, 4};

  Ra1 = arccos(lightvector, yaxis);
  double lightvector_xy[] = {lightvector[0], lightvector[1], 0};
  double pmap_xy[] = {pmap[0], pmap[1], 0};
  Y = arccos(lightvector_xy, pmap_xy);
  double pmap_xz[] = {pmap[0], pmap[2], 0};
  double yaxis_xz[] = {yaxis[0], yaxis[2], 0};
  Ra2 = arccos(pmap_xz, yaxis_xz);
  if (Az > 0) {
    Y = -1 * Y;
  }
  if (avgLM > avgRM) {
    R1 = -1 * R1;
  }
  if (avgLM < avgRM) {
    R2 = -1 * R2;
  }

}

void print_XYZ() {
  Serial1.print("X, Y, Z:");
  Serial1.print(Xpos(), 2);
  //Serial1.print(", ");
  //Serial1.print(Ypos(), 2);
  //Serial1.print(", ");
  //Serial1.println(Zpos(), 2);
}

double Zpos() {
  mpu.update();
  return -1*mpu.getAngleY();
}

double Ypos() {
  mpu.update();
  return mpu.getAngleZ();
}


double Xpos() {
  mpu.update();
  if (millis() - timer > 100) { // print data every second
    return mpu.getAngleX();
    timer = millis();
  }
}

double dot_product(double Vector1[], double Vector2[]) {
  return (Vector1[0] * Vector2[0] + Vector1[1] * Vector2[1] + Vector1[2] * Vector2[2]);
}
double normalise(double Vector[]) {
  return sqrt((pow(Vector[0], 2)) + (pow(Vector[1], 2)) + (pow(Vector[2], 2)));
}
double arccos(double Vector1[], double Vector2[]) {
  return ((180 / pi) * acosf(dot_product(Vector1, Vector2) / ((normalise(Vector1) * normalise(Vector2)))));
}

double favgL (float L1, float L2, float L3) {
  return (L1 + L2 + L3) / 3;
}
double favgR (float R1, float R2, float R3) {
  return (R1 + R2 + R3) / 3;
}
