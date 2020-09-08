#include <math.h>
#include <ESP8266WiFi.h>

#define pi 3.14
#define MAX_DIST 15
#define MAX_DISTC 20
#define MAX_PING 200

//Definition of RSSI variables
int med[12] = {0};
int raw[12] = {0};
int past=0;
int curr=0;

//Defining the ID and password of the beacon
#ifndef STASSID
#define STASSID "Akshta"
#define STAPSK  "sruj1197"
#endif
const char* ssid     = STASSID;
const char* password = STAPSK;

// Ultrasonic sensor
const int trigPin = 15;   //common trigger
const int echoL = 14;     //Left echo
const int echoR = 13;     //Right echo
const int echoC = 12;     //Center echo
long duration, distance, RightSensor,BackSensor,FrontSensor,LeftSensor;
int found = 0;

//motor A connected between A01 and A02
//Motor A
int PWMA = 4; //Speed control(Both pins given to the same PWM pin for speed control)
int AIN1 = 2; //Direction
int AIN2 = 5; //Direction

//motor B connected between B01 and B02
//Motor B
int BIN1 = 16; //Direction
int BIN2 = 0; //Direction

int speedA = 500; //speed for motor A
int speedB = 500; //speed for motor B

void setup() {
  Serial.begin(115200);
  
  //Motor driver
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  //Ultrasonic sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoR, INPUT);
  pinMode(echoL, INPUT);
  pinMode(echoC, INPUT);

   //RSSI
  WiFi.begin(ssid, password); //Connect to WIFI
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  //RSSI configuration
  for (int i=0; i<12; i++){
    for(int j=0; j<12;j++)
    {
      med[j] = WiFi.RSSI();  
    }
    raw[i] = mean(12,med);
    Serial.println(raw[i]);
    Left(20);
    Stop();
  }
  int dir = get_min_index(raw);
  Left(dir*30);
  Serial.println("degree");
  Serial.println(dir);
  Forward();
  delay(1000);
  
  for(int j=0; j<12;j++)
  {
    med[j] = WiFi.RSSI();  
  }
  curr = mean(12,med);
  Forward();
  delay(500);
}

void loop() {
    //Move the car to beacon
    if(abs(curr)>40){
    for(int j=0; j<12; j++)
    {
      med[j] = WiFi.RSSI();  
    }
    curr = mean(12,med);
    Serial.println(curr);
    if (abs(curr)>abs(past) && abs(curr-past)>=4){
      Left(90);
      Forward();
    }
    else{
      Forward();
    }
    past = curr;
  }
  else {
    Stop();
    found = 1;
  }

  if(found==0){
    //Ping the ultrasonic sensor for obstacle avoidance mechanism
    int r = MAX_DIST,l = MAX_DIST,c = MAX_DISTC; 
    SonarSensor(trigPin, echoR);
    RightSensor = distance;
    SonarSensor(trigPin, echoL);
    LeftSensor = distance;
    SonarSensor(trigPin, echoC);
    FrontSensor = distance;

    Serial.print(LeftSensor);
    Serial.print(" - ");
    Serial.print(FrontSensor);
    Serial.print(" - ");
    Serial.println(RightSensor);
  
    if(FrontSensor == 0) FrontSensor = MAX_PING;
    if(RightSensor == 0) RightSensor = MAX_PING;
    if(LeftSensor == 0) LeftSensor = MAX_PING;
  
    if(FrontSensor <= MAX_DISTC){
      Right(90);  
    }
    else if(LeftSensor <= MAX_DIST){
      Right(90);
    }
    else if(RightSensor <= MAX_DIST){
      Left(90);
    }
    else if(FrontSensor<=MAX_DISTC && LeftSensor<=MAX_DIST)
    {
      Serial.println("Turn right");
      Serial.println(FrontSensor);
      Serial.println(LeftSensor);
      
      Right(60);
    }
    else if(FrontSensor<=MAX_DISTC && RightSensor<=MAX_DIST)
    {
      Serial.println("Turn left");
      
      Left(60);
    }
    else
    {
      Serial.println("Forward");
      
      Forward();
    }
    delay(5);
  }
 
}


//Ultrasonic Sensor function definition for calculating the distance in order for the robot car to perform obstacle avoidance
void SonarSensor(int trigPin,int echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration/58.2; 
}


//Function for calculating the mean of RSSI values for moving the vehicle in the direction of RSSI towards the beacon
int mean(int n, int rssi[]){
  int mean;
  int sum = 0;

  for (int i=0; i< n; i++)
      sum += rssi[i];

  mean = sum/n;

  return mean;
}

int get_min_index(int array[]){
  int min_value = abs(array[0]) ; 
  int index = 0 ;                

  for (int i = 1; i<12; i++) 
  {
    if (abs(array[i]) < min_value) {
         index = i;
         min_value = abs(array[i]);
    }
  }
  return index;
}

//Function to stop the robot car
void Stop(){
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    
    analogWrite(PWMA, 255);
  
    //motor B
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
  
}

//Function to move the robot car forward
void Forward(){
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    
    analogWrite(PWMA, speedA);
  
    //motor B
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  
}

//Function to move the robot car backward
void Backward(){
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    
    analogWrite(PWMA, speedA);
  
    //motor B
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  
}

//Function to turn the robot car left
void Right(int angle){
    //motor A
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    
    analogWrite(PWMA, speedA);
  
    //motor B
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  
    delay((3600/360)*angle);
}

//Function to turn the robot car right
void Left(int angle){
    //motor A
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    
    analogWrite(PWMA, speedA);
  
    //motor B
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  
    delay((3600/360)*angle);
}
