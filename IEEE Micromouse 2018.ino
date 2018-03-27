// NYIT IEEE MICROMOUSE COMPETITION - SPRING 2018
// David Stachnik

//Global Variables and Defines

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Sensors
// Sharp IR GP2Y0A41SK0F (4-30cm, analog)
#define sensor1 A0 //Front 
#define sensor2 A1 // Left
#define sensor3 A2 // Right

// Motors 
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_StepperMotor *myMotorRight = AFMS.getStepper(1, 1);
Adafruit_StepperMotor *myMotorLeft = AFMS.getStepper(1, 2);


//Maze layout has 100 values around the perimeter to prevent an out of bounds error when reading cell values around the bot, 100 should be high enough for the bot to never go there.
int maze[8][8] = {{100,100,100,100,100,100,100,100},{100,4,3,2,2,3,4,100},{100,3,2,1,1,2,3,100},{100,2,1,0,0,1,2,100},{100,2,1,0,0,1,2,100},{100,3,2,1,1,2,3,100},{100,4,3,2,2,3,4,100},{100,100,100,100,100,100,100,100}};


//Stack
int stack[100] = {24,12,412,312,31,231,241,24,124,124,124,124,124,3,524,53,57,568,56,5634,5,346,457,456,34,534,635,6,345,235,34,634,524,5,12};
int top;

//Bot starting direction and position
String direction = "north";
int PosX = 1;
int PosY = 1;

float volts1;
float volts2;
float volts3;

float distance1;
float distance2;
float distance3;

short analog_write_range = 255;
byte buzzer = 3;
float power = 1;
short beat = 300;
short offset = 2;

//--------------------------------------------------------------------------------
// SETUP CODE | RUNS ONCE AT START | NOT GLOBAL

void setup() {

  //Wait so that we can place the bot down	
  delay(3000);
  
  Serial.begin(9600); // Start the serial port
  Serial.println("***********RUNNING SETUP*************");
  AFMS.begin();
  myMotorLeft->setSpeed(300);  // 10 rpm
  myMotorRight->setSpeed(300); 

  //Print of maze values
  for(int i=0; i<8; i++){
    for(int e=0; e<8; e++){
      Serial.print(maze[i][e]);
      Serial.print(" ");
    }
    Serial.println();
  }

  for(int i=0; i<16; i++){
    for(int e=0; e<16; e++){
    
      Serial.print(" ");
    }
    Serial.println();
  
  }
}


//--------------------------------------------------------------------------------
// CUSTOM FUNCTIONS 

/*
* Notes: Double
* MOTORS ARE BACKWARDS! Dam hardware team...
*/
//Buzzer
void ringBuzzer (byte id, float freq, short dur, float power) {
  if (freq > 0) {
    long value = round (500000 / freq);
    long cycles = round (freq * dur / 1000);
    long strength = power * analog_write_range;
    for (long i=0; i < cycles; i++) {
      analogWrite (id, strength);
      delayMicroseconds (value);
      analogWrite (id, 0);
      delayMicroseconds(value);
    }
  }
  else {
    delay (dur);
  }
}
/**
 * Gets frequency of given note, 0 being A0 (440 Hz).
 */
float getFreq (short note) {
  return 440 * pow (2, note / 12.0);
}
// Go Forward
void Forward()
{
  Serial.println("Going Forward");
  for(float i=0; i<250; i++)
  {
    myMotorRight->step(1, BACKWARD, DOUBLE); 
    myMotorLeft->step(1, BACKWARD, DOUBLE);

    volts2 = analogRead(sensor2)*0.0048828125; // value from sensor * (5/1024)
    distance2 = 39.4527*pow(0.0614007,volts2); // worked out from datasheet graph
    distance2 = distance2 + 4.45;

    volts3 = analogRead(sensor3)*0.0048828125; // value from sensor * (5/1024)
    distance3 = 39.4527*pow(0.0614007,volts3); // worked out from datasheet graph
    distance3 = distance3 + 4.45;


    if(distance2 <= 5){
      myMotorLeft->step(1, FORWARD, DOUBLE);
      i = i - 0.5;
    }
    if(distance3 <= 5){
      myMotorRight->step(1, FORWARD, DOUBLE);
      i = i - 0.5;
    }

  }    
  if(direction == "north") PosY = PosY + 1;
  else if(direction == "east") PosX = PosX + 1;
  else if(direction == "south") PosY = PosY - 1;
  else if(direction == "west") PosX = PosX - 1;
  else Serial.println("UPDATE POSITION ERROR!!");
}

// Turn Left
void LeftTurn()
{
  Serial.println("Making Left Turn");
  // Left Turn
  for(int i=0; i<87; i++)
  {
    myMotorLeft->step(1,BACKWARD , DOUBLE); 
    myMotorRight->step(1,FORWARD , DOUBLE);
  }
  if(direction == "east") direction = "north";
  else if(direction == "north") direction = "west";
  else if(direction == "west") direction = "south";
  else if(direction == "south") direction = "east";
  else Serial.println("DIRECTION ERROR!!");
}

// Turn Right
void RightTurn()
{
  Serial.println("Making Right Turn");
  for(int i=0; i<87; i++)
  {
    myMotorLeft->step(1,FORWARD , DOUBLE);
    myMotorRight->step(1,BACKWARD , DOUBLE); 
  }
  if(direction == "east") direction = "south";
  else if(direction == "north") direction = "east";
  else if(direction == "west") direction = "north";
  else if(direction == "south") direction = "west";
  else Serial.println("DIRECTION ERROR!");
}

// U Turn
void UTurn()
{
  Serial.println("Making U Turn");
  for(int i=0; i<174; i++)
  {
    myMotorLeft->step(1, FORWARD, DOUBLE);
    myMotorRight->step(1, BACKWARD, DOUBLE); 
  }
  if(direction == "east") direction = "west";
  else if(direction == "north") direction = "south";
  else if(direction == "west") direction = "east";
  else if(direction == "south") direction = "north";
  else Serial.println("DIRECTION ERROR!");
}

//Correct Front
void correctFront(){
  Serial.println("Correcting Front");
  if(distance1<5.5){
    while(distance1<5.5){
      myMotorLeft->step(1, FORWARD, DOUBLE);
      myMotorRight->step(1, FORWARD, DOUBLE);
      volts1 = analogRead(sensor1)*0.0048828125; // value from sensor * (5/1024)
      distance1 = 39.4527*pow(0.0614007,volts1); // worked out from datasheet graph
      distance1 = distance1 + 4.45;    
    }
  }
  else if(distance1>5.5){
    while(distance1>5.5){
      myMotorLeft->step(1, BACKWARD, DOUBLE);
      myMotorRight->step(1, BACKWARD, DOUBLE);
      volts1 = analogRead(sensor1)*0.0048828125; // value from sensor * (5/1024)
      distance1 = 39.4527*pow(0.0614007,volts1); // worked out from datasheet graph
      distance1 = distance1 + 4.45;
    }    
  }
}


//--------------------------------------------------------------------------------
// LOOP | RUNS OVER AND OVER AGAIN

void loop() {
  
  //Prints current status and info
  Serial.println("-----Current Status-----");
  Serial.print("Bot Direction: ");
  Serial.println(direction);
  Serial.print("X,Y Bot Position: ");
  Serial.print(PosX);
  Serial.print(" ");
  Serial.println(PosY);
  Serial.print("Bot Pos Value: ");
  Serial.println(maze[PosX][PosY]);
  Serial.println("------------------------");
  
  //Checks if the bot is in the center, does a 360 and stops
  if(maze[PosX][PosY] == 0){
    Serial.println("*****CENTER REACHED*****");

    playMusic();
 
    UTurn();
    UTurn();
	
    //Endless Loop to Stop Bot
    Serial.println("*****ALL STOP*****");
    while(1);
    }     
  
  //Converts sensor output into cm. Program wise this is when the sensors get their data.
  volts1 = analogRead(sensor1)*0.0048828125;  // value from sensor * (5/1024)
  distance1 = 39.4527*pow(0.0614007,volts1); // worked out from datasheet graph
  distance1 = distance1 + 4.45;
  volts2 = analogRead(sensor2)*0.0048828125;  // value from sensor * (5/1024)
  distance2 = 39.4527*pow(0.0614007,volts2); // worked out from datasheet graph
  distance2 = distance2 + 4.45;
  volts3 = analogRead(sensor3)*0.0048828125; // value from sensor * (5/1024)
  distance3 = 39.4527*pow(0.0614007,volts3); // worked out from datasheet graph
  distance3 = distance3 + 4.45;
  
  // Prints Sensor Distances
  Serial.println("-----Distance values-----");
  Serial.print("Distance1: ");
  Serial.print(distance1);
  Serial.print(" Distance2: ");
  Serial.print(distance2);
  Serial.print(" Distance3: ");
  Serial.println(distance3);
  Serial.println("-------------------------");
    

// Navigation Code


//The values of the cells around the bot
int valueNorth;
int valueEast;
int valueSouth;
int valueWest;

//Different situations depending on which direction the bot is facing. 
//If there is a wall blocking, uses a temp value of 100 to avoid going there, the value to it's rear assumes no wall because it came from there.  
if(direction == "north"){
  if(distance1 >= 12) valueNorth = maze[PosX][PosY+1];
  else valueNorth = 100;
  if(distance2 >= 12) valueWest = maze[PosX-1][PosY];
  else valueWest = 100;
  if(distance3 >= 12) valueEast = maze[PosX+1][PosY];
  else valueEast = 100;
  valueSouth = maze[PosX][PosY-1]+1;
}

if(direction == "east"){
  if(distance1 >= 12) valueEast = maze[PosX+1][PosY];
  else valueEast = 100;
  if(distance2 >= 12) valueNorth = maze[PosX][PosY+1];
  else valueNorth = 100;
  if(distance3 >= 12) valueSouth = maze[PosX][PosY-1];
  else valueSouth = 100;
  valueWest = maze[PosX-1][PosY]+1;
}

if(direction == "south"){
  if(distance1 >= 12) valueSouth = maze[PosX][PosY-1];
  else valueSouth = 100;
  if(distance2 >= 12) valueEast = maze[PosX+1][PosY];
  else valueEast = 100;
  if(distance3 >= 12) valueWest = maze[PosX-1][PosY];
  else valueWest = 100;
  valueNorth = maze[PosX][PosY+1]+1;  
}

if(direction == "west"){
  if(distance1 >= 12) valueWest = maze[PosX-1][PosY];
  else valueWest = 100;
  if(distance2 >= 12) valueSouth = maze[PosX][PosY-1];
  else valueSouth = 100;
  if(distance3 >= 12) valueNorth = maze[PosX][PosY+1];
  else valueNorth = 100;
  valueEast = maze[PosX+1][PosY]+1;
}

// Print Surrounding Values
Serial.println("-----Surrounding Values-----");
Serial.print("Value North: ");
Serial.println(valueNorth);
Serial.print("Value East: ");
Serial.println(valueEast);
Serial.print("Value South: ");
Serial.println(valueSouth);
Serial.print("Value West: ");
Serial.println(valueWest);
Serial.println("---------------------------");

//Takes the value of the cell the bot is in and stores it in currentValue
int currentValue = maze[PosX][PosY];

//Stores none into desired direction, this will either be changed if it finds an open direction with a lower value or stay as none if the current cell needs to update.
String desDirection = "none";

//Compares the currentValue against the values around it, this logic needs to be checked for issues because it's definitely not perfect.

if(valueEast < currentValue){
  currentValue = valueEast;
  desDirection = "east";
}
if(valueNorth < currentValue){
  currentValue = valueNorth;
  desDirection = "north";
}
if(valueSouth < currentValue){
  currentValue = valueSouth;
  desDirection = "south";
}
if(valueWest < currentValue){
  currentValue = valueWest;
  desDirection = "west";
}
if(distance1 <= 5.5 && distance2 <= 5.5 && distance3 <= 5.5)
{
  currentValue = 100;
}

Serial.print("Current Direction: ");
Serial.println(direction);
Serial.print("Desired Direction: ");
Serial.println(desDirection);

//If the desired direction stays none, meaning it didn't find a good open value around the bot, the current cell will add one to its value
if(desDirection == "none"){
  maze[PosX][PosY] = maze[PosX][PosY]+1;
}
else{
  
  //Bot Correction
  if(distance1 < 5.5) correctFront();
  if(distance1 > 5.5 && distance1 < 20) correctFront();

  while(direction != desDirection){
	  if(direction == "north" && desDirection == "east") RightTurn();
	  else if(direction == "east" && desDirection == "south") RightTurn();
	  else if(direction == "south" && desDirection == "west") RightTurn();
	  else if(direction == "west" && desDirection == "north") RightTurn();
	  else LeftTurn();
  }
  Forward();
}


  // LOOP END
}


void playMusic () {
  
  ringBuzzer (buzzer, getFreq(-2 + offset), beat * 1, power);
  
  ringBuzzer (buzzer, getFreq(-2 + offset), beat * 1, power);
  ringBuzzer (buzzer, getFreq(-2 + offset), beat * 0.5, power);
  ringBuzzer (buzzer, getFreq(-2 + offset), beat * 0.5, power);
  ringBuzzer (buzzer, getFreq(-2 + offset), beat * 1, power);
  ringBuzzer (buzzer, getFreq(-2 + offset), beat * 0.5, power);
  ringBuzzer (buzzer, getFreq(-2 + offset), beat * 0.5, power);
  ringBuzzer (buzzer, getFreq(2 + offset), beat * 1, power);
  ringBuzzer (buzzer, getFreq(-2 + offset), beat * 0.5, power);
  ringBuzzer (buzzer, getFreq(-2 + offset), beat * 0.5, power);
  ringBuzzer (buzzer, getFreq(-2 + offset), beat * 0, power);
  ringBuzzer (buzzer, getFreq(2 + offset), beat * 0, power);

  ringBuzzer (buzzer, getFreq(2 + offset), beat * 1, power);
  ringBuzzer (buzzer, getFreq(2 + offset), beat * 0.5, power);
  ringBuzzer (buzzer, getFreq(2 + offset), beat * 0.5, power);
  ringBuzzer (buzzer, getFreq(2 + offset), beat * 1, power);
  ringBuzzer (buzzer, getFreq(2 + offset), beat * 0.5, power);
  ringBuzzer (buzzer, getFreq(2 + offset), beat * 0.5, power);
  ringBuzzer (buzzer, getFreq(5 + offset), beat * 1, power);
  ringBuzzer (buzzer, getFreq(2 + offset), beat * 0.5, power);
  ringBuzzer (buzzer, getFreq(2 + offset), beat * 0.5, power);
  ringBuzzer (buzzer, getFreq(2 + offset), beat * 2, power);

  ringBuzzer (buzzer, getFreq(2 + offset), beat * 1, power);
  ringBuzzer (buzzer, getFreq(2 + offset), beat * 0.5, power);
  ringBuzzer (buzzer, getFreq(2 + offset), beat * 0.5, power);
  ringBuzzer (buzzer, getFreq(2 + offset), beat * 1, power);
  ringBuzzer (buzzer, getFreq(2 + offset), beat * 0.5, power);
  ringBuzzer (buzzer, getFreq(2 + offset), beat * 1, power);
  ringBuzzer (buzzer, getFreq(2 + offset), beat * 0.5, power);
  ringBuzzer (buzzer, getFreq(2 + offset), beat * 2, power);
  ringBuzzer (buzzer, getFreq(2 + offset), beat * 0.5, power);
  ringBuzzer (buzzer, getFreq(2 + offset), beat * 0.5, power);
  
  ringBuzzer (buzzer, getFreq(2 + offset), beat * 1, power);
  ringBuzzer (buzzer, getFreq(2 + offset), beat * 0.5, power);
  ringBuzzer (buzzer, getFreq(2 + offset), beat * 0.5, power);
  ringBuzzer (buzzer, getFreq(2 + offset), beat * 0.5, power);
  ringBuzzer (buzzer, getFreq(2 + offset), beat * 1, power);
  ringBuzzer (buzzer, getFreq(2 + offset), beat * 1.5, power);
  for (byte i = 0; i < 2; i++) {
  ringBuzzer (buzzer, getFreq(2 + offset), beat * 0.5, power);
  ringBuzzer (buzzer, getFreq(5 + offset), beat * 1.5, power);

  ringBuzzer (buzzer, getFreq(7 + offset), beat * 6, power);
  ringBuzzer (buzzer, getFreq(7 + offset), beat * 0.5, power);
  ringBuzzer (buzzer, getFreq(9 + offset), beat * 1.5, power);
  
  ringBuzzer (buzzer, getFreq(2 + offset), beat * 6, power);
  ringBuzzer (buzzer, getFreq(2 + offset), beat * 0.5, power);
  ringBuzzer (buzzer, getFreq(5 + offset), beat * 1.5, power);

  ringBuzzer (buzzer, getFreq(7 + offset), beat * 6, power);
  ringBuzzer (buzzer, getFreq(7 + offset), beat * 0.5, power);
  ringBuzzer (buzzer, getFreq(9 + offset), beat * 1.5, power);
  
  ringBuzzer (buzzer, getFreq(2 + offset), beat * 8, power);

  ringBuzzer (buzzer, 0, beat * 1, power);
  ringBuzzer (buzzer, getFreq(0 + offset), beat * 0.5, power);
  ringBuzzer (buzzer, getFreq(-2 + offset), beat * 0.5, power);
  ringBuzzer (buzzer, getFreq(0 + offset), beat * 1.5, power);
  ringBuzzer (buzzer, getFreq(-2 + offset), beat * 0.5, power);
  ringBuzzer (buzzer, getFreq(0 + offset), beat * 1, power);
  ringBuzzer (buzzer, getFreq(2 + offset), beat * 3, power);
  
  ringBuzzer (buzzer, 0, beat * 1, power);
  ringBuzzer (buzzer, getFreq(-2 + offset), beat * 0.5, power);
  ringBuzzer (buzzer, getFreq(-2 + offset), beat * 0.5, power);
  ringBuzzer (buzzer, getFreq(-2 + offset), beat * 1, power);
  ringBuzzer (buzzer, getFreq(-2 + offset), beat * 0.5, power);
  ringBuzzer (buzzer, getFreq(-2 + offset), beat * 1.5, power);
  ringBuzzer (buzzer, getFreq(-2 + offset), beat * 0.5, power);
  ringBuzzer (buzzer, getFreq(-2 + offset), beat * 1.5, power);
  ringBuzzer (buzzer, getFreq(3 + offset), beat * 1, power);
  
  ringBuzzer (buzzer, getFreq(2 + offset), beat * 12, power);
  ringBuzzer (buzzer, 0, beat * 2, power);
  }
}

