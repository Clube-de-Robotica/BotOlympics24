/***
 This example was created by Dylan Denizon (dylandfd@gmail.com)
 on the 18th April 2022

 This code was made for the Bot'n Roll part of the Bot Olympics competition.
 https://botolympics.pt/

Description:
This file contains the code that serves as a base to start programming with the Bot'n Roll robots,
dealing with the initiation of sensors and libraries in order to facilitate integration with the robots.
Participants are advised to only write code inside the beacons:

** USER CODE BEGIN code **

// place here your code

** CODE END code **

in order to help in the organization of code beings and not to harm the initialization of sensors and libraries.
Good luck and have fun
*/

/* PRIVATE CODE BEGIN Includes */
#include <Wire.h>
#include <BnrOneA.h>
#include <Arduino.h>
#include <LiDAR.h>

/* PRIVATE CODE BEGIN Includes */
/* USER CODE BEGIN Includes -------------------------- */

// place your includes here

/* CODE END Includes */

/* PRIVATE CODE BEGIN DEFINE -------------------------- */
#define SSPIN 2
/* USER CODE BEGIN DEFINE */

// place your defines here

/* CODE END DEFINE */

/* PRIVATE CODE BEGIN VARIABLES -------------------------- */
/* USER CODE BEGIN VARIABLES */

// place your variables here

/* USER CODE END VARIABLES */

/* PRIVATE CODE BEGIN CLASSES -------------------------- */
BnrOneA one;
/* USER CODE BEGIN CLASSES */

// place your classes here

/* USER CODE END CLASSES */

/* PRIVATE CODE BEGIN FUNCTION PROTOTYPES -------------------------- */
/* USER CODE BEGIN FUNCTION PROTOTYPES */

// place your functions here

void rotate(int graus, int direction);

//  The setup function runs once when you press reset or power the board
void setup()
{
  /* PRIVATE CODE BEGIN Initializations -------------------------- */
    Serial.begin(9600);        // set baud rate to 9600bps for printing values at serial monitor.
    one.spiConnect(SSPIN);     // start SPI communication module
    one.stop();                // stop motors
    one.obstacleEmitters(OFF); // desactivate IR emitters
    one.minBat(3);          // set batery treshold
    Lidar.begin();
    /* USER CODE BEGIN Initializations */
    one.magnet(1);
    // place your initializations here

    /* USER CODE END Initializations */

    Lidar.scanI2C();
    while (one.readButton() != 1)
    {
        // stuff to do while waiting for button PB1 to be pressed

        // example to get lidars values and display to Serial monitor
        Serial.println("DISTANCE SENSOR VALUES: ");
        uint16_t left = Lidar.getLidarLeftDistance();
        uint16_t front = Lidar.getLidarFrontDistance();
        uint16_t right = Lidar.getLidarRightDistance();

        Serial.println(
            "Left: " + String(left) + "mm" + 
            "\nFront: " + String(front) + "mm" + 
            "\nRight: " + String(right) + "mm"
        );

        Serial.println("LINE SENSOR VALUES: ");
        int sensor0 = one.readAdc(0);
        int sensor1 = one.readAdc(1);
        int sensor2 = one.readAdc(2);
        int sensor3 = one.readAdc(3);
        int sensor4 = one.readAdc(4);
        int sensor5 = one.readAdc(5);
        int sensor6 = one.readAdc(6);
        int sensor7 = one.readAdc(7);

        // Print values on Serial Monitor
        Serial.print(sensor0);
        Serial.print(" ");
        Serial.print(sensor1);
        Serial.print(" ");
        Serial.print(sensor2);
        Serial.print(" ");
        Serial.print(sensor3);
        Serial.print(" ");
        Serial.print(sensor4);
        Serial.print(" ");
        Serial.print(sensor5);
        Serial.print(" ");
        Serial.print(sensor6);
        Serial.print(" ");
        Serial.print(sensor7);
        Serial.print(" ");
        Serial.println();
        delay(2000); // wait 2sec

    } 
}

int checkWallFront(){
  if(Lidar.getLidarFrontDistance() >120){
    return 0;
  }
  return 1 ;
}

int checkWallLeft(){
  if(Lidar.getLidarLeftDistance() >150){
    return 0;
  }
  return 1;
}

int checkWallRight(){
  if(Lidar.getLidarRightDistance()>300)
    return 0;
  return 1;
}

void followWallRight(){
  while(checkWallRight()){
    if(Lidar.getLidarRightDistance()<=100){
      one.move(15,20);
    }
    if(checkWallFront()){
      rotate(90,0);
      one.move(15,20);
    }
    else if(Lidar.getLidarRightDistance()>=150 && Lidar.getLidarRightDistance()<=300){
      one.move(25,15);
    }
    else{
    one.move(20,20);
    }
  }
  delay(500);
  rotate(90,1);
}


void rotate(int graus, int direction){ // 0 left, 1 right

  if(direction){
    one.move(20,-20);
    delay(4.10*graus);
  }
  else{
    one.move(-20,20);
    delay(4.10*graus);
  }
}

int checkWhite(int indice){
  int limiar = 100;
  if(one.readAdc(indice) <limiar)
  return 1;
  else
  return 0;
}

int allWhite(){
  int limiar = 100;
  int counter = 0;
  for(int i = 0;i<8;i++){
    if(checkWhite(i))
      counter++;
  }
  if(counter >=4){
    return 1;
  }
  else
  return 0;
}

void checkLine(){
    int SPEED= 30 ;
    int limiar = 115;

    if(allWhite()){
      one.brake(100,100);
      delay(300);
      return ;
    }
    else if(one.readAdc(3) <limiar && one.readAdc(4) < limiar){
        one.move(SPEED*2, SPEED*2);
    }
    else if(one.readAdc(0) < limiar  && one.readAdc(1) < limiar){
      one.move(SPEED * 0.0001, SPEED * 3.5);
    }
    else if(one.readAdc(7) < limiar && one.readAdc(6)<limiar){
      one.move(SPEED * 3.5 ,SPEED * 0.0001);
    }
    else if(one.readAdc(2)<limiar){
      one.move(SPEED*0.5,SPEED*3);
    }
    else if(one.readAdc(5)<limiar){
      one.move(SPEED*3,SPEED*0.5);
    }
  return;
}






//  The loop function runs over and over again forever
void loop()
{
  
  /*
  one.move(22,20);

  
  if(checkWallRight()){
    followWallRight();
  }  

  if(checkWallLeft()){
    rotate(180,1);

  }

  if(checkWallFront()){
    rotate(90,0);
  } */
      
  int endLine=0;


  if(allWhite()==0){
    checkLine();
  }
  else{
    one.stop();
  }

                                                                                                                                            
  

}





