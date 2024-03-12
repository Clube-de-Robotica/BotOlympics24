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

/* USER CODE END FUNCTION PROTOTYPES */

//  The setup function runs once when you press reset or power the board
void setup()
{
  /* PRIVATE CODE BEGIN Initializations -------------------------- */
    Serial.begin(9600);        // set baud rate to 9600bps for printing values at serial monitor.
    one.spiConnect(SSPIN);     // start SPI communication module
    one.stop();                // stop motors
    one.obstacleEmitters(OFF); // desactivate IR emitters
    one.minBat(10.5);          // set batery treshold
    Lidar.begin();
    /* USER CODE BEGIN Initializations */

    // place your initializations here

       pinMode(3,OUTPUT);
       digitalWrite(3,1);
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
        delay(500); // wait 2sec

    }
}

//  The loop function runs over and over again forever
void loop()
{ 
  //Testa todoas as possibilidades de movimento dos motores
  
  int speed=100; //velocidade dos motores;

  one.move(speed,speed);
  delay(1000);
  
  one.move(-speed,-speed);
  delay(1000);

  one.move(speed,-speed);
  delay(1000);
 
  one.move(-speed,speed);
  delay(1000);

}