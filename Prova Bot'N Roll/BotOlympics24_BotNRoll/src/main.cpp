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
int checkWhite(int indice)
{
  int limiar = 100;
  if (one.readAdc(indice) < limiar)
    return 1;
  else
    return 0;
}

int checkBlack(int indice)
{
  int limiar = 500;
  if (one.readAdc(indice) > limiar)
    return 1;
  else
    return 0;
}

int allWhite()
{
  int limiar = 100;
  int counter = 0;
  for (int i = 0; i < 8; i++)
  {
    if (checkWhite(i))
      counter++;
  }
  if (counter >= 7)
  {
    return 1;
  }
  else
    return 0;
}

bool allBlack()
{
  int limiar = 100;
  int counter = 0;
  for (int i = 0; i < 8; i++)
  {
    if (checkBlack(i))
      counter++;
  }
  if (counter >= 7)
  {
    return true;
  }
  else
    return false;
}

int mindex(int leituras[])
{

  int min = leituras[0];
  int mindex = 0;
  for (int i = 0; i < 8; i++)
  {
    if (leituras[i] < min)
    {
      min = leituras[i];
      mindex = i;
    }
  }

  return mindex;
};
int readline()
{
  int leituras[8];
  int limiar = 100;
  // int maxval=0;
  // int minval=0;

  // if (allWhite() || allBlack())
  // {
  //   return -1;                  Joca isto e 100% do tempo seu burro da sempre -1
  // }

  for (int i = 0; i < 8; i++)
  {
    leituras[i] = one.readAdc(i);
  }

  if (leituras[3] <= limiar && leituras[4] <= limiar)
  {
    return 0;
  }

  if (mindex(leituras) < 3)
  {

    return (map((leituras[0] * 10 + leituras[1] * 8 + leituras[2] * 3 + leituras[3] * 1) / (10 + 8 + 3 + 1), 0, 1023, 0, 100));
  }

  else if (mindex(leituras) > 4)
  {
    return (map((leituras[7] * (-10) + leituras[6] * (-8) + leituras[5] * (-3) + leituras[4] * (-1)) / (10 + 8 + 3 + 1), -1023, 0, -100, 0));
  }
  else
    return 0;
};

int readwall()
{

  int limiar = 200; // distancia lidar direita
  int dist = Lidar.getLidarRightDistance();

  if (dist == limiar)
  {
    return 0;
  }

  if (dist > limiar)
  {

    return (map(dist, 0, 2600, 0, 100));
  }

  else if (dist < limiar)
  {
    return (map(dist, 0, 2600, -100, 0));
  }
  else
    return 0;
};

//  The setup function runs once when you press reset or power the board
void setup()
{
  /* PRIVATE CODE BEGIN Initializations -------------------------- */
  Serial.begin(9600);        // set baud rate to 9600bps for printing values at serial monitor.
  one.spiConnect(SSPIN);     // start SPI communication module
  one.stop();                // stop motors
  one.obstacleEmitters(OFF); // desactivate IR emitters
  one.minBat(3);             // set batery treshold
  Lidar.begin();
  /* USER CODE BEGIN Initializations */
  one.magnet(1);
  // place your initializations here

  /* USER CODE END Initializations */

  Lidar.scanI2C();
  while (one.readButton()!=1)
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
        "\nRight: " + String(right) + "mm");

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
int estado = 0;
//  The loop function runs over and over again forever
void loop()
{Serial.print("aa");
  int speed = 30;
  int speedL = 0, speedR = 0;
  int curvespeed = 5; // limita a velocidade da roda interior

  int gain = 80; // limita o ganho

  if (one.readAdc(0) < 100 &&
      one.readAdc(1) < 100 &&
      one.readAdc(2) < 100 &&
      one.readAdc(3) < 100 &&
      one.readAdc(4) < 100 &&
      one.readAdc(5) < 100 &&
      one.readAdc(6) < 100 &&
      one.readAdc(7) < 100)
  {
    one.magnet(0);
    estado = 1;
    one.move(-30, -30);
    delay(1500);
    one.move(-30, 30);
    delay(500);
  }
  switch (estado)
  {
  case 0:
  {
    int line = readline();Serial.println(line);
    if (line < 0)
    {
      speedR = (speed - (line * line)) / gain;

      speedL = speed + (speed - speedL);
    }
    else
    {
      speedL = (speed - (line * line)) / gain;

      speedR = speed + (speed - speedR);
    }

    if (speedL < -1)
      speedL = -1;
    if (speedR < -1)
      speedR = -1;
    if (speedL > speed + curvespeed)
      speedL = speed + curvespeed;
    if (speedR > speed + curvespeed)
      speedR = speed + curvespeed;

    one.move(speedL, speedR);
    break;
  }
  case 1:
  {
    int wall = readwall();
    if (wall < 0)
    {
      speedR = (speed - (wall * wall)) / gain;

      speedL = speed + (speed - speedL);
    }
    else
    {
      speedL = (speed - (wall * wall)) / gain;

      speedR = speed + (speed - speedR);
    }

    if (speedL < -1)
      speedL = -1;
    if (speedR < -1)
      speedR = -1;
    if (speedL > speed + curvespeed)
      speedL = speed + curvespeed;
    if (speedR > speed + curvespeed)
      speedR = speed + curvespeed;

    one.move(speedR, speedL);
    break;
  }
  }
}
