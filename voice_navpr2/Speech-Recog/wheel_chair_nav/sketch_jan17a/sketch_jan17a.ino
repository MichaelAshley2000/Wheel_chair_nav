#include <Arduino.h>
#include <avr/io.h>
#include <Sabertooth.h>

// define variables
unsigned long current_time_millis;
unsigned long starting_time_millis;
unsigned long x_var;
int current_speed = 0;
double gradient = 0.002;

// define functions
void set_speed(int cmd_speed);

//intialise functions
Sabertooth ST(128);


void setup() {
  SabertoothTXPinSerial.begin(9600); // 9600 is the default baud rate for Sabertooth packet serial.
  ST.autobaud();
  delay(1000);

  set_speed(127);
  Serial.println(".....................................................................................................");
  delay(2000);

  set_speed(200);
  Serial.println(".....................................................................................................");
  delay(2000);

  set_speed(60);
  Serial.println(".....................................................................................................");
  delay(2000);

  set_speed(-127);
  Serial.println(".....................................................................................................");
  delay(2000);

  set_speed(-200);
  Serial.println(".....................................................................................................");
  delay(2000);

  set_speed(-50);
  Serial.println(".....................................................................................................");
  delay(2000);

  set_speed(0);
  Serial.println(".....................................................................................................");
  delay(2000);
}

void loop() {
  delay(1000);
}

void set_speed(int cmd_speed){
  if(cmd_speed > current_speed){
    if (gradient < 0){
      gradient = -gradient;
    }
    
    starting_time_millis = millis();
    while (cmd_speed > current_speed){
      current_time_millis = millis();
      x_var = current_time_millis - starting_time_millis;
      current_speed = int(gradient * x_var + current_speed);
      delay(5);
      ST.motor(1, current_speed);
      ST.motor(2, current_speed);
    }
  }

  else{
    if (gradient > 0){
      gradient = -gradient;
    }

    starting_time_millis = millis();
    while (cmd_speed < current_speed){
      current_time_millis = millis();
      x_var = current_time_millis - starting_time_millis;
      current_speed = int(gradient * x_var + current_speed);
      delay(1);
      Serial.print(x_var);
      Serial.print("\t\t");
      Serial.println(current_speed);
      ST.motor(1, current_speed);
      ST.motor(2, current_speed);
    }
  }
}  