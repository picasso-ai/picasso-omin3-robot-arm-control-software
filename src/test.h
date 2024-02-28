#ifndef __TEST_H__
#define __TEST_H__
#include <Arduino.h>
#include "config.h" // Pin configuration for motor
#include <HardwareSerial.h>
#include <ESP32CAN.h>
#include <CAN_config.h>
#include <SPI.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

extern int txid;
void stop(int);                   // Emergency stop
void set_status(int, float);      // Send control signal to motor (e.g., stop or restart)
int read_status(int);             // Read motor status
void set_mode(int, float);        // Set motor mode (torque, speed, or position)
int read_mode(int);               // Read motor mode
void set_zp(int);                 // Set current position to zero
void set_PID(int, float, float);  // Set PID parameters
float read_PID(int, float);       // Read PID parameters
void set_lim(int, float, float);  // Set limit parameters
float read_lim(int, float);       // Read limit parameters
void spr(int, float);             // Run to target value
void spt(int, float, float);      // Run to target position with given speed and current parameters
void set_ctp(int, float, float);  // Set trajectory based on position
void set_ctv(int, float, float);  // Set trajectory based on speed
void set_ctmf(int, float, float); // Set trajectory based on torque 
void tdr(int, float);             // Run specified trajectory
void record(int, float);          // Record data (position, speed, current)
float read_rd(int, float);        // Read motor state
void set_CAN_ID(int, float);      // Set CAN ID
void reset(int);                  // Reset settings
void receive_data();              // Start receiving data

#endif