#include <Arduino.h>
#include "Encoder.h"
#include "Motor.hpp"
#include "PinDefinitions.hpp"

#define ENCODER_USE_INTERRUPTS

// Define encoder pins
const uint8_t M1_encoderPin1 = ENCODER_A1;
const uint8_t M1_encoderPin2 = ENCODER_B1;
const uint8_t M2_encoderPin1 = ENCODER_A2;
const uint8_t M2_encoderPin2 = ENCODER_B2;
const uint8_t M3_encoderPin1 = ENCODER_A3;
const uint8_t M3_encoderPin2 = ENCODER_B3;

// Create the Encoder objects
Encoder m1Encoder(M1_encoderPin1, M1_encoderPin2);
Encoder m2Encoder(M2_encoderPin1, M2_encoderPin2);
Encoder m3Encoder(M3_encoderPin1, M3_encoderPin2);

// Create the Motor objects
Motor motor1(&m1Encoder, PWM_1, DIRECTION_A1, DIRECTION_B1, 1);
Motor motor2(&m2Encoder, PWM_2, DIRECTION_A2, DIRECTION_B2, 2);
Motor motor3(&m3Encoder, PWM_3, DIRECTION_A3, DIRECTION_B3, 3);

//Motor testMotor1(nullptr, 5, 6, 7, 4); //Test Case
double m1EncVal=0;


// Global Variables
const int baseMotorSpeed = 200; // Base PWM value -was 150
long long currentPos1 = 0;
long long currentPos2 = 0;
long long currentPos3 = 0;

// Current sensor parameters
const int currentSensorPin = BOOST_CURRENT_PIN;  // Analog pin for current sensor
const float currentSensorScale = 0.2;  // Sensitivity of ACS712 sensor (adjust based on datasheet)

// Voltage sensor parameters
const int voltageSensorPin = BOOST_VOLTAGE_PIN;  // Analog pin for voltage sensor
const float voltageSensorScale = 5.0 / 1023.0;  // Voltage scaling factor

const float speedScalingFactor1=50000;
const float speedScalingFactor2=50000;
const float speedScalingFactor3=50000;

void setup() {
    Serial.begin(9600);
    pinMode(A0, INPUT_PULLUP);
    // Initialize motors
    motor1.begin();
    motor2.begin();
    motor3.begin();
    
    // Set speed for testing
    motor1.setSpeed(baseMotorSpeed);
    motor2.setSpeed(baseMotorSpeed);
    motor3.setSpeed(baseMotorSpeed);

    // Print initial encoder positions
    Serial.println();
    Serial.print("Encoder1 pos: ");
    Serial.println((int)motor1.returnPos());
    Serial.print("Encoder2 pos: ");
    Serial.println((int)motor2.returnPos());
    Serial.print("Encoder3 pos: ");
    Serial.println((int)motor3.returnPos());
}

void moveAllMotors(long long pos1, long long pos2, long long pos3, int baseSpeed) {
    motor1.setSpeed(baseSpeed);
    motor2.setSpeed(baseSpeed);
    motor3.setSpeed(baseSpeed);
  
    motor1.setMotorPosition(static_cast<double>(pos1));
    motor2.setMotorPosition(static_cast<double>(pos2));
    motor3.setMotorPosition(static_cast<double>(pos3));

    long long maxDistance = abs(pos1 - motor1.returnPos()); //tracks the max distance required to move
       

    bool inPos[3] = {false, false, false};
    unsigned long startTime = millis();
    unsigned long timeout = 10000; // Adjust the timeout as needed
    //int printChk = 0;
    bool whileTimeOut = false;

    while ((!inPos[0]) && !whileTimeOut) {

        if (!inPos[0]) {
          currentPos1 = motor1.getStoredPosition();
            long long remaining1 = abs(pos1 - motor1.returnPos());
             // int speed1 = baseSpeed * remaining1 / maxDistance;
            int speed1 = (-(speedScalingFactor1)/(remaining1+(speedScalingFactor1/baseSpeed))+baseSpeed);
            
            motor1.setSpeed(speed1);
            inPos[0] = (motor1.update() == 0);
            // if(printChk%100==0){
            Serial.print(" Rem Dist: ");
            Serial.print((long)remaining1);
            double tempEncPos = currentPos1-motor1.returnPos();       
            if(tempEncPos>10000){

              remaining1 = remaining1 - 32000;
              
            
            }
            else if(tempEncPos<-10000){

              remaining1 = remaining1 + 32000;
                
              // Serial.print(" Rem Dist: ");
              // Serial.print((int)remaining1);
            }
            // Serial.print(" TmpEncPosi: ");
            // Serial.print(tempEncPos);
            Serial.print(" Motor1 Position: ");
            Serial.print((int)currentPos1);
            
            // }
        }

        // if (!inPos[1]) {
        //   currentPos2 = motor2.getStoredPosition();
        //     long long remaining2 = abs(pos2 - motor2.returnPos());
        //     // int speed2 = baseSpeed * remaining2 / maxDistance;
            
        //     int speed2 = (-(speedScalingFactor2)/(remaining2+(speedScalingFactor2/baseSpeed))+baseSpeed);
        //     motor2.setSpeed(speed2);
        //     // Serial.print(" Speed: ");
        //     // Serial.print(speed2);
        //     // Serial.print(" Rem Dist: "); 
        //     // Serial.print((int)remaining2);
        //     inPos[1] = (motor2.update() == 0);
            
        //     double tempEncPos2 = currentPos2-motor2.returnPos();
        //     if(tempEncPos2>10000){

        //       remaining2 = remaining2 - 32000;
        //     }
        //     else if(tempEncPos2<-10000){

        //       remaining2 = remaining2 + 32000;
        //     }
        //     Serial.print(" Motor2 Position: ");
        //     Serial.print((int)currentPos2);
        //   // }
        // }
        
        // if (!inPos[2]) {
        //     currentPos3 = motor3.getStoredPosition();
        //     long long remaining3 = abs(pos3 - motor3.returnPos());
        //     // int speed3 = baseSpeed * remaining3 / maxDistance;
           
        //    int speed3 = (-(speedScalingFactor3)/(remaining3+(speedScalingFactor3/baseSpeed))+baseSpeed);
        //     motor3.setSpeed(speed3);
        //     // Serial.print(" Speed: ");
        //     // Serial.print(speed3);
        //     // Serial.print(" Rem Dist: ");
        //     // Serial.print((int)remaining3);
        //     inPos[2] = (motor3.update() == 0);
        //     double tempEncPos3 = currentPos3-motor3.returnPos();
        //       if(tempEncPos3>10000){

        //         remaining3 = remaining3 - 32000;
        //       }
        //       else if(tempEncPos3<-10000){

        //         remaining3 = remaining3 + 32000;
        //       }
        //       Serial.print(" Motor3 Position: ");
        //       Serial.print((int)currentPos3);
        //     // }
        // }
        // if(printChk%100==0){
        Serial.println();
        // }
        //Check for timeout
        if (millis() - startTime >= timeout) {
            Serial.println("Synchronization timeout!");
            motor1.setStop();
            // motor2.setStop();
            // motor3.setStop();
            whileTimeOut = true;
        }

        //printChk ++;
    }
    Serial.print(" Motor1 Position: ");
    Serial.print((int)motor1.readAndResetPos());
    // Serial.print(" Motor2 Position: ");
    // Serial.print((int)motor2.readAndResetPos());
    // Serial.print(" Motor3 Position: ");
    // Serial.print((int)motor3.readAndResetPos());

}

void moveMotorTo(Motor& motor, long long targetPos, int baseSpeed) {
    motor.setSpeed(baseSpeed);
    motor.setMotorPosition(targetPos);

    unsigned long startTime = millis();
    unsigned long timeout = 10000; // 10 seconds

    while (motor.update() != 0) {
        Serial.print("Current: "); Serial.println(motor.getStoredPosition());
        if (millis() - startTime > timeout) {
            Serial.println("Timeout!");
            motor.setStop();
            break;
        }
    }
    Serial.println("Done!");
}


void serialHandler(){ 

    if (Serial.available() > 0) { 

         // Read input as a string 

         String input = Serial.readStringUntil('\n'); 

         input.trim(); 

         

         // Verify the input format 

         if (input.indexOf(',') == -1 || input.lastIndexOf(',') == input.indexOf(',')) { 

             Serial.println("Invalid input format. Please enter positions as: pos1,pos2,pos3"); 

             return; 

         } 
    }
}

float readCurrent() {
    int rawCurrentValue = analogRead(currentSensorPin);
    return ((rawCurrentValue-515)*currentSensorScale) ;
}

float readVoltage() {
    int rawVoltageValue = analogRead(voltageSensorPin);
    return (rawVoltageValue * voltageSensorScale * 12);
}

void printCurrentAndVoltage() {
    float current = readCurrent();
    float voltage = readVoltage();
    
    //
    Serial.print("Currentraw: ");
    Serial.print(currentSensorPin);
    
    Serial.print(" Voltageraw: ");
    Serial.print(voltageSensorPin);
    //
    Serial.print("Current: ");
    Serial.print(current);
    Serial.print("A  ");
    Serial.print("Voltage: ");
    Serial.print(voltage);
    Serial.println("V");
}

void testMotor(Motor aMotor){
    bool isInitialized;
    bool encoderInitialized;
    //Initialization Test
    isInitialized = aMotor.isinit();
    Serial.print(isInitialized);
    //bool encoderInitialized;
    //isBoolInitialized = aMotor.isEncoderInit(); //this function needs to be looked at.
    //Serial.print(isBoolInitialized);

    //Connections
    aMotor.testDirection(); //I made motor go forward
    Serial.print(aMotor.getStoredPosition()==0); //False because I moved the motor | Encoder Test here.
    aMotor.testRevDirection2(); //I made motor go reverse

    //Behaviour


}

// void testAllMotors(){
//   testMotor(motor1);
//   testMotor(motor2);
//   testMotor(motor3);
// }

long long pos1,pos2,pos3 = 10;


void loop() {
  //testMotor1.begin();

    //moveAllMotors(pos1, pos2, pos3, baseMotorSpeed);
    //m1EncVal = motor1.returnPos();
   //Serial.println("Encoder Reading: ");
    //Serial.println(m1EncVal);
    //delay(500);
    if (Serial.available() > 0) {
        // Read input as a string
        String input = Serial.readStringUntil('\n');
        input.trim();
        
        // Verify the input format
        if (input.indexOf(',') == -1 || input.lastIndexOf(',') == input.indexOf(',')) {
            Serial.println("Invalid input format. Please enter positions as: pos1,pos2,pos3");
            return;
        }
        //testMotor(motor1);
        
        // Parse positions for each motor
        int pos1End = input.indexOf(',');
        int pos2End = input.indexOf(',', pos1End + 1);
        
        long long pos1 = input.substring(0, pos1End).toInt()*STEP_PER_BLADE_DEGREE;  // Assuming scaling factor of 3
        long long pos2 = input.substring(pos1End + 1, pos2End).toInt()*STEP_PER_BLADE_DEGREE;
        long long pos3 = input.substring(pos2End + 1).toInt()*STEP_PER_BLADE_DEGREE;

        Serial.print("Moving Motors to positions: ");
        Serial.print((int)pos1);
        Serial.print(", ");

        //Serial.print(motor1.printSpeed());

        Serial.print((int)pos2);
        Serial.print(", ");
        Serial.println((int)pos3);
        Serial.print("\n");
        // Synchronize and move all motors to the given positions
        moveMotorTo(motor1, pos1, baseMotorSpeed); //test new function without non-linear adjustments
        //moveAllMotors(pos1, pos2, pos3, baseMotorSpeed);
    }

    //Optionally print current and voltage readings
     //printCurrentAndVoltage();
     //Negative = clckwise ; Positive = Counter Clckwise
    
}