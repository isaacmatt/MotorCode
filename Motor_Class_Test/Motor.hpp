#pragma once
#include <stdint.h>
#include "Encoder.h"
#include <Adafruit_MotorShield.h>
#include "PinDefinitions.hpp"

#define BLADE_TO_MOTOR_RATIO 3
#define STEP_PER_MOTOR_DEGREE 17.0 //17.0 before
#define STEP_PER_BLADE_DEGREE 200.0 //51.0 before
#define MOTOR_STEP_ERROR 25

#define MIN_PWM 40          // Minimum motor effort to overcome stiction
#define MAX_PWM 255         // Full speed
#define KP 1.0              // Proportional gain (tune this)
#define ERROR_DEADBAND 2    // Within this range, consider "at target"
#define KI 0.02  //Integral Term 0.02 to 0.1


class Motor {
private:
    int motorID{-1};
    int PWM;
    int Direction_A;
    int Direction_B;
    Encoder* encoder; // Pointer to Encoder object
    bool initialized = false;
    int motorDir;
    int speed; //PWM value
    long long currentPosition = 0;
    long long targetPosition = 0;
    int32_t lastEncoderReading = 0;
    int32_t storedPosition=0; // New member variable to store the position 
    double integral = 0.0;

    void init();
    
    void forward();
    void backward();
    void stop();
    void run(int direction);
    void moveMotor(long long stepCount);

public:
    Motor(Encoder* enc, int PWMPin, int directionPinA, int directionPinB, int motorID);
    ~Motor();
    void begin();
    void setSpeed(int speed);
    void setForward(){forward();};
    void setBackward(){backward();};
    void setStop(){stop();};
    double returnPos();
    void setMotorPosition(double position);
    void addMotorPosition(double delta);
    bool isinit();
    bool isEncoderInit();
    bool isforward();
    bool isbackward();
    bool isstop();
    void zeroOutMotor();
    void testDirection();
    void testDirection2();
    void testRevDirection2();
    int update();
    void setGroupCommand(int direction, int speed); // New method for group commands
    void storePosition(); // New method to store position
    int32_t readAndResetPos(); //reset Encoder pos to Zero
    // void resetEncodToZero();
    int32_t getStoredPosition(); // New method to get the stored position
    void updateStoredPosition(); // New method to update the stored position
};
