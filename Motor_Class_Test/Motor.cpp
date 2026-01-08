#include "Motor.hpp"

Motor::Motor(Encoder* enc, int PWMPin, int directionPinA, int directionPinB, int motorID)
    : encoder(enc), PWM(PWMPin), Direction_A(directionPinA), Direction_B(directionPinB), motorID(motorID) {
}

Motor::~Motor() {
    // Cleanup if necessary
}

void Motor::begin() {
    if (!isEncoderInit()) {
        Serial.print("Error: Encoder not initialized for motor ");
        Serial.println(motorID);
        return;
    }

    pinMode(PWM, OUTPUT);
    pinMode(Direction_A, OUTPUT);
    pinMode(Direction_B, OUTPUT);
    initialized = true;
    lastEncoderReading = encoder->read();  // or 0 if you're resetting
    speed = 250; // Default speed
}


void Motor::setSpeed(int speedVal) {
    speed = speedVal;
    
}

// void Motor::printSpeed() {
//     Serial.print(speed);   
// }

void Motor::forward() {
    analogWrite(PWM, speed);
    digitalWrite(Direction_A, HIGH);
    digitalWrite(Direction_B, LOW);
    motorDir = 1;
    //delay(100);  //Removing because it causes delay and this makes it non Real time
}

void Motor::backward() {
    analogWrite(PWM, speed);
    digitalWrite(Direction_A, LOW);
    digitalWrite(Direction_B, HIGH);
    motorDir = -1;
    //delay(100); //Removing because it causes delay and this makes it non Real time
}

void Motor::stop() {
    analogWrite(PWM, 0);
    digitalWrite(Direction_A, LOW);
    digitalWrite(Direction_B, LOW);
    motorDir = 0;
}

void Motor::run(int direction) {
    if (direction > 0) {
        forward();
    } else if (direction < 0) {
        backward();
    } else {
        stop();
    }
}

void Motor::moveMotor(long long stepCount) {
    long long target = currentPosition + stepCount;
    unsigned long startTime = millis();
    unsigned long timeout = 5000; // Timeout period in milliseconds

    while (currentPosition != target) {
        if (currentPosition < target) {
            run(1);
        } else {
            run(-1);
        }
        currentPosition = encoder->read();

        // Check for timeout
        if (millis() - startTime >= timeout) {
            Serial.println("Motor movement timeout!");
            break;
        }
    }
    stop();
}

double Motor::returnPos() {
    currentPosition= encoder->read();
    return currentPosition;
}


int32_t Motor::readAndResetPos(){
  return encoder->readAndReset();
}

// resetEncodToZero(){


// }

int32_t Motor::getStoredPosition() {
    return currentPosition;
}

void Motor::updateStoredPosition() {
    storedPosition = encoder->read();
}

void Motor::setMotorPosition(double position) {
    targetPosition = position;
    integral = 0.0;   // RESET integral
}

void Motor::addMotorPosition(double delta) {
    targetPosition += delta;
}

bool Motor::isinit() {
    return initialized;
}

bool Motor::isEncoderInit() {
    return encoder != nullptr;
}

bool Motor::isforward() {
    return motorDir == 1;
}

bool Motor::isbackward() {
    return motorDir == -1;
}

bool Motor::isstop() {
    return motorDir == 0;
}

void Motor::zeroOutMotor() {
    encoder->write(0);
    currentPosition = 0;
}

void Motor::testDirection() {
    Serial.println("Testing forward direction (counter-clockwise)...");
    setSpeed(500);
    forward(); // Set direction to forward
    delay(1000); // Keep the motor running in forward direction for 1 second
    stop(); // Stop the motor
    Serial.println("Test complete.");
}

void Motor::testDirection2() {
    Serial.println("Testing forward direction (clockwise)...");
    setSpeed(500);
    forward(); // Set direction to forward
    delay(1000); // Keep the motor running in forward direction for 1 second
    stop(); // Stop the motor
    Serial.println("Test complete.");
}

void Motor::testRevDirection2() {
    Serial.println("Testing backward direction (clockwise)...");
    setSpeed(500);
    backward(); // Set direction to backward
    delay(1000); // Keep the motor running in backward direction for 1 second
    stop(); // Stop the motor
    Serial.println("Test complete.");
}

int Motor::update() {
    int32_t encoderNow = encoder->read();
    int32_t delta = encoderNow - lastEncoderReading;
    lastEncoderReading = encoderNow;

    currentPosition += delta;

    int32_t error = targetPosition - currentPosition;

    // --- INTEGRAL TERM ---
    integral += error;
    double iEffort = KI * integral;

    int pwm = abs(error) * KP + abs(iEffort); 
    pwm = constrain(pwm, MIN_PWM, MAX_PWM);

    int direction = (error > 0) ? 1 : -1;

    setSpeed(pwm);
    run(direction);

    if (abs(error) <= ERROR_DEADBAND) {
        stop();
        integral = 0; // Reset integral when stopped
        return 0;
    }
    return direction;
}



void Motor::setGroupCommand(int direction, int speed) {  
    setSpeed(speed);
    run(direction);
}

void Motor::storePosition() {
    currentPosition = encoder->read();
}
