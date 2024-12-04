#include <math.h>
#include "Arduino.h"
#include "Enes100.h"

//Assigning Pins Left Motor (A) 
const int EN_A = 10;//speed
const int IN1_A = 48; //direction
const int IN2_A = 49; //direction

//Assigning Pins Right Motor (B)
const int EN_B = 11;//speed
const int IN1_B = 47; //direction
const int IN2_B = 46; //direction

//Assigning Ultrasonic Pins
// defines pins numbers
const int trigPinA = 7;
const int echoPinA = 8;

const int trigPinB = 5;
const int echoPinB = 6;

// defines variables
long duration;
int distance;


void setup() {
    // put your setup code here, to run once:
    pinMode(EN_A, OUTPUT);
    pinMode(IN1_A, OUTPUT);
    pinMode(IN2_A, OUTPUT);

    pinMode(EN_B, OUTPUT);
    pinMode(IN1_B, OUTPUT);
    pinMode(IN2_B, OUTPUT);

    pinMode(trigPinA, OUTPUT); // Sets the trigPin as an Output
    pinMode(echoPinA, INPUT); // Sets the echoPin as an Input

    pinMode(trigPinB, OUTPUT); // Sets the trigPin as an Output
    pinMode(echoPinB, INPUT); // Sets the echoPin as an Input

    Enes100.begin("Project Kowalski", WATER, 249, 53, 52);
    
    Enes100.println("Starting directions towards landing zone");
    //setDirection(1, 75);
    navigateMission();
}

void loop() {
    delay(1000);
    updateCoords();
    moveTillX(3);
    //break;
}

void updateCoords()
{

  float x, y, theta; boolean v;

  x = Enes100.getX();
  y = Enes100.getY();
  theta = Enes100.getTheta();
  v = Enes100.isVisible();
  
  if(v){
    Enes100.print("X = "); Enes100.println(x);
    Enes100.print("Y = "); Enes100.println(y);
    Enes100.print("Theta = "); Enes100.println(theta);
  }else{
    Enes100.println("Not Found");
  }
  
}


float measureDistance(int trigPin, int echoPin){
  digitalWrite(trigPin, LOW);
  delay(2);
  digitalWrite(trigPin, HIGH);
  delay(10);
  digitalWrite(trigPin, LOW);

  // Read the echo pin and calculate distance based on pulse duration
  long duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.034 / 2;  // Convert time to distance (in meters)
  return distance;
}

void distSense()
{
  // Measure distances from both sensors
    float distanceSensor1 = measureDistance(trigPinA, echoPinA);
    float distanceSensor2 = measureDistance(trigPinB, echoPinB);

    // Print distances to Enes100 for monitoring
    Enes100.print("Sensor 1 Distance: ");
    Enes100.println(distanceSensor1);
    Enes100.print("Sensor 2 Distance: ");
    Enes100.println(distanceSensor2);

    // Set the logic based on the sensor readings
    if (distanceSensor1 < 0.3 || distanceSensor2 < 0.3) {  // Trigger avoidance if either sensor detects an obstacle within 0.3m
        if (Enes100.getY() < 1) {
            turn90Degrees(1);
            moveTillY(1.45);
            turn90Degrees(2);
            delay(100);
        } else {
            turn90Degrees(2);
            moveTillY(0.55);
            turn90Degrees(1);
            delay(100);
        }
    }  
}


void rotateOTV() {
    // Set a maximum speed for the motors
    const int maxSpeed = 150;
    // Threshold for stopping (.2 radians)
    const float threshold = .2; 
    
    // Continue rotating until theta is within the specified range
    while (fabs(Enes100.getTheta()) > threshold) {
        updateCoords ();
        // Get the current theta
        float currentTheta = Enes100.getTheta();
    
        // Calculate the speed based on how close we are to zero
        int speed = maxSpeed;

        // Adjust the speed based on the angle's proximity to zero
        if (fabs(currentTheta) < threshold + .2) {
            //map(int x, int in_min, int in_max, int out_min, int out_max)
            speed = map(fabs(currentTheta), threshold, 0.5, 50, maxSpeed); 
            // Slow down as it approaches threshold
        }

        // Control the motor direction and speed
        if (currentTheta < 0) {
            // clockwise if theta is negative
            setDirection(2, speed);
        } else {
            // counterclockwise if theta is positive
            setDirection (1, speed);
        }
        //Motors need to stop to check thetha
        delay(100);
    }
    stopMotors();
}
const float kP_theta = 1.0;   // Proportional gain for angle
const float kI_theta = 0.1;   // Integral gain for accuracy in angle control
const float kD_theta = 0.05;  // Derivative gain for angle stability

void rotateToZero() {
    // Initialize PID control variables
    static float integralTheta = 0.0;
    static float previous_errorTheta = 0.0;
    const float threshold = 0.05; // Smaller threshold for precision in radians
    const float dt = 0.1;         // Time interval for PID calculation

    // Loop until the robot's theta angle is within the defined threshold
    while (fabs(Enes100.getTheta()) > threshold) {
        updateCoords();

        // Get the current theta and calculate the error (target is 0)
        float currentTheta = Enes100.getTheta();
        float errorTheta = -currentTheta; // Target angle is zero

        // Calculate integral and derivative for PID
        integralTheta += errorTheta * dt;
        float derivativeTheta = (errorTheta - previous_errorTheta) / dt;
        previous_errorTheta = errorTheta;

        // Calculate control output using PID
        float controlOutputTheta = kP_theta * errorTheta + kI_theta * integralTheta + kD_theta * derivativeTheta;

        // Adjust speed based on control output and constrain to maxSpeed
        int speed = constrainAng(fabs(controlOutputTheta), 50, 150);  // Set min speed of 50 for responsiveness

        // Determine motor direction and set speed
        if (currentTheta < 0) {
            // Rotate clockwise if theta is negative
            setDirection(2, speed);
        } else {
            // Rotate counterclockwise if theta is positive
            setDirection(1, speed);
        }

        // Stop motors momentarily to update theta reading
        stopMotors();
    }
    // Final stop when target angle is reached
    stopMotors();
}


void moveTillX(float xCord){
    //Moves Tank till x values reach target within threshold of .3
    while(fabs(Enes100.getX() - xCord) > 0.3){
        Enes100.println("Reached");
        setDirection(0, 200);
        delay(100);
        stopMotors();
        
        float distanceSensor1 = measureDistance(trigPinA, echoPinA);
        float distanceSensor2 = measureDistance(trigPinB, echoPinB);

        int count = 0;
        if(distanceSensor1 < 0.3 || distanceSensor2 < 0.3){
            distSense();
            ++count;
            if(count <= 2){
              moveCenter();
              count == 0;
            }
        }


    }
    if(Enes100.getY() < 1.1){
      turn90Degrees(1);
      moveTillY(1.45);
      turn90Degrees(2);
    }
}

void moveCenter(){
    if (Enes100.getY() < 1) {
        turn90Degrees(1);
        moveTillY(0);
        turn90Degrees(2);
        delay(100);
                
     } else {
         turn90Degrees(2);
         moveTillY(0);
         turn90Degrees(1);
         delay(100);
                
        }   
}

void moveTillY(float yCord){
    //Moves Tank till y values reach target within threshold of .3
    Enes100.println("MoveTill");
    float currentY = Enes100.getY();
    while (fabs(currentY - yCord) > 0.1) {
        currentY = Enes100.getY();

        // Calculate the difference
        float distance = yCord - currentY;

        // Calculate speed (you can customize this)
        int maxSpeed = 100;
        int speed = maxSpeed;

        // Optional: Adjust speed based on how close you are to the target
        if (fabs(distance) < 0.2) {
            speed = map(fabs(distance), 0.0, 0.2, 50, maxSpeed);
        }
        setDirection(0, speed);
        // Optional: Add a small delay to allow the motors to react
        delay(10);
    }

    // Stop the motors once the target x coordinate is reached
    stopMotors();
}


void moveOTV(int time, int direction){
    setDirection(direction, 255);
    delay(time);
    stopMotors();
}

void moveObstacle(int time, int direction){
     moveTillX(3.5);
     stopMotors();
}

void stopMotors(){
    //setLeftMotorPWM
    analogWrite(EN_A, 0);
    analogWrite(EN_B, 0);

    digitalWrite(IN1_A, LOW);
    digitalWrite(IN2_A, LOW);

    digitalWrite(IN1_B, LOW);
    digitalWrite(IN2_B, LOW);
    //setRightMotorPWM
}
// Constants
const float MAX_TURN_SPEED = 255; // Maximum turn speed for motors
const float THRESHOLD = 0.1;       // Angle threshold for stopping
const float SOME_CONSTANT = 1.0;   // Proportional gain for turning

// Constrain function
const int constrainAng(float value, int minValue, int maxValue) {
    if (value < minValue) {
        return minValue;
    } else if (value > maxValue) {
        return maxValue;
    } else {
        return value;
    }
}

float normalizeTheta(float theta) {
    // Normalize theta to the range [-pi, pi]
    while (theta > PI) {
        theta -= 2 * PI;
    }
    while (theta < -PI) {
        theta += 2 * PI;
    }
    return theta;
}

void turn90Degrees(int direction){
    //should pass in 1 (left) or 2 (right)
    float target;
    float currentTheta = Enes100.getTheta();
    
    if(direction == 1){
        //determines 90 degree target theta left turn
       target = normalizeTheta(currentTheta + (PI/2));
    }else if (direction == 2){
        //detemines 90 degree target theta right turn
      target = normalizeTheta(currentTheta - (PI/2));
    } else {
        //Invalid direction, exit the function
        return;
    }
    
    Enes100.println( target);
    
    while (fabs(normalizeTheta(Enes100.getTheta()) - target) > .5) {
        updateCoords ();
        currentTheta = Enes100.getTheta();
    
        // Calculate the speed based on how close we are to zero
        int maxSpeed = 160;
        int speed = maxSpeed;

        // Adjust the speed based on the angle's proximity to zero
        if (fabs (normalizeTheta(currentTheta-target)) <  0.5) {
            //map(int x, int in_min, int in_max, int out_min, int out_max)
            
            speed = map(fabs(currentTheta - target), 0, .5, 20, maxSpeed); 
            // Slow down as it approaches threshold
        }

        // Control the motor direction and speed
        if (normalizeTheta(currentTheta - target) < 0) {
            // clockwise if theta is negative
            setDirection(2, speed);
        } else {
            // counterclockwise if theta is positive
            setDirection (1, speed);
        }

        delay(10);
       
        
    }
     stopMotors();
 
}

void navigateMission(){
    float currentY = Enes100.getY();
    
    if(currentY < 1){ //lower starting position
        rotateOTV(); 
        turn90Degrees(1);
        moveTillY (1.5);
        delay (1000);
        turn90Degrees(2);
        moveTillX(3.5);
    }else{ //upper starting position
        rotateOTV();
        turn90Degrees(2);
        moveTillY(.55);
        delay(1000);
        turn90Degrees(1);
        moveTillX(3.5);
    }
    // Initial positioning with PID control
    navigateToPosition(3.0, 1.5);
    
    // Fine adjustments using direct movement
    moveTillX(3.0);  // Move to specific X coordinate
    moveTillY(1.5);  // Then move to specific Y coordinate
}



void setDirection(int motors, int power) {
    if(power > 150){
        power = 150;
    }else if(power < -150){
        power = -150;
    }
    
    if (motors == 0){//straight
        analogWrite(EN_A, 75);
        analogWrite(EN_B, 75);

        digitalWrite(IN1_A, LOW);
        digitalWrite(IN2_A, HIGH);

        digitalWrite(IN1_B, LOW);
        digitalWrite(IN2_B, HIGH);
    }else if (motors == 1){//turn left
        analogWrite(EN_A, power);
        analogWrite(EN_B, power);

        digitalWrite(IN1_A, HIGH);
        digitalWrite(IN2_A, LOW);

        digitalWrite(IN1_B, LOW);
        digitalWrite(IN2_B, HIGH);
       
    }else if (motors == 2){//turn right
        analogWrite(EN_A, power);
        analogWrite(EN_B, power);

        digitalWrite(IN1_A, LOW);
        digitalWrite(IN2_A, HIGH);

        digitalWrite(IN1_B, HIGH);
        digitalWrite(IN2_B, LOW);
    }

}

// PID control constants
const float kP = 1.0;   // Proportional gain
const float kI = 0.5;   // Integral gain for accuracy
const float kD = 0.3;   // Derivative gain for stability

// Function to control position based on PID for X or Y axis
void positionControl(float targetPos, char axis) {
    // Initialize control variables
    static float integral = 0.0;
    static float previous_error = 0.0;
    const float dt = 0.1; // Time interval

    // Get current position based on specified axis
    float currentPos = (axis == 'X') ? Enes100.getX() : Enes100.getY();
    float error = targetPos - currentPos;

    // Calculate integral and derivative terms
    integral += error * dt;
    float derivative = (error - previous_error) / dt;
    previous_error = error;

    // Control output
    float controlOutput = kP * error + kI * integral + kD * derivative;
    
    // Convert control output to motor speeds
    int speed = constrainAng(controlOutput, -255, 255);
    setDirection(0, speed); // Move forward or backward to reach the target position

    // Stop motors if target is reached within a threshold
    if (fabs(error) < 0.3) {
        stopMotors();
    }
}

// Main function to call during navigation
void navigateToPosition(float targetX, float targetY) {
    // Update current position and navigate to each axis sequentially
    positionControl(targetX, 'X');
    delay(500); // Delay for stabilization
    positionControl(targetY, 'Y');
}

