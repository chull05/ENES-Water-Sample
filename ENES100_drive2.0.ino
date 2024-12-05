#include <math.h>
#include "Arduino.h"
#include "Enes100.h"

//Assigning Pins Left Motor (A) 
const int EN_A = 4;//speed
const int IN1_A = 48; //direction
const int IN2_A = 49; //direction

//Assigning Pins Right Motor (B)
const int EN_B = 5;//speed
const int IN1_B = 47; //direction
const int IN2_B = 46; //direction

//Assigning Ultrasonic Pins
// defines pins numbers
const int trigPinA = 6;
const int echoPinA = 7;

const int trigPinB = 9;
const int echoPinB = 8;
//Arm Pins
const int EN_C = 3;
const int IN1_C = 22; 
const int IN2_C = 23; 

// defines variables
long duration;
int distance;

// Pin Definitions
const int waterSensorPowerPin = 32;   // Digital pin to power the water sensor
const int waterSensorPin = A5;       // Analog pin connected to the water sensor
const int pumpControlPin = 28;        // Digital pin controlling the pump
const int waterThreshold = 500;      // Water sensor threshold value (adjust as needed)
const unsigned long pumpRunTime = 10000; // Pump run time in milliseconds (10 seconds)

// Variables
bool pumpRunning = false;             // State of the pump
unsigned long pumpStartTime = 0;      // Time when the pump was turned on

// Color Sensor pins
const int S0 = 11;
const int S1 = 33;
const int S2 = 13;
const int S3 = 12;
const int signal = 34;

// define variables for pulses
unsigned long red;
unsigned long blue;
unsigned long green;
unsigned long clear;

//variable for found pollutant
boolean pollutant = false;

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

    pinMode(waterSensorPowerPin, OUTPUT); // Set sensor power pin as output
    pinMode(pumpControlPin, OUTPUT);     // Set pump control pin as output
    digitalWrite(waterSensorPowerPin, LOW); // Start with sensor off
    digitalWrite(pumpControlPin, LOW);   // Ensure the pump starts off

    Enes100.begin("Project Kowalski", WATER, 249, 51, 50);

    pinMode(EN_C, OUTPUT);
    pinMode(IN1_C, OUTPUT);
    pinMode(IN2_C, OUTPUT);

    analogWrite(EN_C, 75);

    // set pin modes
    pinMode(S0,OUTPUT);
    pinMode(S1,OUTPUT);
    pinMode(S2,OUTPUT);
    pinMode(S3,OUTPUT);
    pinMode(signal,INPUT);

    /* set frequency scaling - 
      S0 S1 | Output frequency scaling
      L  L  | power down
      L  H  | 2%
      H  L  | 20%
      H  H  | 100%
    */
    digitalWrite(S0,HIGH);
    digitalWrite(S1,LOW);
    Serial.begin(9600);
    Enes100.println("Starting directions towards landing zone");
    //Raises arm
    digitalWrite(IN1_C, LOW);
    digitalWrite(IN2_C, HIGH);
    delay(500);
    //navigateMission();
    //moveToMission();
}

void loop() {
    delay(1000);
    float dist1 = measureDistance(trigPinA, echoPinA);
    float dist2 = measureDistance(trigPinB, echoPinB);
    Serial.println(dist1);
    Serial.println(dist2);
    
    //updateCoords();
    //moveTillX(3);
    //break;
}
void moveToMission(){
  setDirection(0, 100);
  delay(2500);
  stopMotors();
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
    if (distanceSensor1 < 30 || distanceSensor2 < 30) {  // Trigger avoidance if either sensor detects an obstacle within 0.3m
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
        if (fabs(currentTheta) < threshold + .6) {
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
    const float threshold = 0.3; // Smaller threshold for precision in radians
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
    while(fabs(Enes100.getX() - xCord) > 0.5){
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
    while (fabs(currentY - yCord) > 0.3) {
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

    setDirection(direction, 120);
    delay(2900);
    stopMotors();
 
}

void navigateMission(){
    float currentY = Enes100.getY();
    //sets OTV to face theta = 0
    rotateOTV();
    if(currentY < 1){ //lower starting position
        turn90Degrees(1);
        moveToMission();
        //complete mission
        digitalWrite(IN1_C, HIGH);
        digitalWrite(IN2_C, LOW);
        delay(500);
        colorSensorRun();
        pumpWater();
        delay (2000);
        turn90Degrees(2);
        moveTillX(3.5);
    }else{ //upper starting position
        turn90Degrees(2);
        moveToMission();
        //complete mission
        digitalWrite(IN1_C, HIGH);
        digitalWrite(IN2_C, LOW);
        delay(500);

        colorSensorRun();
        pumpWater();
        delay(2000);
        turn90Degrees(1);
        moveTillX(3.5);
    }
    stopMotors();
    //adjustment to reach limbo
    if(Enes100.getY() < 1.1){
      turn90Degrees(1);
      moveTillY(1.45);
      turn90Degrees(2);
    }

    setDirection(0, 50);
    delay(1000);
    stopMotors();
    Enes100.println("Mission Complete");

}


//Motor calls
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
        analogWrite(EN_B, power + 30);

        digitalWrite(IN1_A, LOW);
        digitalWrite(IN2_A, HIGH);
         
        delay (10);

        digitalWrite(IN1_B, HIGH);
        digitalWrite(IN2_B, LOW);
       
    }else if (motors == 2){//turn right
        analogWrite(EN_A, power + 30);
        analogWrite(EN_B, power);

        digitalWrite(IN1_B, HIGH);
        digitalWrite(IN2_B, LOW);
        
        delay (10);

        digitalWrite(IN1_A, LOW);
        digitalWrite(IN2_A, HIGH);

        
    }

}

//Color sensor code
boolean colorSensorRun(){
  // clear
  digitalWrite(S2,HIGH);
  digitalWrite(S3,LOW);
  clear = pulseIn(signal,HIGH);
  
  // red
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  red = pulseIn(signal,HIGH);

  // green
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  green = pulseIn(signal,HIGH);

  // blue
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  blue = pulseIn(signal,HIGH);

  /* map the red, green, and blue values to a more intuitive 0-255 range where
     0 means less light and 255 means more. This part will require calibration
     depending on your colored surfaces and ambient light levels.
  */
  red = map(red, 75, 40, 0, 255);
  green = map(green, 85, 55, 0, 255);
  blue = map(blue, 90, 60, 0, 255);  // Blue channel has a different range
  // Serial.println(red);
  // Serial.println(green);
  // Serial.println(blue);
  // turn LEDs on depending on which color is detected
  int runs = 0;

  while (runs <= 10 && pollutant == false){
      if((red>200 && green<100 && blue<100 )|| (red<100 && green>150 && blue<150) || (red<100 && green<100 && blue>80)){  // red detected
        pollutant = true;
        Serial.print("Pollutant was found");
        Serial.print(" ");
      
      // delay();
      }
    
      else{  // none of the three colors above detected
        pollutant = false;
        Serial.println("NO POLUTANT");
          
        delay(100);
      }
  }
  if(pollutant == true){
    Enes100.println("Pollutant was Found");
  }else{
    Enes100.println("NO POLUTANT");
  }

}

//water pump code
void pumpWater (){
  // Power the sensor
  digitalWrite(waterSensorPowerPin, HIGH);  // Turn the sensor ON
  delay(10);                                // Wait 10 milliseconds for stabilization

  // Read water level
  int waterLevel = analogRead(waterSensorPin);

  // Debugging output
  Serial.print("Water level: ");
  Serial.println(waterLevel);
  Enes100.print("Water Level: ");
  Enes100.println(waterLevel);

  // Check if water is detected
  if (!pumpRunning && waterLevel > waterThreshold) {
    Serial.println("Water detected! Turning pump on.");
    digitalWrite(pumpControlPin, HIGH);     // Turn on the pump
    pumpStartTime = millis();               // Record the time the pump started
    pumpRunning = true;                     // Set pump state to running
  }

  // Check if pump should be turned off
  if (pumpRunning && (millis() - pumpStartTime >= pumpRunTime)) {
    Serial.println("Turning pump off.");
    digitalWrite(pumpControlPin, LOW);      // Turn off the pump
    pumpRunning = false;                    // Set pump state to off
  }

  // Turn the sensor off to save power
  digitalWrite(waterSensorPowerPin, LOW);

  delay(100); // Short delay for stability
}

