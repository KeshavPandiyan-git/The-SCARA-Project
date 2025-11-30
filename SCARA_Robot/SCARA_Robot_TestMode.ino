/*
   Arduino based SCARA Robot - TEST MODE (No Hardware Required)
   Modified version for testing GUI without physical robot
   by Dejan, www.HowToMechatronics.com
   AccelStepper: http://www.airspayce.com/mikem/arduino/AccelStepper/index.html

   This version skips homing and motor movements for testing GUI communication
*/

#include <AccelStepper.h>
#include <Servo.h>
#include <math.h>

#define limitSwitch1 11
#define limitSwitch2 10
#define limitSwitch3 9
#define limitSwitch4 A3

// Define the stepper motors and the pins the will use
AccelStepper stepper1(1, 2, 5); // (Type:driver, STEP, DIR)
AccelStepper stepper2(1, 3, 6);
AccelStepper stepper3(1, 4, 7);
AccelStepper stepper4(1, 12, 13);

Servo gripperServo;  // create servo object to control a servo

double x = 10.0;
double y = 10.0;
double L1 = 228; // L1 = 228mm
double L2 = 136.5; // L2 = 136.5mm
double theta1, theta2, phi, z;

int stepper1Position, stepper2Position, stepper3Position, stepper4Position;

const float theta1AngleToSteps = 44.444444;
const float theta2AngleToSteps = 35.555555;
const float phiAngleToSteps = 10;
const float zDistanceToSteps = 100;

byte inputValue[5];
int k = 0;

String content = "";
int data[10];

int theta1Array[100];
int theta2Array[100];
int phiArray[100];
int zArray[100];
int gripperArray[100];
int positionsCounter = 0;

void setup() {
  Serial.begin(115200);

  pinMode(limitSwitch1, INPUT_PULLUP);
  pinMode(limitSwitch2, INPUT_PULLUP);
  pinMode(limitSwitch3, INPUT_PULLUP);
  pinMode(limitSwitch4, INPUT_PULLUP);

  // Stepper motors max speed
  stepper1.setMaxSpeed(4000);
  stepper1.setAcceleration(2000);
  stepper2.setMaxSpeed(4000);
  stepper2.setAcceleration(2000);
  stepper3.setMaxSpeed(4000);
  stepper3.setAcceleration(2000);
  stepper4.setMaxSpeed(4000);
  stepper4.setAcceleration(2000);

  gripperServo.attach(A0, 600, 2500);
  // initial servo value - open gripper
  data[6] = 180;
  gripperServo.write(data[6]);
  delay(1000);
  data[5] = 100;
  
  // SKIP HOMING FOR TEST MODE
  // homing(); // Commented out - no hardware needed for GUI testing
  
  // Initialize positions to zero for testing
  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
  stepper3.setCurrentPosition(0);
  stepper4.setCurrentPosition(0);
  
  Serial.println("SCARA Robot - TEST MODE Ready");
  Serial.println("GUI connection active - no hardware required");
}

void loop() {

  if (Serial.available()) {
    content = Serial.readString(); // Read the incomding data from Processing
    // Extract the data from the string and put into separate integer variables (data[] array)
    for (int i = 0; i < 10; i++) {
      int index = content.indexOf(","); // locate the first ","
      data[i] = atol(content.substring(0, index).c_str()); //Extract the number from start to the ","
      content = content.substring(index + 1); //Remove the number from the string
    }
    /*
     data[0] - SAVE button status
     data[1] - RUN button status
     data[2] - Joint 1 angle
     data[3] - Joint 2 angle
     data[4] - Joint 3 angle
     data[5] - Z position
     data[6] - Gripper value
     data[7] - Speed value
     data[8] - Acceleration value
    */
    // If SAVE button is pressed, store the data into the appropriate arrays
    if (data[0] == 1) {
      theta1Array[positionsCounter] = data[2] * theta1AngleToSteps; //store the values in steps = angles * angleToSteps variable
      theta2Array[positionsCounter] = data[3] * theta2AngleToSteps;
      phiArray[positionsCounter] = data[4] * phiAngleToSteps;
      zArray[positionsCounter] = data[5] * zDistanceToSteps;
      gripperArray[positionsCounter] = data[6];
      positionsCounter++;
      Serial.print("Position saved: ");
      Serial.print("J1=");
      Serial.print(data[2]);
      Serial.print(" J2=");
      Serial.print(data[3]);
      Serial.print(" J3=");
      Serial.print(data[4]);
      Serial.print(" Z=");
      Serial.print(data[5]);
      Serial.print(" G=");
      Serial.println(data[6]);
    }
    // clear data
    if (data[0] == 2) {
      // Clear the array data to 0
      memset(theta1Array, 0, sizeof(theta1Array));
      memset(theta2Array, 0, sizeof(theta2Array));
      memset(phiArray, 0, sizeof(phiArray));
      memset(zArray, 0, sizeof(zArray));
      memset(gripperArray, 0, sizeof(gripperArray));
      positionsCounter = 0;
      Serial.println("Program cleared");
    }
  }
  // If RUN button is pressed
  while (data[1] == 1) {
    stepper1.setSpeed(data[7]);
    stepper2.setSpeed(data[7]);
    stepper3.setSpeed(data[7]);
    stepper4.setSpeed(data[7]);
    stepper1.setAcceleration(data[8]);
    stepper2.setAcceleration(data[8]);
    stepper3.setAcceleration(data[8]);
    stepper4.setAcceleration(data[8]);

    // execute the stored steps (SIMULATED - no actual movement)
    for (int i = 0; i <= positionsCounter - 1; i++) {
      if (data[1] == 0) {
        break;
      }
      
      // In test mode, just update positions without moving motors
      stepper1.setCurrentPosition(theta1Array[i]);
      stepper2.setCurrentPosition(theta2Array[i]);
      stepper3.setCurrentPosition(phiArray[i]);
      stepper4.setCurrentPosition(zArray[i]);
      
      Serial.print("Executing position ");
      Serial.print(i + 1);
      Serial.print("/");
      Serial.print(positionsCounter);
      Serial.print(": J1=");
      Serial.print(theta1Array[i] / theta1AngleToSteps);
      Serial.print(" J2=");
      Serial.print(theta2Array[i] / theta2AngleToSteps);
      Serial.println();
      
      if (i == 0) {
        gripperServo.write(gripperArray[i]);
      }
      else if (gripperArray[i] != gripperArray[i - 1]) {
        gripperServo.write(gripperArray[i]);
        delay(800); // wait 0.8s for the servo to grab or drop - the servo is slow
      }
      
      delay(500); // Simulate movement time

      //check for change in speed and acceleration or program stop
      if (Serial.available()) {
        content = Serial.readString(); // Read the incomding data from Processing
        // Extract the data from the string and put into separate integer variables (data[] array)
        for (int i = 0; i < 10; i++) {
          int index = content.indexOf(","); // locate the first ","
          data[i] = atol(content.substring(0, index).c_str()); //Extract the number from start to the ","
          content = content.substring(index + 1); //Remove the number from the string
        }

        if (data[1] == 0) {
          break;
        }
      }
    }
    Serial.println("Program execution complete");
  }

  // Update positions without moving motors (test mode)
  stepper1Position = data[2] * theta1AngleToSteps;
  stepper2Position = data[3] * theta2AngleToSteps;
  stepper3Position = data[4] * phiAngleToSteps;
  stepper4Position = data[5] * zDistanceToSteps;

  stepper1.setSpeed(data[7]);
  stepper2.setSpeed(data[7]);
  stepper3.setSpeed(data[7]);
  stepper4.setSpeed(data[7]);

  stepper1.setAcceleration(data[8]);
  stepper2.setAcceleration(data[8]);
  stepper3.setAcceleration(data[8]);
  stepper4.setAcceleration(data[8]);

  // Just update positions in test mode (no actual movement)
  stepper1.setCurrentPosition(stepper1Position);
  stepper2.setCurrentPosition(stepper2Position);
  stepper3.setCurrentPosition(stepper3Position);
  stepper4.setCurrentPosition(stepper4Position);
  
  delay(100);
  gripperServo.write(data[6]);
  delay(300);
}

void serialFlush() {
  while (Serial.available() > 0) {  //while there are characters in the serial buffer, because Serial.available is >0
    Serial.read();         // get one character
  }
}

// Homing function is skipped in test mode
/*
void homing() {
  // Homing code commented out for test mode
}
*/



