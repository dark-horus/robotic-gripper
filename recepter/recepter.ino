
#include <PololuMaestro.h>
#include <SoftwareSerial.h>
#include <avr/wdt.h>

#define bluetoothLed 6
#define rbtRxPin 7
#define rbtTxPin 8
#define rxPin 9
#define txPin 10
#define key 11
#define state 12



// Init serial lines
SoftwareSerial bluetooth = SoftwareSerial(rxPin, txPin);
SoftwareSerial maestroSerial = SoftwareSerial(rbtRxPin, rbtTxPin);
MiniMaestro maestro(maestroSerial);


// Init variables
int index = 0;
int bluetoothState = 0;
String message = "";

String servo = "";
int servoInt = 0;
int servoVals[5] = {0, 0, 0, 0};

String command = "";
int commandInt = 0;

bool firstCommand = true;
bool button1State = false;
bool button2State = false;

int counter = 0;


// Debug mode
bool debug = false;


void setup() {

  // Setup pins
  pinMode(rbtRxPin, INPUT);
  pinMode(rbtTxPin, OUTPUT);
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  pinMode(key, OUTPUT);
  pinMode(key, LOW);
  pinMode(bluetoothLed, OUTPUT);
  pinMode(state, INPUT);


  // Console
  Serial.begin(9600);
  Serial.println("Gripper Robot - Recepter");
  Serial.println("------------------------\r\n");


  if (debug)
    Serial.println("DEBUG MODE");


  // Setup Robot
  maestroSerial.begin(57600);
  for (int i = 0; i <= 3; i++)
  {
    maestro.setSpeed(i, 127);
    maestro.setAcceleration(i, 127);
  }


  // Bluetooth connection
  bluetooth.begin(57600);
  digitalWrite(bluetoothLed, LOW);
  if (!debug) {
    bluetoothConnect();
    bluetooth.flush();
    delay(2000);
  }


  // All is fine
  Serial.println("Status OK");


  // Init robot position
  resetPosition();
}


void loop() {

  int commandInput = 0;

  // Manage buttons repetitions
  if (button1State || button2State)
    counter++;
  if (counter > 10) {
    button1State = false;
    button2State = false;
    counter = 0;
  }


  bluetooth.flush();


  // Check Bluetooth state
  if (digitalRead(state) == 0 && !debug) {
    bluetoothConnectionLost();
  } else {
    // Read Bluetooth
    message = bluetooth.readStringUntil('#');

    if (debug)
      Serial.println(message);
  }


  // If message contain commands
  if (message.length() > 0) {


    // Extract commands for each servos
    for (int y = 0; y <= 5; y++) {

      // Servos commands
      if (y >= 0 && y <= 3) {

        index = y * 7; //length of a command for one servo
        servo = message.substring(index, index + 1);
        servoInt = String(servo).toInt();


        // Format command
        command = message.substring(index + 2, index + 6);
        commandInput = String(command).toInt();
      }


      // Buttons
      if (message.substring(30, 31).toInt() == 1 && counter == 0) {
        button1State = true;
        //Serial.println("button 1 pressed");
      }
      if (message.substring(34, 35).toInt() == 1 && counter == 0) {
        button2State = true;
        //Serial.println("button 2 pressed");
      }
      if (button1State && button2State) {
        hardReset();
        break;
      } else if (button1State && !button2State) {
        resetPosition();
        break;
      } else if (button2State && !button1State) {
        break;
      }


      // Map command joystick->robot
      if (servoInt == 0) { //Turn
        commandInt = map(commandInput, -512, 512, 1800, 10000);
      }
      if (servoInt == 1) { // Top Down
        commandInt = map(commandInput, -512, 512, 4500, 8000);
      }
      if (servoInt == 2) { // Front Back
        commandInt = map(commandInput, -512, 512, 6000, 9000);
      }
      if (servoInt == 3) { // Clamp
        commandInt = map(commandInput, -512, 512, 6600, 9000);
      }


      // Init last command
      if (firstCommand) {
        servoVals[servoInt] = commandInt;
        firstCommand = false;
      }


      // Send command only if new command and current are different
      if (servoVals[servoInt] != commandInt && commandInt > 0 && servoInt >= 0 && servoInt <= 3) {

        if (debug) {
          Serial.print(servoInt);
          Serial.print(":");
          Serial.print(commandInt);
          Serial.print(":");
          Serial.println(message);
        }

        // Check Bluetooth state
        if (digitalRead(state) == 0) {
          bluetoothConnectionLost();
        } else {
          // Send to robot
          maestro.setTarget(servoInt, commandInt);
        }

        // Save current command
        servoVals[servoInt] = commandInt;

      }
    }
  }
}


void bluetoothConnect() {

  Serial.print("Bluetooth connection ");


  // Wait for master
  while ( bluetoothState < 10 ) {
    if (digitalRead(state) == 1) {
      bluetoothState++;
      Serial.print(".");
    } else {
      bluetoothState = 0;
      digitalWrite(bluetoothLed, HIGH);
      delay(200);
      digitalWrite(bluetoothLed, LOW);
      delay(100);
    }
    delay(100);
  }
  

  // Connected
  bluetoothState = 0;
  Serial.println("OK");
  digitalWrite(bluetoothLed, HIGH);
  delay(1000);
}


void bluetoothConnectionLost() {

  Serial.println("Bluetooth connection lost");
  bluetoothState = 0;
  digitalWrite(bluetoothLed, LOW);
  bluetoothConnect();
}


void resetPosition() {

  Serial.println("Reset position");
  delay(500);


  // Check Bluetooth state
  if (digitalRead(state) == 0) {
    bluetoothConnectionLost();
  } else {
    // Send to robot initial position: 1410, 1360, 1504, 1965
    if (debug)
      Serial.println("Send robot in initial position");

    maestro.setTarget(0, 5900);
    delay(250);
    maestro.setTarget(1, 6250);
    delay(250);
    maestro.setTarget(2, 6600);
    delay(250);
    maestro.setTarget(3, 7800);
    delay(1000);
    bluetooth.flush();
  }
}


void hardReset()
{

  Serial.println("Hard reset");
  delay(500);


  // Enter reset mode
  int resetPin = 5;
  pinMode(resetPin, OUTPUT);
  digitalWrite(resetPin, HIGH);
}
