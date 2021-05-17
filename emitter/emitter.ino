
#include <SoftwareSerial.h>

#define rxPin 9
#define txPin 10
#define key 11
#define state 12
#define bluetoothLed 13


// Init serial line
SoftwareSerial bluetooth = SoftwareSerial(rxPin, txPin);


// Init variables
char message;
char command[38];
int bluetoothState = 0;

int joystick_1_button = 7;
int joystick_1_axis_x = A0;
int joystick_1_axis_y = A1;

int joystick_2_button = 8;
int joystick_2_axis_x = A2;
int joystick_2_axis_y = A3;

int jx1 = 0;
int jy1 = 0;
int jx2 = 0;
int jy2 = 0;
int button1 = 0;
int button2 = 0;

int jx1_zero_point = 0;
int jy1_zero_point = 0;
int jx2_zero_point = 0;
int jy2_zero_point = 0;

int diff = 0;
int limitDiff = 127;
int resolution = 10;

bool send = false;
bool zeroPointDefined = false;


// Debug mode
bool debug = false;


void setup() {

  // Setup pins
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  pinMode(key, OUTPUT);
  pinMode(key, LOW);
  pinMode(state, INPUT);
  pinMode(bluetoothLed, OUTPUT);
  pinMode(joystick_1_button, INPUT);
  pinMode(joystick_2_button, INPUT);


  // INPUT_PULLUP
  digitalWrite(joystick_1_button, HIGH);
  digitalWrite(joystick_2_button, HIGH);


  // Console
  Serial.begin(9600);
  Serial.println("Gripper Robot - Emitter");
  Serial.println("-----------------------\r\n");


  if (debug)
    Serial.println("DEBUG MODE");


  // Setup zero point for joysticks
  Serial.print("Define the zero points of joysticks ");
  for (int i = 0; i <= 3; i++) {
    Serial.print(".");
    jx1_zero_point += analogRead(joystick_1_axis_x);
    jy1_zero_point += analogRead(joystick_1_axis_y);
    jx2_zero_point += analogRead(joystick_2_axis_x);
    jy2_zero_point += analogRead(joystick_2_axis_y);
  }
  jx1_zero_point = (unsigned int)(jx1_zero_point / 4);
  jy1_zero_point = (unsigned int)(jy1_zero_point / 4);
  jx2_zero_point = (unsigned int)(jx2_zero_point / 4);
  jy2_zero_point = (unsigned int)(jy2_zero_point / 4);


  if (debug) {
    Serial.println("Result:");
    Serial.println(String("0:" + String(jx1_zero_point) + "|1:" + String(jy1_zero_point) + "|2:" + String(jx2_zero_point) +  "|3:" + String(jy2_zero_point) ) );
  }

  Serial.println("OK");


  // Configure bluetooth dongle
  // AT MODE ON
  pinMode(key, HIGH);
  bluetooth.begin(57600);
  bluetooth.print("AT+POWE2\r\n");
  delay(500);
  bluetooth.print("AT+NAMEARDBT01\r\n");
  delay(500);
  bluetooth.print("AT+ROLE1\r\n");
  delay(500);
  bluetooth.print("AT+PIN964277\r\n");
  delay(500);
  bluetooth.print("AT+CONAD43639C79AA6\r\n");
  //delay(2000); // wait for recepter
  // AT MODE OFF
  pinMode(key, LOW);


  // Bluetooth connection
  digitalWrite(bluetoothLed, LOW);
  if (!debug)
    bluetoothConnect();
  bluetooth.flush();


  // All is fine
  Serial.println("Status OK");
}


void loop() {

  // Read joysticks
  int jx1_new = analogRead(joystick_1_axis_x);
  int jy1_new = analogRead(joystick_1_axis_y);
  int jx2_new = analogRead(joystick_2_axis_x);
  int jy2_new = analogRead(joystick_2_axis_y);
  button1 = !digitalRead(joystick_1_button);
  button2 = !digitalRead(joystick_2_button);


  // Joystick X1
  diff = jx1_zero_point - jx1_new;
  if (diff < 0 && abs(diff) > 10 ) {
    jx1 -= abs(diff) / resolution;
  }
  if (diff > 0 && abs(diff) > 10 ) {
    jx1 += abs(diff) / resolution;
  }
  jx1 = constrain(jx1, -512, 512);

  // Joystick Y1
  diff = jy1_zero_point - jy1_new;
  if (diff < 0 && abs(diff) > 10 ) {
    jy1 -= abs(diff) / resolution;
  }
  if (diff > 0 && abs(diff) > 10 ) {
    jy1 += abs(diff) / resolution;
  }
  jy1 = constrain(jy1, -512, 512);

  // Joystick X2
  diff = jx2_zero_point - jx2_new;
  if (diff < 0 && abs(diff) > 10 ) {
    jx2 -= abs(diff) / resolution;
  }
  if (diff > 0 && abs(diff) > 10 ) {
    jx2 += abs(diff) / resolution;
  }
  jx2 = constrain(jx2, -512, 512);

  // Joystick Y2
  int diff = jy2_zero_point - jy2_new;
  if (diff < 0 && abs(diff) > 10 ) {
    jy2 -= abs(diff) / (resolution / 4);
  }
  if (diff > 0 && abs(diff) > 10 ) {
    jy2 += abs(diff) / (resolution / 4);
  }
  jy2 = constrain(jy2, -512, 512);


  // Format command
  sprintf(command, "0:%04d|1:%04d|2:%04d|3:%04d|4:%01d|5:%01d#", jy1, jx1, jx2, jy2, button1, button2);


  // Check Bluetooth state
  if (digitalRead(state) == 0 && !debug) {
    bluetoothConnectionLost();
  } else {
    // Send by bluetooth
    bluetooth.flush();
    if (!debug) {
      bluetooth.print(command);
    } else {
      Serial.println(command);
    }
  }

  delay(50);

}

void bluetoothConnect() {

  Serial.print("Bluetooth connection ");

  // AT MODE ON
  pinMode(key, HIGH);
  delay(500);

  // Wait for slave
  bluetooth.print("AT+CONAD43639C79AA6\r\n");
  delay(2000);

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


  // AT MODE OFF
  pinMode(key, LOW);
  delay(500);
}

void bluetoothConnectionLost() {

  Serial.println("Bluetooth connection lost");
  bluetoothState = 0;
  digitalWrite(bluetoothLed, LOW);
  bluetoothConnect();
}
