#include <Servo.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>


Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
RF24 radio(48, 49); // CE, CSN

const byte address[6] = "00001";
const int trigPin = 6;
const int echoPin = 5;
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

Servo FL;
Servo FR;
Servo RL;
Servo RR;

void setup() {

  Serial.begin(115200);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  FL.attach(9);
  FR.attach(10);
  RL.attach(11);
  RR.attach(12);
  initSensors();
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
}


void initSensors(){
  if(!accel.begin()){
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin()){
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  if(!bmp.begin())
  {
    Serial.println("Ooops, no BMP180 detected ... Check your wiring!");
    while(1);
  }
}

int input;
int buttons = 0;
int button1 = 0;
int button2 = 0;
int button3 = 0;
int button4 = 0;

int throttle = 0;
int throttleFL = 0;
int throttleFR = 0;
int throttleRL = 0;
int throttleRR = 0;

float accelX;
float accelY;
float accelZ;

float roll;
float pitch;
float heading;

float impliedAltitude;
float absoluteAltitude;

float temperature;

void pollInfo() {
  //controller
  if (radio.available()) {
    input = 0;
    radio.read(&input, sizeof(input));
    
    if ((input & 0b1100000000000000) == 0b0000000000000000) {
      throttle = input & 0b0011111110000000;
      throttle = map(throttle, 8192, 16256, 1000, 2000);
    }
    if ((input & 0b1100000000000000) == 0b0100000000000000) {
      buttons = input & 0b0011110000000000;
      
      if((input & 0b0010000000000000) == 0b0010000000000000){
        button1 = 0;
      }else{
        button1 = 1;
      }
      
      if((input & 0b0001000000000000) == 0b0001000000000000){
        button2 = 0;
      }else{
        button2 = 1;
      }
      
      if((input & 0b0000100000000000) == 0b0000100000000000){
        button3 = 0;
      }else{
        button3 = 1;
      }
      
      if((input & 0b0000010000000000) == 0b0000010000000000){
        button4 = 0;
      }else{
        button4 = 1;
      }
      
    }
  }
  //sensors
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_event_t bmp_event;
  sensors_vec_t   orientation;

  accel.getEvent(&accel_event);
  if (dof.accelGetOrientation(&accel_event, &orientation)){
    accelX = accel_event.acceleration.x;
    accelY = accel_event.acceleration.y;
    accelZ = accel_event.acceleration.z;

    roll = orientation.roll;
    pitch = orientation.pitch;
  }
  mag.getEvent(&mag_event);
  if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation)){
    heading = orientation.heading;
  }
  bmp.getEvent(&bmp_event);
  if (bmp_event.pressure){
    bmp.getTemperature(&temperature);

    impliedAltitude = bmp.pressureToAltitude(seaLevelPressure, bmp_event.pressure, temperature);
  }
  //for ultrasonic sensor
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  absoluteAltitude = pulseIn(echoPin, HIGH)*0.034/0.02;//gets value from sensor, converts to actual altitude



  
  Serial.println(F(""));
  //delay(1000);
}

void printReport(){
  Serial.print("Raw Input: ");
  Serial.println(input, BIN);
  
  Serial.print("Throttle: ");
  Serial.println(throttle);

  Serial.println("Buttons State: ");
  Serial.println(buttons, BIN);
  Serial.println(button1);
  Serial.println(button2);
  Serial.println(button3);
  Serial.println(button4);

  Serial.println("Acceleration: ");
  Serial.print("X: "); Serial.print(accelX); Serial.print("  ");
  Serial.print("Y: "); Serial.print(accelY); Serial.print("  ");
  Serial.print("Z: "); Serial.print(accelZ); Serial.print("  ");Serial.println("m/s^2 \n");
  
  Serial.println("Orientation: ");
  Serial.print(F("Roll: "));
  Serial.print(roll);
  Serial.print(F("; "));
  Serial.print(F("Pitch: "));
  Serial.print(pitch);
  Serial.print(F("; "));

  Serial.print(F("Heading: "));
  Serial.print(heading);
  Serial.println(F("; \n"));

  Serial.print(F("Implied Altitude (from altimeter): "));
  Serial.print(impliedAltitude);
  Serial.println(F(" m; \n"));

  Serial.print(F("Absolute Altitude (from ultrasonic sensor): "));
  Serial.print(absoluteAltitude);
  Serial.println(F(" m; \n"));
    
  Serial.print(F("Temp: "));
  Serial.print(temperature);
  Serial.print(F(" C\n"));

  
  
}

void balanceThrottle(){
  throttleFL = throttle;
  throttleFR = throttle;
  throttleRL = throttle;
  throttleRR = throttle;
}


void control(){
  
}


void sendMotor() {
  FL.write(throttleFL);
  FR.write(throttleFR);
  RL.write(throttleRL);
  RR.write(throttleRR);
}

void loop() {

  pollInfo();
  balanceThrottle();
  control();
  sendMotor();
  printReport();

}
