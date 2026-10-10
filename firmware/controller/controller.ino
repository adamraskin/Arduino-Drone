#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";
void setup() {
  Serial.begin(9600);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
  Serial.println("ready");
}
int stickL=0;
int buttons=0;
void loop() {
  
  stickL = 0b00;
  stickL = stickL << 7;
  stickL += analogRead(A0)/8;
  stickL = stickL << 7;
  stickL += analogRead(A1)/8;
  
  buttons = 0b01;
  buttons = buttons << 1;
  buttons += digitalRead(6);
  buttons = buttons << 1;
  buttons += digitalRead(3);
  buttons = buttons << 1;
  buttons += digitalRead(4);
  buttons = buttons << 1;
  buttons += digitalRead(5);
  buttons = buttons << 10;
  buttons += 0b0000000000;
  
  Serial.print("Joystick X: ");
  Serial.println(analogRead(A0)/8, BIN);
  Serial.print("Joystick Y: ");
  Serial.println(analogRead(A1)/8, BIN);
  Serial.print("Joystick Button: ");
  Serial.println(digitalRead(6), BIN);
  Serial.print("Button 1: ");
  Serial.println(digitalRead(3), BIN);
  Serial.print("Button 2: ");
  Serial.println(digitalRead(4), BIN);
  Serial.print("Button 3: ");
  Serial.println(digitalRead(5), BIN);
  Serial.print("Left Stick State: ");
  Serial.println(stickL, BIN);
  Serial.print("Buttons State: ");
  Serial.println(buttons, BIN);
  
  radio.write(&stickL, sizeof(stickL));
  radio.write(&buttons, sizeof(buttons));
  //delay(1000);
}
