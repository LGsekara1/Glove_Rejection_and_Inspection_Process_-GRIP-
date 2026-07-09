#include <SoftwareSerial.h>



#define rxPin 3
#define txPin 4

SoftwareSerial ODriveSerial = SoftwareSerial (rxPin,txPin);
//AltSoftSerial ODriveSerial; 

void setup() {
  // pinMode(rxPin,INPUT);
  // pinMode(txPin,OUTPUT);

  Serial.begin(115200);       // USB debug to PC
  ODriveSerial.begin(115200); // to ODrive, must match ODrive's UART baud

  delay(2000); // let ODrive finish booting

  // Enable closed loop control on axis0
  ODriveSerial.println("w axis0.requested_state 8");
  delay(500);

  // Send a position command
  ODriveSerial.println("q 0 1.5 0 0");
}

void loop() {
  // Forward anything ODrive sends back to the PC terminal
  while (ODriveSerial.available()) {
    Serial.write(ODriveSerial.read());
    
  }
}