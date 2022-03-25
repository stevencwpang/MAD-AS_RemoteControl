//This program is used to check the command over the radio module and respond with a message.

//Include all necessary libraries:
#include <SoftwareSerial.h>
#include <avr/power.h>
#include <LowPower.h>

//For BoSL/Arduino Board
#define BAUDRATE 9600
#define HC_TX 7 //HC-12 TX Pin
#define HC_RX 6 //HC-12 RX Pin

//SoftwareSerial
SoftwareSerial HC12(HC_TX, HC_RX); //Radio Module HC-12

//Variables Declaration
volatile int WebCmd = 0; //Command from the cloud website: 0 means no sampling, 1 means start sampling.
extern volatile unsigned long timer0_millis; //millis timer variable 

void setup() {
  Serial.begin(BAUDRATE);             // Serial port to computer
  HC12.begin(BAUDRATE);               // Serial port to HC12
}

void loop() {
  //Repeatedly check command from the radio every 10 seconds, until the command is YES.
  while (WebCmd == 0){
    Serial.println("Start checking command from the radio...");
    CheckRadioCmd();
    if (WebCmd == 0){
      Serial.println("No start command is given from the radio. Next check starts in 10 seconds.");
    }else{
      Serial.println("Command to start sampling is received from the radio.");
      Respond();
      break;
    }
    Sleepy(10);
  }
  Serial.println("can start the rest of program now...");
  Sleepy(0);
}

//Check command from the radio
void CheckRadioCmd(){
  int j;
  byte x;
  for (j=1;j<2000;j++){
    x = HC12.read();
    delay(1);
  }
  if(x==1){
    WebCmd = 1;
  }
}

//Respond to say the command is received
void Respond(){
  int i;
  Serial.println("Sending response...");
  for (i=1;i<2000;i++){
    HC12.write(WebCmd);
    delay(1);
  }
  Serial.println("Response sent.");
}

//Low power function
void Sleepy(double ScanInterval){ //Sleep Time in seconds
  //simCom.flush(); // must run before going to sleep
  Serial.flush(); // ensures that all messages have sent through serial before arduino sleeps

  if(ScanInterval>0){
    while(ScanInterval >= 1){
      LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF); //8 seconds dosen't work on the 8mhz
      //advance millis timer as it is paused in sleep
      noInterrupts();
      timer0_millis += 1;
      interrupts();
      ScanInterval -= 1;
    }
  }else{
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  }
}
