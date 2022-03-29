//This program is used to check the command on the cloud website and communicate the command to the nearby device through radio modules.

//Include all necessary libraries:
#include <SoftwareSerial.h>
#include <avr/power.h>
#include <LowPower.h>

//Device specific config
#define DEVICEID "MAD-AS Testing"

//For BoSL/Arduino Board
#define BAUDRATE 9600
#define HC_TX 7 //HC-12 TX Pin
#define HC_RX 6 //HC-12 RX Pin
#define HC_SET 9 //HC-12 Set Pin

//SoftwareSerial
SoftwareSerial HC12(HC_TX, HC_RX); //Radio Module HC-12

//Variables Declaration
volatile int WebCmd = 0; //Command from the cloud website: 0 means no sampling, 1 means start sampling.
volatile int DeviceWakeUp = 0; //If the nearby device receives wake-up command (start sampling): 0 means no response from nearby device, 1 means response given by nearby device.
extern volatile unsigned long timer0_millis; //millis timer variable 


void setup() {
  Serial.begin(BAUDRATE);             // Serial port to computer
  HC12.begin(BAUDRATE);               // Serial port to HC12

  //Pin Mode Set Up
  pinMode(HC_SET,OUTPUT);
  digitalWrite(HC_SET,HIGH);
  delay(20);
  HC12Sleep();   
}

void loop() {
  //Repeatedly check command from the cloud every ten seconds, until the command is YES.
  while (WebCmd == 0){
    Serial.println("Start checking command from the cloud...");
    CheckWebCmd();
    if (WebCmd == 0){
      Serial.println("No start command is given from the cloud. Next check starts in 10 seconds.");
      Sleepy(10);
    }else{
      Serial.println("Command to start sampling is received from the cloud.");
      HC12WakeUp();
      WakeUp();
      if (DeviceWakeUp != 0){
        Serial.println("The nearby device has received the command and its response is now captured.");
        Serial.println("This program ends.");
        HC12Sleep();
        Sleepy(0);
      }
    }
  }
}



//Check command from the cloud
void CheckWebCmd(){
  ////////////////////////////////////////////
  delay(5000);
  WebCmd = 1;
}

//Wake up the nearby device to start sampling
void WakeUp(){
  byte x;
  while (DeviceWakeUp == 0){
    //Send command to the nearby device for 2 seconds
    Serial.println("Sending command over radio...");
    long StartTime = millis();
    while(millis() - StartTime < 1000){
        HC12.write(WebCmd);
    }
    //Listen from the nearby device for 2 seconds
    Serial.println("Listening for response over radio...");
    StartTime = millis();
    while( millis() - StartTime < 1000 ){
      if(HC12.available()){
        x=HC12.read();
      }
    }
    if(x==1){
      DeviceWakeUp = 1;
    }
  }
}

void HC12WakeUp(){
  digitalWrite(HC_SET, LOW);
  delay(200);
  HC12.print("AT+DEFAULT");
  delay(200);
  digitalWrite(HC_SET, HIGH);
  delay(200);
}

void HC12Sleep(){
  digitalWrite(HC_SET, LOW);
  delay(200);
  HC12.print("AT+SLEEP");
  delay(200);
  digitalWrite(HC_SET, HIGH);
  delay(200);
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
