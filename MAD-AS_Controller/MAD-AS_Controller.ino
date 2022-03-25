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

//SoftwareSerial
SoftwareSerial HC12(HC_TX, HC_RX); //Radio Module HC-12

//Variables Declaration
volatile int WebCmd = 0; //Command from the cloud website: 0 means no sampling, 1 means start sampling.
volatile int DeviceWakeUp = 0; //If the nearby device receives wake-up command (start sampling): 0 means no response from nearby device, 1 means response given by nearby device.
extern volatile unsigned long timer0_millis; //millis timer variable 

void setup() {
  Serial.begin(BAUDRATE);             // Serial port to computer
  HC12.begin(BAUDRATE);               // Serial port to HC12
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
      WakeUp();
      if (DeviceWakeUp != 0){
        Serial.println("The nearby device has received the command and its response is now capture.");
        Serial.println("This program ends.");
        Sleepy(0);
      }else{
        Serial.println("Something went wrong if you see this message!!!!!");
        Serial.println("This program ends.");
        Sleepy(0);
      }
    }
  }
}



//Check command from the cloud
void CheckWebCmd(){
  ////////////////////////////////////////////
  WebCmd = 1;
}

//Wake up the nearby device to start sampling
void WakeUp(){
  int i,j;
  byte x;
  while (DeviceWakeUp == 0){
    //Send command to the nearby device for 2 seconds
    Serial.println("Sending command over radio...");
    for (i=1;i<2000;i++){
      HC12.write(WebCmd);
      delay(1);
    }
    //Listen from the nearby device for 2 seconds
    Serial.println("Listening for response over radio...");
    for (j=1;j<2000;j++){
      x = HC12.read();
      delay(1);
    }
    if(x==1){
      DeviceWakeUp = 1;
    }
  }
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
