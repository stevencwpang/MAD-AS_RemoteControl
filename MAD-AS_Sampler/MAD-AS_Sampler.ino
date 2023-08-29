//__________________________________________________________________________________________
// Include all libraries required to run this program
#include <SoftwareSerial.h>
#include <avr/power.h>
#include <LowPower.h>
#include <MCP7940.h>
#include <EEPROM.h>

//__________________________________________________________________________________________
// microBoSL board pin definition
#define BAUDRATE 9600
//#define HC_TX 10        //HC-12 TX Pin
//#define HC_RX 11        //HC-12 RX Pin
//#define HC_SET 13       //HC-12 Set Pin
#define HallPwrPin 6    //Hall effect sensor Power Pin
#define HallSigPin A0   //Hall effect sensor Signal Pin
#define WakePwrPin 5    //Wake-up hall effect sensor Power Pin
#define WakeSigPin A2   //Wake-up hall effect sensor Signal Pin
#define MotorPowerPin 9 //The pin used to switch the MOSFET to power the motor
#define interruptPin 2  //Attach interrupt pin of the RTC alarm

//__________________________________________________________________________________________
// Configure here: sampling program set-up
#define NumberOfSpins 30        // NumberOfSpins defines the number of pump spins that the user would like to undertake in each sampling cycle. 
                                // If the user would like the pump to continuously run then set to high value like 9999999.
                                // If there is NO Hall effect sensor intalled, then this definition is NOT used. Use the next definition.

//#define DurationOfPumping 10  // DurationOfPumping is only used when there is NO Hall effect sensor. 
                                // It defines the duration of pumping that occurs each time the pump turns on, in seconds.

#define PumpEveryXMins 5        // PumpEveryXMins defines how often the pump is turned on, in minutes. 

#define DurationOfRun 2880      // DurationOfRun defines the total duration of the sampling, in minutes.
                                // e.g. if we run the pump for a day, it will be 1440 minutes.

#define ActivationMethod 0      // 0: sampling starts immediate after calibration; 1: activate program by the Wake-up hall effect sensor; 2: activate program by radio command (not finished).

#define triggering_threshold 30 // Triggering_threshold declares the triggering threshold of Wake-up hall effect sensor. Adjust based on the types of hall effect sensor.
                                // You usually do not need to modify this value.

//__________________________________________________________________________________________
// SoftwareSerial set-up
//SoftwareSerial HC12(HC_TX, HC_RX);    //Radio Module HC-12 

//__________________________________________________________________________________________
// Variables declaration
volatile int WebCmd = 0;                      //Command from the cloud website: 0 means no sampling, 1 means start sampling.
extern volatile unsigned long timer0_millis;  //millis timer variable,used for low power library
const uint8_t  SPRINTF_BUFFER_SIZE = 32;      //Buffer size for sprintf(), used for MCP7940 RTC
double MinVal,MaxVal;       //for hall effect sensor calibration
double MagneticStrength;    //magnetic strength detected by the hall effect sensor
long i,SpinCounter;         //counter for motor spins
long TimeTakenToSpinMe;     //time taken to spin
double DelayTime;           //delay time used only when there is NO hall effect sensor
double HallWakeSig;         //wake-up hall effect sensor signal
int ScanCheck = 1;          //used to check if the number of sampling is reached
uint16_t CycleCounter;      //number of cycles the MAD-AS has performed
uint16_t zero = 0;          //used to reset cycle counter
String RadioMsgBuffer = ""; //message to send through HC12 radio
enum alarmTypes {
  matchSeconds,
  matchMinutes,
  matchHours,
  matchDayOfWeek,
  matchDayOfMonth,
  Unused1,
  Unused2,
  matchAll,
  Unknown
};
MCP7940_Class MCP7940;                           // Create an instance of the MCP7940
char          inputBuffer[SPRINTF_BUFFER_SIZE];  // Buffer for sprintf()/sscanf()
int yr = 2020; int mth = 1; int dy = 1; int hr = 00; int mn = 00; int sc = 00; // Date and time variables

//__________________________________________________________________________________________
// Initialise and set up the sampling program
void setup() {
  Serial.begin(BAUDRATE);             // Serial port to computer
//  if (ActivationMethod == 2){
//    HC12.begin(BAUDRATE);               // Serial port to HC12
//  }

  // Use serial monitor to reset the cycle counter
//  Serial.println(F("----------- Device ID: N00 -----------"));
  Serial.println(F("Enter 'R' to reset the cycle counter."));
  delay(3000);
  if (Serial.available()){
    uint8_t command = Serial.read();
    if(command = 'R'){
      EEPROM.put(0,zero);
      Serial.println(F("The cycle counter has been reset."));
    }
    else{
      Serial.println(F("The cyele counter is not reset."));
    }
  }
  Serial.print(F("Number of cycles performed before this restart: "));
  EEPROM.get(0,CycleCounter);
  Serial.println(CycleCounter);
  

  // Pin mode set up
  pinMode(MotorPowerPin, OUTPUT);
  pinMode(HallPwrPin, OUTPUT);
  pinMode(WakePwrPin, OUTPUT);
  digitalWrite(HallPwrPin,LOW);digitalWrite(WakePwrPin,LOW);                  //turn off hall effect sensor
  pinMode(interruptPin, INPUT_PULLUP);                                        //attach interrupt pin for the RTC alarm
//  if (ActivationMethod == 2){
//    pinMode(HC_SET,OUTPUT);
//    digitalWrite(HC_SET,HIGH);
//    delay(20);
//    HC12Sleep();                                                                //switch HC-12 to low power mode
//  }
  GetMinsMaxs();                                                              //calibrate the hall effect sensor and comment off this line if NO hall effect sensor is used


  // Initialize the Real-Time Clock module to ensure the precise smapling intervals
  while (!MCP7940.begin()) {                                                  // Initialize RTC communications
        Serial.println(F("Unable to find MCP7940N. Checking again in 3s."));  // Show error text
        delay(3000);                                                          // wait three seconds
      }                                                                       // of loop until device is located  
  while (!MCP7940.deviceStatus()) {                                           //Turn oscillator on if necessary
        Serial.println(F("Oscillator is off, turning it on."));
        bool deviceStatus = MCP7940.deviceStart();                            // Start oscillator and return state
     if (!deviceStatus) {                                                     // If it didn't start
        Serial.println(F("Oscillator did not start, trying again."));         // Show error and
        delay(1000);                                                          // wait for a second
     }                                                                        // of if-then oscillator didn't start
  }                                                                           // of while the oscillator is off
  SetClock();       //Set the clock of the RTC


  //MAD-AS waiting for activation
  if (ActivationMethod == 1){
    Serial.println(F("The device now starts to detect the triggering magnet every 10 seconds."));
    ActivateByHallEffectSensor(); //Activate by Hall Effect Senor
  }
  else if (ActivationMethod == 2){
//    Serial.println(F("The device now starts to listen for the radio command."));
//    ActivateByRadioCommand();     //Activate by Radio Command
  }
  else{
    Serial.println(F("The device is set to start sampling program immediately."));
  }
  
  Serial.println(F("The sampling program starts now."));
}

//__________________________________________________________________________________________
// Main sampling program - loop until the end of DurationofRun
void loop() {
  if (ScanCheck <= (DurationOfRun/PumpEveryXMins)){
    SetAlarm();   //set the alarm of next operation
    
    Serial.print(F("Scan Check: "));
    Serial.println(ScanCheck);
    
    //Check any further command (stop sampling command) if radio communication is used
//    if (ActivationMethod == 2){
//      Serial.println(F("Start checking command from the radio..."));
//      HC12WakeUp();
//      CheckRadioCmd();
//      if (WebCmd == 0 || WebCmd == 2){
//      Serial.println(F("Command to stop sampling is received from the radio, sending back the response..."));
//      Respond();
//      HC12Sleep();
//      Serial.println(F("The MAD-AS device now starts sleeping forever."));
//      Sleepy(0); //Sleep forever
//      }
//      HC12Sleep();
//      Serial.println(F("No command is found. Sampling program continues."));
//    }

    
    //Spin the pump the desired number of spins ["NumberOfSpins"] and record the time it took to do so. 
    //If there is no Hall effect sensor, then this line is commented out.
    TimeTakenToSpinMe = SpinMe(NumberOfSpins);

    //Uncomment the next few lines if NO hall effect sensor is used.
    //Turn on the pump by pulling the MotorPowerPin high for the defined duration of pumping.
    //digitalWrite(MotorPowerPin,HIGH);
    //DelayTime = 0;
    //DelayTime = DurationOfPumping *1.0 * 1000;
    //delay(DelayTime);
    //TimeTakenToSpinMe = DelayTime;
    //digitalWrite(MotorPowerPin,LOW);
    
    //Print to the serial information about the time it took to conduct the required number of rotations.
    Serial.print(F("TimeTakenToSpin="));
    Serial.println(TimeTakenToSpinMe);
    
    //Add 1 to the cycle counter
    EEPROM.get(0,CycleCounter);
    CycleCounter = CycleCounter + 1;
    EEPROM.put(0,CycleCounter);

    //Send message out if radio communication is used
//    if (ActivationMethod == 2){
//      RadioMsgBuffer = "C" + String(CycleCounter);
//      RadioMsgBuffer = RadioMsgBuffer + "E";
//      HC12WakeUp();
//      Serial.println(F("Sending radio message..."));
//      long StartTime = millis();
//      while(millis() - StartTime < 4000){  
//        HC12.print(RadioMsgBuffer);
//      }
//      HC12Sleep();
//      Serial.println(F("Message sent."));
//      RadioMsgBuffer = "";
//    }
    
    Serial.println(F("This sampling cycle is finished."));
    attachInterrupt(digitalPinToInterrupt(interruptPin), Scan, LOW);
  }
  else{
    Serial.println(F("Sampling program ended."));
  }
  
  Sleepy(0); //Sleep until next trigger. Set to 0 for sleeping forever. 
}

//__________________________________________________________________________________________
// Looped in attachInterrupt (trigger from the RTC alarm)
void Scan(){
  ScanCheck++;
  detachInterrupt(digitalPinToInterrupt(interruptPin));
}


//__________________________________________________________________________________________
// HC-12 Wake-Up Function
//void HC12WakeUp(){
//  digitalWrite(HC_SET, LOW);
//  delay(200);
//  HC12.print("AT+DEFAULT");
//  delay(200);
//  digitalWrite(HC_SET, HIGH);
//  delay(200);
//}

//__________________________________________________________________________________________
// HC-12 Sleep Function
//void HC12Sleep(){
//  digitalWrite(HC_SET, LOW);
//  delay(200);
//  HC12.print("AT+SLEEP");
//  delay(200);
//  digitalWrite(HC_SET, HIGH);
//  delay(200);
//}

//__________________________________________________________________________________________
// Check command from the radio
//void CheckRadioCmd(){
//  byte x = 0;
//  long StartTime = millis();
//  while( millis() - StartTime < 4000 ){
//      if(HC12.available()){
//        x=HC12.read();
//      }
//      WebCmd=x;
//    }
//}

//__________________________________________________________________________________________
// Respond to say the command is received
//void Respond(){
//  Serial.println(F("Sending response..."));
//  long StartTime = millis();
//  while(millis() - StartTime < 4000){
//        HC12.write(WebCmd);
//    }
//  Serial.println(F("Response sent."));
//}

//__________________________________________________________________________________________
// Activation Method 1: activate by Wake-up Hall Effect Senor
void ActivateByHallEffectSensor(){
  //This function is used to detect the external magnet which will activate the sampling program.
  do {
    //turn on pin!
    Sleepy(10);
    Serial.print(F("Wake run. "));
    digitalWrite(WakePwrPin,HIGH);
    Serial.print(F("Wake Sig: "));
    HallWakeSig = analogRead(WakeSigPin);
    HallWakeSig = HallWakeSig + analogRead(WakeSigPin);
    HallWakeSig = HallWakeSig + analogRead(WakeSigPin);
    HallWakeSig = HallWakeSig + analogRead(WakeSigPin);
    HallWakeSig = HallWakeSig + analogRead(WakeSigPin);
    HallWakeSig = HallWakeSig / 5;
    HallWakeSig = sqrt((HallWakeSig - 512.0)*(HallWakeSig - 512.0)) / (1024)*100;
    Serial.println(HallWakeSig); 
    digitalWrite(WakePwrPin,LOW);
     
  } while (HallWakeSig < triggering_threshold);
}

//__________________________________________________________________________________________
// Activation Method 2: activate by Radio Command
//void ActivateByRadioCommand(){
//  while (WebCmd != 1 && WebCmd != 2){
//    Serial.println(F("Start checking command from the radio..."));
//    HC12WakeUp();
//    CheckRadioCmd();
//    if (WebCmd != 1 && WebCmd != 2){
//      HC12Sleep();
//      Serial.println(F("No start command is given from the radio. Next check starts in 30 seconds."));
//    }else{
//      if (WebCmd == 1){Serial.println(F("Command to start sampling is received from the radio, sending back the response..."));}
//      if (WebCmd == 2){Serial.println(F("Command to sleep forever is received from the radio, sending back the response..."));}
//      Respond();
//      HC12Sleep();
//      if (WebCmd == 2){
//      Serial.println(F("The MAD-AS device now starts sleeping forever."));
//      Sleepy(0); //Sleep forever
//      }
//      break;
//    }
//    Sleepy(30);
//  } //Repeatedly check command from the radio every 30 seconds, until the command is YES.
//}

//__________________________________________________________________________________________
// Hall Effect Sensor calibration for precise spin
void GetMinsMaxs(){
   //This function gets the maximum and minimum values for the hall effect sensor in its current environment as the motor spins around. 
   //These values are then used to determine cut off thresholds during the main program. 
   //This function is only called once, when the unit powers on.
   
   Serial.println(F("Calibrating peristaltic pump..."));   
   digitalWrite(MotorPowerPin,HIGH);      //turn motor on
   digitalWrite(HallPwrPin,HIGH);         //turn hall effect sensor on

   //Create two values, with unrealistic numbers as thresholds.
   MinVal = 99999;
   MaxVal = 0;

   //Loop around and record the minimum and highest values from the Hall effect sensor - do this for around 5 seconds which is usually two or three rotations. 
   //If your pump is slow you may need to increase 5000 in the below for loop to ensure you get a good representation of values to define minimum and maximum.   
   for (i=1;i<5000;i++){
      //Obtain the current reading from the Hall sensor - a read of 512 (in this current program) represents no magentic field presence.
      MagneticStrength = analogRead(HallSigPin)*1.0;
      //Calculate the strength as a percentage of the number of bits possible in this current setup (1024). 
      //As such, the reading is now directionless and is a percentage; 0% indicates that the sensor is unable to measure a magentic field, while 100% indicates the field is stronger than the sensor can read.
      MagneticStrength = sqrt((MagneticStrength - 512.0)*(MagneticStrength - 512.0)) / (1024)*100;
      //Serial.println(MagneticStrength);
     
      //Check if the current value is bigger than what we have seen previously, and if so then reset the maximum to this value, else do nothing.
      if(MagneticStrength > MaxVal){
        MaxVal = MagneticStrength;
      }
      //Check if the current value is lower than what we have seen previously, and if so then reset the minimum to this value, else do nothing.
      if(MagneticStrength < MinVal){
        MinVal = MagneticStrength;
      }
      
      delay(1);     //delay for 1 millisecond
   }
   //Serial.println(MaxVal);
   //Serial.println(MinVal);
   digitalWrite(MotorPowerPin,LOW);   //turn motor off
   digitalWrite(HallPwrPin,LOW);      //turn hall effect sensor off
   Serial.println(F("Finished calibrating peristaltic pump."));
   delay(2000);                       //delay for 2000 milliseconds
   Serial.println(F("The MAD-AS device is initiated. Check if the pump spins twice."));
   SpinMe(2);                         //spin the motor twice to initiate the sampler
   delay(3000);
}

//__________________________________________________________________________________________
// Program for pump operation
long SpinMe(int SpinTimes){
  //This function spins the pump for the desired number of rotations, in this instance an integer value refered to as SpinTimes, sent by the calling function.

  //Record the start time of this spinning process so we can return the time it took us to do this process.This is then used in the main loop to calculate delay times.
  long StartTime;
  StartTime = millis();
  
  //Turn motor and Hall effect sensor on.
  digitalWrite(MotorPowerPin,HIGH);
  digitalWrite(HallPwrPin,HIGH);

  //Conduct a loop until we spin the pump the desired number of times.
  while (SpinCounter <SpinTimes) {
    //Obtain the current reading from the Hall sensor - a read of 512 (in this current program) represents no magentic field presence.
    MagneticStrength = analogRead(HallSigPin)*1.0;
    //Calculate the strength as a percentage of the number of bits possible in this current setup (1024).
    //As such, the reading is now directionless and is a percentage; 0% indicates that the sensor is unable to measure a magentic field, while 100% indicates the field is stronger than the sensor can read.
    MagneticStrength = sqrt((MagneticStrength - 512.0)*(MagneticStrength - 512.0)) / (1024)*100;
    //If the magnetic strength is within 1% of the maximum value then record a spin
    if (MagneticStrength > (MaxVal-1)){
      SpinCounter = SpinCounter + 1;
      //print the number of spins and the strength [in percentage] out to serial
      Serial.print(SpinCounter);
      Serial.print(F(": "));
      Serial.println(MagneticStrength);
 
      abcd: //a point in the code to return to - NB this should be changed to a while or do loop

      //Obtain the current reading from the Hall sensor - a read of 512 (in this current program) represents no magentic field presence.
      MagneticStrength = analogRead(HallSigPin)*1.0;
      //Calculate the strength as a percentage of the number of bits possible in this current setup (1024). 
      //As such, the reading is now directionless and is a percentage; 0% indicates that the sensor is unable to measure a magentic field, while 100% indicates the field is stronger than the sensor can read.
      MagneticStrength = sqrt((MagneticStrength - 512.0)*(MagneticStrength - 512.0)) / (1024)*100;

      //If the magentic strength is greater than 1% from the minimum value then loop until not 
      if (MagneticStrength >(MinVal+1)){
        goto abcd;
      }
    }
    //delay for 1 millisecond
    delay(1);
  }
  
  digitalWrite(MotorPowerPin,LOW);    //turn motor off
  digitalWrite(HallPwrPin,LOW);       //turn sensor off

  SpinCounter = 0;                    //reset the SpinCounter to zero

  return millis() - StartTime;        //return the time [in milliseconds] it took to conduct these spins
}

//__________________________________________________________________________________________
// MCP7940 RTC Set Clock
void SetClock() {
  Serial.println(F("Setting MCP7940M to date and time as programmed..."));
  MCP7940.adjust(DateTime(yr,mth,dy,hr,mn,sc));                                                 //set to library compile Date/Time
  Serial.print(F("Date and time is set to "));
  DateTime now = MCP7940.now();                                                                 //get the current time
  sprintf(inputBuffer, "%04d-%02d-%02d %02d:%02d:%02d", now.year(), now.month(), now.day(),
          now.hour(), now.minute(), now.second());                                              //use sprintf() to pretty print date/time with leading zeroes
  Serial.println(inputBuffer);

  MCP7940.setBattery(true);                                                                     //enable battery backup mode
  if (!MCP7940.getBattery()) {                                                                  //check if successful
    Serial.println(F("Couldn't set Battery Backup, is this a MCP7940N?"));
  }else{                                                                                        //if-then battery mode couldn't be set
    Serial.println(F("The Battery backup has successfully been set up!"));
  }
}

//__________________________________________________________________________________________
// MCP7940 RTC Set Alarm
void SetAlarm() {
  DateTime now = MCP7940.now();  // get the current time
  int nowminute = now.minute();
  int FIRST_INTERVAL = int(PumpEveryXMins); 
  Serial.print(F("Setting alarm 1 to go off at "));
  now = now + TimeSpan(0, 0, FIRST_INTERVAL, 0);  // Add interval to current time
  sprintf(inputBuffer, "%04d-%02d-%02d %02d:%02d:%02d", now.year(), now.month(), now.day(),
          now.hour(), now.minute(), now.second());
  Serial.println(inputBuffer);
  MCP7940.setAlarm(1, matchAll, now, true);  // Set alarm to go off then
  delay(10);
}

//__________________________________________________________________________________________
// Low Power Function
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
