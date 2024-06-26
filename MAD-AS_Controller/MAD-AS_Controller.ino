//This program is used to check the command on the cloud website and communicate the command to the nearby device through radio modules.

//////////////////////////Include all necessary libraries//////////////////////////
#include <SoftwareSerial.h>
#include <avr/power.h>
#include <LowPower.h>




//////////////////////////////Device specific config//////////////////////////////
#define SITEID "Trial"




//////////////////////////////////For BoSL Board//////////////////////////////////
#define SIMCOM_7000           //SIM7000A/C/E/G
#define BAUDRATE 9600
#define HC_TX 7               //HC-12 TX Pin
#define HC_RX 8               //HC-12 RX Pin
#define HC_SET 9              //HC-12 Set Pin
#define PWRKEY 4
#define DTR 5                 //Connect with solder jumper
#define BOSL_RX 3             //Microcontroller RX
#define BOSL_TX 2             //Microcontroller TX
#define CHARBUFF 196          //SIM7000 serial response buffer; longer than 255 will cause issues




//////////////////////////////////SoftwareSerial//////////////////////////////////
SoftwareSerial simCom = SoftwareSerial(BOSL_RX, BOSL_TX);     //SIM Card Module
SoftwareSerial HC12(HC_TX, HC_RX);                            //Radio Module HC-12




///////////////////////////////Variables Declaration///////////////////////////////
volatile int WebCmd = 0;                      
//Command from the cloud website: 0 means no sampling, 1 means start sampling, 2 means stop sampling.
volatile int DeviceWakeUp = 0;                
//If the nearby device receives wake-up command (start sampling): 0 means no response from nearby device, 1 means response given by nearby device.
int SendOrNot = 0;                           
 //Send data to the cloud or not.
int CycleCounter = -1;  String CycleCounterTemp;                     
//Number of Cycles the MAD-AS device has performed. 
int CmdRecCounter = 0; String CmdRecCounterTemp;
//Number of Commands receieved by MAD-AS controller. This is uesed to understand where does the lost of communication happen.
int ErrorCode = -1; String ErrorCodeTemp;
//0 means normal, -1 means the MAD-AS is waiting for start command, -2 means the MAD-AS stops sampling according to the stop command, 
//-3 and -4 means the controller cannot hear from the sampler and time-out is triggered (-3: during sampling stage, -4: during waking-up stage).
int SleepForever = 0;                         
//Used in the mechanism which makes the controller sleep forever: 0 means no, 1 means the controller will sleep forever.
byte incomingByte;                            
//Incoming Byte over the radio
String RadioMsgBuffer = "";                   
//Message received over radio after MAD-AS starts sampling. 
extern volatile unsigned long timer0_millis;  
//millis timer variable,used for low power library
char response[CHARBUFF];                      
//sim7000 serial response buffer
String dataStr;                               
//Transmit URL
int SIM_count = 0;
//I am not sure what does this mean? -----------------------------canwei




//////////////////////////////////Setup Program//////////////////////////////////
void setup() {
  Serial.begin(BAUDRATE);             // Serial port to computer
  HC12.begin(BAUDRATE);               // Serial port to HC12
  simCom.begin(BAUDRATE);             // Serial port to SIM card


  /////////////////////Pin Mode Set Up/////////////////////
  pinMode(HC_SET,OUTPUT);
  digitalWrite(HC_SET,HIGH);
  delay(20);
  HC12Sleep();
  simOff();
  Serial.println(F("The controller program has been set up."));
}




////////////////////////////////////Main Program////////////////////////////////////
void loop() {
  while (WebCmd == 0 && DeviceWakeUp == 0 && SleepForever == 0){
    //Repeatedly check command from the cloud every 6 minutes, until the command is YES.
    Serial.println(F("Start checking command from the cloud..."));
    CheckWebCmd();
    if (WebCmd != 1 && WebCmd != 2){ //Command 0 or any other number will be treated as no sampling command.
      Serial.println(F("No start command is given from the cloud. Next check starts in 600 seconds."));
      Sleepy(600);
    }else{
      if (WebCmd == 1){Serial.println(F("Command to start sampling is received from the cloud."));}
      if (WebCmd == 2){Serial.println(F("Command to sleep forever is received from the cloud."));}
      HC12WakeUp();
      WakeUp();
      HC12Sleep();
      if (DeviceWakeUp == 1){
        Serial.println(F("The nearby device has received the command and its response is now captured."));
      }
    }
  }

  while (WebCmd == 1 && DeviceWakeUp == 1 && SleepForever == 0){  
    //Turn on the radio and listen for message from the MAD-AS device, send message to the cloud if message is heard.
    Serial.println(F("Turning on radio to hear from the nearby MAD-AS device."));
    HC12WakeUp();
    HC12Listen();
    HC12Sleep();
    CheckMsgBuffer();
    Serial.print(F("Number of cycles is "));
    Serial.println(CycleCounter);
    if (SendOrNot == 1){
      Serial.println(F("Sending cycle number to the cloud..."));
      delay(5000);
      simOn();
      Transmit();
      simOff();
      Serial.println(F("Done!"));
      SendOrNot = 0;
    }
    RadioMsgBuffer = "";
  }

  while ((WebCmd == 0 || WebCmd == 2) && DeviceWakeUp == 1 && SleepForever == 0){
    //Ask the MAD-AS Sampler to stop sampling.
    Serial.println(F("Asking MAD-AS Sampler to stop sampling..."));
    StopSampling();
    Serial.println(F("The nearby MAD-AS Sampler has stopped sampling."));
  }

  while (SleepForever == 1){
    //MAD-AS controller will sleep forever, after sending the last message to the cloud (to tell the reason).
    Serial.println(F("This MAD-AS Controller is set to sleep forever soon."));
    Serial.println(F("Sending the last message..."));
    delay(5000);
    simOn();
    Transmit();
    simOff();
    Serial.println(F("Goodbye!"));
    Sleepy(0);
  }
}




////////////////////////////Check command from the cloud////////////////////////////
void CheckWebCmd(){
  Serial.println(F("Initialising SIM 7000"));
  delay(5000);

  simOn();
  if(SIM_count == 0){
    simInit();
  }else{
    netUnreg();
  }

  Transmit(); 
  simOff();
}




///////////////////////Wake up the nearby device to start sampling///////////////////////
void WakeUp(){
  byte x = 0;
  byte i = 0;
  while (DeviceWakeUp == 0 && i < 3600){
    //Send command to the nearby device for 1 seconds
    Serial.println(F("Sending command over radio..."));
    long StartTime = millis();
    while(millis() - StartTime < 1000){
        HC12.write(WebCmd);
    }
    //Listen from the nearby device for 1 seconds
    Serial.println(F("Listening for response over radio..."));
    StartTime = millis();
    while( millis() - StartTime < 1000 ){
      if(HC12.available()){
        x=HC12.read();
      }
    }
    if(x == 1){
      DeviceWakeUp = 1;
      ErrorCode = 1;
    }
    if(x == 2){
      DeviceWakeUp = 1;
      ErrorCode = -2;
      SleepForever = 1;
    }
    i = i + 1; // i = 1800 = 1 hour
  }
  if (DeviceWakeUp == 0 && i >= 3600){
    //stop listening if no handshake is done in 2 hours.
    Serial.println(F("No radio signal is received for 2 hours. This device will turn off soon."));
    ErrorCode = -4;
    SleepForever = 1;
  }
}




///////////////////////////////HC-12 Wake-Up Function////////////////////////////////
void HC12WakeUp(){
  simCom.flush();
  simCom.end();
  HC12.listen();
  digitalWrite(HC_SET, LOW);
  delay(200);
  HC12.print("AT+DEFAULT");
  delay(200);
  while(HC12.available()){
    HC12.read();
  }
  digitalWrite(HC_SET, HIGH);
  delay(200);
}




///////////////////////////////HC-12 Sleep Function////////////////////////////////
void HC12Sleep(){
  digitalWrite(HC_SET, LOW);
  delay(200);
  HC12.print("AT+SLEEP");
  delay(200);
  while(HC12.available()){
    HC12.read();
  }
  digitalWrite(HC_SET, HIGH);
  delay(200);  
  HC12.flush();
  HC12.end();
  simCom.listen();
}




///////////////////////////////Listen for HC-12 when MAD-AS is sampling///////////////////////////////
void HC12Listen(){
  long StartTime = millis();
  Serial.println(F("Listening for message over radio..."));
  while( RadioMsgBuffer.length() < 15 && millis() - StartTime < 7200000){
    if(HC12.available()){
      incomingByte = HC12.read();
      RadioMsgBuffer += char(incomingByte);
    }
  }
  if(millis() - StartTime >= 7200000){
    //stop listening if no signal is detected in 2 hours
    Serial.println(F("No radio signal is received for 2 hours. This device will turn off soon."));
    ErrorCode = -3;
    SleepForever = 1;    
  }
}




///////////////////////////////Ask to stop sampling over HC-12///////////////////////////
void StopSampling(){
  HC12WakeUp();
  byte x = -1;
  while (DeviceWakeUp == 1){
    //Send command to the nearby device for 1 seconds
    Serial.println(F("Sending command over radio..."));
    long StartTime = millis();
    while(millis() - StartTime < 1000){
        HC12.write(WebCmd);
    }
    //Listen from the nearby device for 1 seconds
    Serial.println(F("Listening for response over radio..."));
    StartTime = millis();
    while( millis() - StartTime < 1000 ){
      if(HC12.available()){
        x=HC12.read();
      }
    }
    if(x==0 || x==2){
      DeviceWakeUp = 0;
      ErrorCode = -2;
      SleepForever = 1;
    }
  }
  HC12Sleep();
}




/////////////////////////////Extract information from the message buffer//////////////////////////
void CheckMsgBuffer(){
  int indexOfC = RadioMsgBuffer.indexOf('C');
  int indexOfE = RadioMsgBuffer.indexOf('E',indexOfC);
  if (indexOfC == -1){
    Serial.println(F("No message from the nearby MAD-AS device."));
  }else{
    int indexOfNumber;
    String TempCycleCounter = "";
    indexOfNumber = indexOfC + 1;
    TempCycleCounter = RadioMsgBuffer.substring(indexOfNumber,indexOfE);
    CycleCounter = TempCycleCounter.toInt();
    SendOrNot = 1;
    CmdRecCounter = CmdRecCounter + 1;
  }
}




///////////////////////////////power off SIM7000///////////////////////////////
void simOff() {
  //Turn off pins to save power.
  digitalWrite(BOSL_TX, LOW);
  digitalWrite(BOSL_RX, LOW);
  digitalWrite(PWRKEY, LOW);

  //See spec sheets for your particular module.
  xDelay(1200); // For SIM7000
  digitalWrite(PWRKEY, HIGH);
  xDelay(2000);
}




///////////////////////////////power on SIM7000///////////////////////////////
void simOn() {
  pinMode(PWRKEY, OUTPUT);
  pinMode(BOSL_TX, OUTPUT);
  digitalWrite(BOSL_TX, HIGH);
  pinMode(BOSL_RX, INPUT_PULLUP);

  digitalWrite(PWRKEY, LOW);
  // See spec sheets for your particular module
  xDelay(1000); // For SIM7000
  digitalWrite(PWRKEY, HIGH);
  xDelay(4000);
}




/////////////////////////////////initialise sim/////////////////////////////////
void simInit(){
   sendATcmd(F("AT+IPR=9600"),"OK",1000);   
   sendATcmd(F("ATE0"),"OK",1000);
   sendATcmd(F("AT&W0"),"OK",1000);
}




//////////////////////////power down cellular functionality//////////////////////////
void netUnreg(){
    sendATcmd(F("AT+CFUN=0"), "OK", 1000);
}




//////////////////////////TRANSMITS STATUS DATA BACK//////////////////////////
void Transmit(){

    String MyResponse;
    CycleCounterTemp = CycleCounter;
    ErrorCodeTemp = ErrorCode;
    CmdRecCounterTemp = CmdRecCounter;
    
    dataStr = "AT+HTTPPARA=\"URL\",\"www.bosl.com.au/IoT/LMP/scripts/WriteMe.php?SiteName=";
    dataStr += SITEID;
    dataStr += ".csv";
    dataStr += "&T=";
    dataStr += CycleCounterTemp;
    dataStr += "&EC=";
    dataStr += ErrorCodeTemp;
    dataStr += "&D=";
    dataStr += CmdRecCounterTemp;
    dataStr += "\"";

    netReg();
    //close open bearer
    sendATcmd(F("AT+SAPBR=2,1"), "OK",1000);
    if (strstr(response, "1,1") == NULL){
        if (strstr(response, "1,3") == NULL){
        sendATcmd(F("AT+SAPBR=0,1"), "OK",1000);
        }
        sendATcmd(F("AT+SAPBR=3,1,\"APN\",\"simbase\""), "OK",1000); //set bearer apn
        sendATcmd(F("AT+SAPBR=1,1"), "OK",1000);
    }
    
    sendATcmd(F("AT+HTTPINIT"), "OK",1000);
    sendATcmd(F("AT+HTTPPARA=\"CID\",1"), "OK",1000);
   
    sendATcmd(dataStr, "OK",1000);
   
   sendATcmd(F("AT+HTTPACTION=0"), "200",2000);

   //read the Sampling mode to the bosl
   sendATcmd(F("AT+HTTPREAD"), "OK",2000);
   MyResponse = "";
   MyResponse.concat(response);
   
    sendATcmd(F("AT+HTTPTERM"), "OK",1000);
  //close the bearer connection
    sendATcmd(F("AT+SAPBR=0,1"), "OK",1000);
    
    netUnreg();

    //PRINT DATA
    String VariableData;
  
    VariableData = FindVariable(MyResponse, "1", "OK");
    if(VariableData!="" & (VariableData=="1" || VariableData=="0" || VariableData=="2")){
      //OK, we are in a position to allow variable data to overwrite the logged datasets
      WebCmd = VariableData.toInt();
    }

    Serial.print(F("Sampling Command: "));
    Serial.println(WebCmd);
    delay(100);
    
}




////////////////////////////register to network////////////////////////////
void netReg(){
     sendATcmd(F("AT+CFUN=1"), "OK", 1000);//enable the modem
   
     xDelay(2000);
     sendATcmd(F("AT+CGDCONT=1,\"IP\",\"simbase\""), "OK", 2000);//set IPv4 and apn
     if(sendATcmd(F("AT+CREG?"), "+CREG: 0,5", 2000) == 0){//check that modem is registered as roaming
         sendATcmd(F("AT+COPS=1,2,50501"), "OK",50000); //register to telstra network
         sendATcmd(F("AT+CREG?"), "+CREG: 0,5", 2000); //check that modem is registered as roaming
     }
}







///////////////////sends at command, checks for reply///////////////////
bool sendATcmd(String ATcommand, char* expctAns, unsigned int timeout){
    uint32_t timeStart;
    bool answer;
    uint8_t a=0;
    
    do{a++;
    
    Serial.println(ATcommand);
    
    answer=0;
    
    timeStart = 0;


    delay(100);

    while( simCom.available() > 0) {
        simCom.read();    // Clean the input buffer
    }
    
    simCom.println(ATcommand);    // Send the AT command 


    uint8_t i = 0;
    timeStart = millis();
    memset(response, '\0', CHARBUFF);    // Initialize the string

    // this loop waits for the answer

    do{
        if(simCom.available() != 0){    
            response[i] = simCom.read();
            i++;
            // check if the desired answer is in the response of the module
            if (strstr(response, expctAns) != NULL)    
            {
                answer = 1;
            }
        }    
            
            
        
        // Waits for the asnwer with time out
    }
    while((answer == 0) && ((millis() - timeStart) < timeout)); 

    if (expctAns == "0"){
                answer = 1;
            }
    Serial.println(response);
    
    }while(answer == 0 && a < 5);
    
     a = 0;
     return answer;
}




///////////////////////////////Low Power Function////////////////////////////////
void Sleepy(double ScanInterval){ //Sleep Time in seconds

  simCom.flush(); // must run before going to sleep
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




//////////////////////////////////xDelay Function//////////////////////////////////
void xDelay(uint32_t tmz){
  uint32_t tmzslc = tmz/64;
  //64 chosen as ballance between power savings and time spent in full clock mode
  clock_prescale_set(clock_div_64);
    delay(tmzslc);
  clock_prescale_set(clock_div_1);
  
  cli();
  timer0_millis += 63*tmzslc; 
  sei();
  
  delay(tmz-64*tmzslc);
}




//////////////////////////////////FindVarible Function//////////////////////////////////
String FindVariable(String HayStack, String StartTerm, String EndTerm){
  long Start, End;
  Start = HayStack.indexOf(StartTerm);
  Start = Start + StartTerm.length();
  End = HayStack.indexOf(EndTerm);
  End = End - 1;
  int i;
  String MalcomInTheMiddle;

  if(End<=0 || (Start + 1) < StartTerm.length()){
    //Do nothing- means either the start or the end was not found...
    MalcomInTheMiddle="";
  }else if(End<Start){
    //do nothing- means that some weird stuff is going on!
    MalcomInTheMiddle="";
  } else{
    //ok, seems things are looking up! we have found a string of some description!
    for(i=Start;i<=End;i++){
      if(HayStack[i]!=10 && HayStack[i]!=13){
        MalcomInTheMiddle.concat(HayStack[i]);  
      }  
    }
  }
  return MalcomInTheMiddle;
}
