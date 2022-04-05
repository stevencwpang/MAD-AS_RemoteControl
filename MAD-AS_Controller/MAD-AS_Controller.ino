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
volatile int WebCmd = 0;                      //Command from the cloud website: 0 means no sampling, 1 means start sampling.
volatile int DeviceWakeUp = 0;                //If the nearby device receives wake-up command (start sampling): 0 means no response from nearby device, 1 means response given by nearby device.
volatile int CYCLE = 1;
volatile int RESTART = 2;
volatile int ACTIVATION = 3;
extern volatile unsigned long timer0_millis;  //millis timer variable,used for low power library
char response[CHARBUFF];                      //sim7000 serial response buffer
String dataStr;                               //Transmit URL
int SIM_count = 0;




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
}




////////////////////////////////////Main Program////////////////////////////////////
void loop() {
  //Repeatedly check command from the cloud every ten seconds, until the command is YES.
  while (WebCmd == 0){
    Serial.println("Start checking command from the cloud...");
    CheckWebCmd();
    if (WebCmd == 0){
      Serial.println("No start command is given from the cloud. Next check starts in 30 seconds.");
      Sleepy(30);
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




///////////////////////////////HC-12 Wake-Up Function////////////////////////////////
void HC12WakeUp(){
  digitalWrite(HC_SET, LOW);
  delay(200);
  HC12.print("AT+DEFAULT");
  delay(200);
  digitalWrite(HC_SET, HIGH);
  delay(200);
}




///////////////////////////////HC-12 Sleep Function////////////////////////////////
void HC12Sleep(){
  digitalWrite(HC_SET, LOW);
  delay(200);
  HC12.print("AT+SLEEP");
  delay(200);
  digitalWrite(HC_SET, HIGH);
  delay(200);
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
    
    dataStr = "AT+HTTPPARA=\"URL\",\"www.bosl.com.au/IoT/LMP/scripts/WriteMe.php?SiteName=";

    dataStr += SITEID;
    dataStr += ".csv";
    dataStr += "&T=";
    dataStr += CYCLE;
    dataStr += "&EC=";
    dataStr += RESTART;
    dataStr += "&D=";
    dataStr += ACTIVATION;
    dataStr += "\"";
    
    netReg();
    
    
    ///***check logic
   //set CSTT - if it is already set, then no need to do again...
        sendATcmd(F("AT+CSTT?"), "OK",1000);   
        if (strstr(response, "mdata.net.au") != NULL){
            //this means the cstt has been set, so no need to set again!
            Serial.println("CSTT already set to APN ...no need to set again");
       } else {
            sendATcmd(F("AT+CSTT=\"mdata.net.au\""), "OK",1000);
        }
    
    
    //close open bearer
    sendATcmd(F("AT+SAPBR=2,1"), "OK",1000);
    if (strstr(response, "1,1") == NULL){
        if (strstr(response, "1,3") == NULL){
        sendATcmd(F("AT+SAPBR=0,1"), "OK",1000);
        }
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

    Serial.print(F("My Response: "));
    Serial.println(MyResponse);
    delay(100);

    Serial.print(F("VariableData: "));
    Serial.println(VariableData);
    delay(100);

    Serial.print(F("Sampler mode: "));
    Serial.println(WebCmd);
    delay(100);
    
}




////////////////////////////register to network////////////////////////////
void netReg(){
    sendATcmd(F("AT+CFUN=0"), "OK", 1000);
    
    if(sendATcmd(F("AT+CFUN=1"), "+CPIN: READY", 1000) == 0){
        sendATcmd(F("AT+CFUN=6"), "OK", 10000);
        xDelay(10000);
        
        sendATcmd(F("AT+CFUN=1"), "OK", 1000);
    }
    xDelay(2000);
    sendATcmd(F("AT+CREG?"), "+CREG: 0,1", 2000);
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
