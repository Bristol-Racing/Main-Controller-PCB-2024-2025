/*
Copyright (c) [2023] [Ben Hepple]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

///////////////////////////
// Important Information //
///////////////////////////

/*

In this 'Base Build' you will find the bare minimum needed for the Ajax 2 V4
board to run correctly. I have included the 'Start' and 'Pickup' Mode Functions
to save you the time of working out the control logic required to have soft starts.

You will see the USER VARIABLES below, and while you are allowed to change these 
variables, I would make sure you are familiar with the ENTIRE script before you
do so. Please read all the comments and do not change anything unless you know
it's function within the script. 

If you wish to add code and don't know where to begin or you don't understand 
why I have done something a certain way please do not hesitate to contact the 
team and I at: coalescence.gp@gmail.com or www.coalescencegp.co.uk

We are very friendly and always willing to help. 

Ben 

*/

/////////////////////////////////////////////////////

/*           USER VARIABLE (CHANGE THESE)          */

/////////////////////////////////////////////////////

////////////////////////
// DANGER DEBUG ONLY! //
////////////////////////

// Please check this area everytime you upload! 
// If in doubt, use 'false' state during races.

bool DEBUG          = false;   // This must be true for any debug commands to work
bool debugRamp      = false;  // Extra failsafe for the start up test ramp - will ramp motor in the setup to check motor drive is working. 
const int debugPwm  = 700;    // Steady-state speed of the motor after debugRamp in setup, for debug testing. Only used if debugRamp = true. 

// To see all commands run under debug mode
// use Ctrl+F and search DEBUG ONLY.

////////////////////////
//       Limits       //
////////////////////////

bool voltageLimiting = true;         // False defaults to Current Limiting Control Scheme.

float startLimit     = 43;
float pickupLimit    = 38;
float bypassLimit    = 40;

float currentLimit1  = 26;           // These are F24+ Limits, please lower them for F24 as you need to last another 30 minutes!
float currentLimit2  = 27;
float currentLimit3  = 28;

float voltageLimit1  = 21;
float voltageLimit2  = 22;
float voltageLimit3  = 23;

// WARNING: Changing these values changes how the car
//          responds to the limits, be careful. 

float Kpa            = 2;
float Kpv            = 20;

////////////////////////
//    Fan Settings    //
////////////////////////

int fanSpeed = 50;  // Enter value between 1023 -> 0 where 1023 pwm is 0% FAN Ouput, and 0 pwm is 100% FAN Output (Inverted)

/////////////////////////////////////////////////////

/*      SETUP VARIABLE (DO NOT CHANGE THESE)       */

/////////////////////////////////////////////////////

////////////////////////
//      Fan Setup     //
////////////////////////

#define PWMb 13
const int pwmChannelb = 1;

////////////////////////
//   SD Card Setup    //
////////////////////////

#include "FS.h"
#include "SD.h"
#include "SPI.h"

unsigned long dataMillis = 0;
const int   dataInterval = 100;
unsigned long raceTime   = 0;
String dataString; 

// SD Card command functions

void appendTitle(fs::FS &fs, const char * path, const char * message)
{
  File file = fs.open(path, FILE_APPEND);
  file.print(message);
  file.close();
}

void appendData(fs::FS &fs, const char * path, String  message)
{
  File file = fs.open(path, FILE_APPEND);
  file.print(message);
  file.close();
}

////////////////////////
// Motor Drive Setup  //
////////////////////////

#define PWMa 27
#define TH1 35
#define TH2 32
#define IN1 25
#define IN2 26

int throttle1 = 0;
int throttle2 = 0;
int input1   = 0;
int input2   = 0;

const int freq        = 15000;
const int resolution  = 10;
const int pwmChannela = 0;
int       pwm         = 0;

int       switchState = 1;
int       driveMode   = 1;

// Variables to detect 'Start Mode' procedure
bool startButtonTimer = true;                          // Flag that tells the control loop to start the soft start detection timer when the throttle turns off. 
const int waitKeyInt  = 6000;                          // Interval of time the car must be stationary for the soft start to activate. (milliseconds)
unsigned long waitKey = 0;                             

bool firstTouch       = true;                          // Has the throttle NOT been touched for the first time? (See Main loop, StartMode and Pickup throttle functions)
bool pickup           = false;                         // Has the throttle been released and the car is still moving? (See Main loop and Pickup throttle functions)
bool rampInc          = false;                         // Allow car to increase throttle (See StartMode and Timer Functions)

int totalError        = 0;
int previousError     = 0;
float u;


////////////////////////
//   Timer(s) Setup   //
////////////////////////

// I have listed what each timer is currently timing.

// I would recommend you do the same so you know which 
// functions/variables would be effected by changing 
// the invtervals of each timer. 

unsigned long lowPriorityMillis = 0;
const int lowPriorityInterval   = 500; 
// Current Items:
//  modesSwitchStates();
//  outputDebug();     

unsigned long medPriorityMillis = 0;
const int medPriorityInterval   = 250;
// Current Items:
//  rampInc = true

unsigned long hiPriorityMillis  = 0;
const int hiPriorityInterval    = 125;
// Current Items:
//  saveData()

////////////////////////
//  ADS Sensor Setup  //
////////////////////////

#include <Wire.h>
#include <Adafruit_ADS1X15.h>
Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */

//const float gain = 0.0001875 // for Gain = 2/3x
const float gain = 0.000125; // for Gain = 1x
const float mapb = 4.907;    // Calibrated Current milli amps per bit value. 

const int offsetSmoothing  = 60;
const int currentSmoothing = 3;
int       zeroingValue     = 0;

float batteryVoltage = 0;
float motorVoltage = 0;
float Current      = 0;

// NOTE: I found this table below online and include it just for reference. 
// In the setup() function i have used a GAIN of ONE for our purposes as we
// will not exceed the VDD on the input side. WARNING: Increasing the GAIN 
// may inhibit the speed at which you can get readings from the ADS1115. 

// The ADC input range (or gain) can be changed via the following
// functions, but be careful never to exceed VDD +0.3V max, or to
// exceed the upper and lower limits if you adjust the input range!
// Setting these values incorrectly may destroy your ADC!
//                                                                ADS1015  ADS1115
//                                                                -------  -------
// ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
// ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
// ads.setGain(GAIN_TWO);         // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
// ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
// ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
// ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

// In theory with a gain set to one, the ads1115 would see 0.5 amp as a 101.89bit increment.
// therefore we can set the mapb (milli amps per bit value) as 4.907 (500ma/101.89bit)


////////////////////////
//      RPM Setup     //
////////////////////////

int RPM                               = 0;
const uint8_t rpmPin                  = 33;

volatile unsigned long motorPoll      = 0;
unsigned long lastMotorSpeedPollTime  = 0;

//attach external interrupt to internal RAM
void IRAM_ATTR isr() {
  // KEEP THIS SHORT
  motorPoll++; // every time interrupt is triggered, add one to poll count. 
}

/////////////////////////////////////////////////////

/////////////////////////////////////////////////////


/*                      SETUP                      */


/////////////////////////////////////////////////////

/////////////////////////////////////////////////////

void setup()
{  
  Serial.begin(115200);

  // Setup SD card 
  if(!SD.begin(5)){
    Serial.println("Card Mount Failed");
  }
  // Add ", Variable_Name" to the string if you have added a variable to be logged. e.g. "TIME, BV, MV, C, RPM, AmpHours"
  appendTitle(SD, "/HadesData.txt", "TIME, BV, MV, C, RPM");


  // Setup Driver Inputs
  pinMode(TH1, INPUT);
  pinMode(TH2, INPUT);
  pinMode(IN1, INPUT);
  pinMode(IN2, INPUT);


  // RPM Interrupt
  attachInterrupt(rpmPin, isr, RISING);


  // Start ADS1115 ADC module
  if (!ads.begin())
  {
    Serial.println("Failed to initialize ADS.");
    while (1);
  } 
  ads.setGain(GAIN_ONE);


  // ADC Zeroing
  int adcAverage = 0;
  for (int i = 0; i<offsetSmoothing; i++)
  { 
    adcAverage = adcAverage + ads.readADC_SingleEnded(2);
  } 
  zeroingValue = adcAverage/offsetSmoothing;


  // initiate soft start checking
  waitKey = millis();  


  // Set Motor PWM Channel and Frequency.
  ledcSetup(pwmChannela, freq, resolution);
  ledcAttachPin(PWMa, pwmChannela);


  // Set Fan PWM Channel and Frequency.
  ledcSetup(pwmChannelb, freq, resolution);
  ledcAttachPin(PWMb, pwmChannelb);


  // Set initial fan state.
  updateFan();


  //////////////////////////////////////
  //------- DANGER DEBUG ONLY! -------//
  //////////////////////////////////////

  if(debugRamp == true && DEBUG == true){
  /* Attach the LED PWM Channel to the GPIO Pin */
    delay(100);
    
    ledcWrite(pwmChannela, 0);
        for(int i = 0; i<debugPwm; i++){
        delay(30);
        ledcWrite(pwmChannela, i);
      }
  }
}

/////////////////////////////////////////////////////

/////////////////////////////////////////////////////


/*                  Main Loop                      */


/////////////////////////////////////////////////////

/////////////////////////////////////////////////////


void loop(){


  // Get Throttle Inputs
  throttle1 = digitalRead(TH1);   // NOTE: Throttle state is INVERTED: LOW = Throttle PRESSED(ON), HIGH = Throttle RELEASED(OFF).
  throttle2 = digitalRead(TH2);


  // Get Sensor Variables
  getSensorData();

  //Checks if time dependant operations are needed.
  timer();

  //Car mode decision logic
  updateDriveMode();

  /////////////////////////////
  // Main Control Loop Start // 
  /////////////////////////////

  // Throttle ON state = LOW, Operations to be completed:
  if(throttle1 == LOW || throttle2 == LOW)
  {

    // start detection trigger
    startButtonTimer = true;

    // Switch case decides what control theory the car uses based of driveMode variable. 
    switch(driveMode){

      case 0: 
        startMode();
        break;

      case 1:
        if(voltageLimiting == true){
          pwm += calculatePwm(voltageLimit1);        
        }
        else if (voltageLimiting == false){
          pwm += calculatePwm(currentLimit1);        
        }
        break;


      case 2:
        if(voltageLimiting == true){
          pwm += calculatePwm(voltageLimit2);        
        }
        else if (voltageLimiting == false){
          pwm += calculatePwm(currentLimit2);        
        }
        break;


      case 3:
        if(voltageLimiting == true){
          pwm += calculatePwm(voltageLimit3);        
        }
        else if (voltageLimiting == false){
          pwm += calculatePwm(currentLimit3);        
        }
        break;


      case 4:
        pwm += calculatePwm(bypassLimit);
        break;

      case 5:
        throttlePickup();
    }

    
  }

  // Throttle OFF state = HIGH, Operations to be completed:
  else if(throttle1 == HIGH || throttle2 == HIGH)
  {
    firstTouch = true;

    if(startButtonTimer == true){
      startButtonTimer = false;
      waitKey = millis();    
    }  

    if(driveMode != 0){
      pickup = true;    
    } 

    pwm = 0;
  }

  ///////////////////////////
  // Main Control Loop End // 
  ///////////////////////////
  
  // DEBUG ONLY
  if(debugRamp == true && DEBUG == true){
    pwm = debugPwm;
  }

  // Range PWM output values to within resolution.
  if(pwm>=1023){
    pwm = 1023;
  }
  else if(pwm < 0){
    pwm = 0;
  }
  
  // Output Resolved PWM
  ledcWrite(pwmChannela, pwm);
}

/////////////////////////////////////////////////////

/////////////////////////////////////////////////////


/*                  Functions                      */


/////////////////////////////////////////////////////

/////////////////////////////////////////////////////



/////////////////////////////////////////////////////
/*               Timed Tasks Function              */
/////////////////////////////////////////////////////
// Seperate function for timers
void timer()                                                                  // Seperate function for timers
{
  
  // Operations to be completed every now an again to update non critical variables. 
  if (millis() >= lowPriorityMillis + lowPriorityInterval){
    lowPriorityMillis += lowPriorityInterval;

    // Enter Low Priority Functions below
    modeSwitchStates();
    outputDebug();
  }

  // Operations to be completed fairly regularly
  if (millis() >= medPriorityMillis + medPriorityInterval){
    medPriorityMillis += medPriorityInterval;

    // Enter Medium Priority Functions below 
    rampInc = true;
  }

  // Operations to be completed very regularly  
  if (millis() >= hiPriorityMillis + hiPriorityInterval){
    hiPriorityMillis += hiPriorityInterval;

    // Enter High Priority Functions below
    saveData();
  }

  // Start Mode and Pickup Mode Ramp timers.

}

/////////////////////////////////////////////////////
/*             Get 'Mode-Switch' States            */
/////////////////////////////////////////////////////

void modeSwitchStates(){

  // Update driver mode settings.
  input1 = digitalRead(IN1);
  input2 = digitalRead(IN2);

  int tempMode = switchState;

  // We use two SPST toggle switches to give us 4 modes (2^2). 
  // The combination of one switch being up or down relative to the other gives binary outputs, 
  // i.e: 00 | 01 | 10 | 11 == up,up | up,down | down,up | down,down
  
  // Each state is commented with the physical position of the switch 
  // e.g. up, up = LOW, LOW (Inverted because of pull up resistors)

  if((input1 == HIGH) && (input2 == HIGH)){     // down, down 
    switchState = 1;
  }
  else if((input1 == HIGH) && (input2 == LOW)){ // down, up
    switchState = 2;
  }
  else if((input1 == LOW) && (input2 == HIGH)){ // up, down
    switchState = 3;
  }
  else if((input1 == LOW) && (input2 == LOW)){  // up, up
    switchState = 4;
  }
}

/////////////////////////////////////////////////////
/*                Update Drive Mode                */
/////////////////////////////////////////////////////

void updateDriveMode(){

  // Check if Start procedure is active
  if((millis() > (waitKey + waitKeyInt)) && Current <= 2 && motorVoltage <= 3.5){
    driveMode = 0;
  }
  else if(motorVoltage >= 21.5){
    driveMode = switchState;
  }

  // Mode 4 = override all other modes and start functions ("BYPASS")
  if(switchState == 4){
    driveMode = switchState;
  }

  // Cornering pickup detection 
  if(pickup == true && driveMode != 0 && driveMode != 4){
    driveMode = 5;
  }
  else if(pickup == false && driveMode != 0){
    driveMode = switchState;
  }
} 

/////////////////////////////////////////////////////
/*            'Soft' Start Mode  Functions         */
/////////////////////////////////////////////////////

// Start mode detection 
void startMode(){ 
  if(firstTouch == true){
    firstTouch = false; 
    pwm = 350; // abitrary number to "shunt" the motor.
  }
  
  // Increase output if ramp conditions met.
  if(rampInc == true && Current <= startLimit){
    rampInc = false;
    pwm = pwm + 20;
  } 
}

/////////////////////////////////////////////////////
/*      Cornering & Throttle Pickup Function       */
/////////////////////////////////////////////////////

// Cornering detection and throttle pickup function. 
void throttlePickup(){

  //Detect first press since throttle has been released.
  if(firstTouch == true){
    firstTouch = false;
    pwm = 750;
  }

  // Ramp power output linearly for smooth corner exit.
  if(rampInc == true && Current <= pickupLimit){
    rampInc = false;
    pwm = pwm + 15;
  }

  // 
  if(motorVoltage >= 21.5){
    pickup = false;
  }  
}

/////////////////////////////////////////////////////
/*           Main PWM Calculation Function         */
/////////////////////////////////////////////////////

// Normal automatic PID pwm response loop
// Only difference between voltage limiting and current 
// limiting is the speed of control response (See: Kpv and Kpa Variables).

int calculatePwm(float limit){
  
  if(voltageLimiting == false || driveMode == 4){
    float error = limit - Current;
    float u1;
    u1 = Kpa * error; 
    return u1;   
  }
  
  else if(voltageLimiting == true){
    float error = limit - motorVoltage;
    float u1;
    u1 = Kpv * error; 
    return u1;
  }

}

/////////////////////////////////////////////////////
/*              Fan Control Function               */
/////////////////////////////////////////////////////

void updateFan(){
  
  // Fan Output PWM is ACTIVE LOW; 0 pwm = 100% FAN output.
  // Value between 1023 -> 0
  
  //Check user entered value is within range. 
  if(fanSpeed>=1023){
    fanSpeed = 1023;
  }
  else if(fanSpeed < 0){
    fanSpeed = 0;
  }

  ledcWrite(pwmChannelb, fanSpeed); 

}

/////////////////////////////////////////////////////
/*              Sensor Data Function               */
/////////////////////////////////////////////////////

//returns nothing, uses global variable ;(.
void getSensorData()
{

  // Declare Temporary Sensor memory 
  int16_t adc0, adc1, adc2, adc3;
  float volts0, volts1, volts2;
  
  // Get spooky single shot reading from ADS sensor.
  adc0 = ads.readADC_SingleEnded(0);
  adc1 = ads.readADC_SingleEnded(1);
  adc2 = ads.readADC_SingleEnded(2);

  // Calibrate each current reading.
  adc2 -= zeroingValue;

  // constrain output so there are no erroneous negative values
  adc2 = constrain(adc2,0,30000);

  // Multiply each reading with a calibrated gain. 
  volts0 = adc0 * gain;
  volts1 = adc1 * gain;
  volts2 = adc2 * mapb; // mapb: milli amps per bit = 4.907

  // fudge the voltage readings and format to the correct scale. 
  batteryVoltage  = (volts1 * 9.06) + 0.15;
  motorVoltage    = batteryVoltage - (volts0 * 9.06);
  Current         = volts2 / 1000; //convert milli amps to amps

  motorVoltage = constrain(motorVoltage,0,30);

  // trigger rpm readings when 28 values have been collected. (doesn't really work for low speed, but that doesn't matter)
  // this is temporary and i have an intention to change the way this is done. 
  if(motorPoll >= 28){
    readMotorRPM();
  }

}

/////////////////////////////////////////////////////
/*          Motor RPM Calculation Function         */
/////////////////////////////////////////////////////

void readMotorRPM()
{
  // This may look familiar to some of you eChookers ;)

  // Records the number of interrupts that have occured since the last function recall  
  int tempMotorPoll = motorPoll;                                                    

  // Reset previous value
  motorPoll = 0;                                                                    
  
  // Use variables to remember the time when the last RPM was calculated.
  unsigned long tempLastMotorPollTime = lastMotorSpeedPollTime;                     
  
  // Record the current time to be used in the this cycle and the next.
  lastMotorSpeedPollTime = millis();    

  // Now calculate the number of revolutions of the motor shaft                                            
  float motorRevolutions = tempMotorPoll / 3;                                       
  
  // Calculate the revolutions in one minute
  motorRevolutions = motorRevolutions * 60.0;              

  // Find the time difference in seconds                         
  float timeDifference = (lastMotorSpeedPollTime - tempLastMotorPollTime)/1000.0;   
  
  // Finally calculate RPM 
  float ShaftRPM = motorRevolutions / timeDifference;                               
  RPM = ShaftRPM;
  
}

/////////////////////////////////////////////////////
/*             Debug Serial Output Funtion         */
/////////////////////////////////////////////////////
  
void outputDebug(){

  // DEBUG ONLY
  if(DEBUG == true){

    // Display all important variables, feel free to add any global variables you wish to see when Debugging.
    Serial.println("-----------------------------------------------------------");
  
    Serial.print("BVolts: "); Serial.print(batteryVoltage); Serial.println("V");
    Serial.print("MVolts: "); Serial.print(motorVoltage); Serial.println("V");
    Serial.print("Current: "); Serial.print(Current); Serial.println("A");
    Serial.print("TH1: "); Serial.print(throttle1);Serial.print("    TH2: "); Serial.println(throttle2);
    Serial.print("IN1: "); Serial.print(input1); Serial.print("    IN2: "); Serial.println(input2);
    Serial.print("PWM: "); Serial.print(pwm); Serial.print("    RPM: "); Serial.println(RPM);
    Serial.print("MODE"); Serial.println(driveMode);
  }
}

/////////////////////////////////////////////////////
/*          SD Card Save Function                  */
/////////////////////////////////////////////////////

void saveData()
{
  raceTime = millis();

  // Super inefficient use of RAM but super reliable, and since the esp32 is a beast we don't care about RAM usage. 
  // If you add a variable to the String of data, please add a suitable "varible_name," to the appendTitle() function in the Setup() function.
  String (dataString) = "\n" + String(raceTime) + "," + String(batteryVoltage) + "," + String(motorVoltage) + "," + String(Current) + "," + String(RPM);
  appendData(SD, "/HadesData.txt", dataString);
  
}
