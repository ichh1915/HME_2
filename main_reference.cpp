#include "mbed.h"
#include "rtos.h"
#include "RawSerial.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include <math.h>
 
//Photointerrupter input pins
#define I1pin D2
#define I2pin D11
#define I3pin D12
 
//Incremental encoder input pins
#define CHApin   D7
#define CHBpin   D8  
 
//Motor Drive output pins   //Mask in output byte
#define L1Lpin D4           //0x01
#define L1Hpin D5           //0x02
#define L2Lpin D3           //0x04
#define L2Hpin D6           //0x08
#define L3Lpin D9           //0x10
#define L3Hpin D10          //0x20
 
//Mapping from sequential drive states to motor phase outputs
/*
State   L1  L2  L3
0       H   -   L
1       -   H   L
2       L   H   -
3       L   -   H
4       -   L   H
5       H   L   -
6       -   -   -
7       -   -   -
*/
//Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};
 
//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};  
//const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; //Alternative if phase order of input or drive is reversed
 
//Phase lead to make motor spin
volatile int8_t lead = 1;  //2 for forwards, -2 for backwards
 
//Status LED
DigitalOut led1(LED1);
 
InterruptIn I1(I1pin);
DigitalIn I2(I2pin);
DigitalIn I3(I3pin);
InterruptIn CHA(CHApin);
InterruptIn CHB(CHBpin);
RawSerial pc(SERIAL_TX, SERIAL_RX); // tx rx
Thread pidThread(osPriorityHigh, 256);
Thread serialThread(osPriorityNormal, 1024);
Thread positionControlThread(osPriorityNormal, 768);
 
//Motor Drive outputs
PwmOut L1L(L1Lpin);
PwmOut L1H(L1Hpin);
PwmOut L2L(L2Lpin);
PwmOut L2H(L2Hpin);
PwmOut L3L(L3Lpin);
PwmOut L3H(L3Hpin);
 
// number of quadrature states on the wheel
const int quadStates = 468;
// current count of quadrature states
volatile int angle = 0;
// current count of whole turns
volatile int turns = 0;
int angleTemp = 0;
int turnsTemp = 0;
int angleTarget = 0;
int turnsTarget = 0;
int anglesGap = 0;
float stepsPerTime = 0;
 
// used in reseting the angle count at the first photointerrupter
volatile bool angleFlag = false;
volatile int ccwAngle = 0;
volatile int cwAngle = 0;
 
// target number of quadrature steps to advance in (msCountSteps) ms
float targetSteps = 0;
 
// number of milliseconds to count steps taken (pid function) at both high and low speeds
const int msCountStepsHigh = 5;
const int msCountStepsLow = 25;
int msCountSteps = 0;
// factor to convert from RPS to quadrature states per (msCountSteps) ms
const float speedFactorHigh = (quadStates*msCountStepsHigh/1000);
const float speedFactorLow = (quadStates*msCountStepsLow/1000);
// PID controller
float kp = 0.0005;
float fbError = 0.0; // feedback error
float lastError = 0.0;
 
// PWM parameters
float dutyCycle = 1.0;
float period = 0.0005;
 
// variables for serial parsing thread
bool infRotate = true;
bool goMaxSpeed = true;
double rotationsTarget = 0; 
double speedTarget = 0; // RPS
const double defaultMaxSpeed = 9999; // default maximum speed in RPS
 
char serialBuffer[17];
char tempBuffer[6];
char inputChar; 
int index = 0;
int speedIndex = 0;
bool printFlag = false;
bool speedRotateFlag = false;
 
// Interrupt routines to keep track of the motor angle and number of turns so far
void Arise () {
    // at the rising edge of CHA, this interupt checks CHB to see which direction
    // the rotor is moving and increments/decrements values accordingly
    if (CHB == 1) {
        // angle overflow handling
        if (angle <= 0) {
            angle = (quadStates - 1);
            turns--;
        }
        else {
            angle--;
        }
    }
    if (CHB == 0) {
        if (angle >= (quadStates - 1)) {
            angle = 0;
            turns++;
        }
        else {
            angle++;
        }
    }
}
 
void Afall () {
    if (CHB == 0) {
        if (angle <= 0) {
            angle = (quadStates - 1);
            turns--;
        }
        else {
            angle--;
        }
    }
    if (CHB == 1) {
        if (angle >= (quadStates - 1)) {
            angle = 0;
            turns++;
        }
        else {
            angle++;
        }
    }
}
 
void Brise () {
    if (CHA == 0) {
        if (angle <= 0) {
            angle = (quadStates - 1);
            turns--;
        }
        else {
            angle--;
        }
    }
    if (CHA == 1) {
        if (angle >= (quadStates - 1)) {
            angle = 0;
            turns++;
        }
        else {
            angle++;
        }
    }
}
 
void Bfall () {
    if (CHA == 1) {
        if (angle <= 0) {
            angle = (quadStates - 1);
            turns--;
        }
        else {
            angle--;
        }
    }
    if (CHA == 0) {
        if (angle >= (quadStates - 1)) {
            angle = 0;
            turns++;
        }
        else {
            angle++;
        }
    }
}
 
// Interrupt routines to store the current values of photo interrupters
void I1rise () {
    // on the first time this interrupt is triggered, it record the angle and stores
    // the value to reset the angle counter, eliminating drift
    if (angleFlag == false) {
        // a rising edge in one direction represents a different angle to a rising edge in the opposite direction
        ccwAngle = angle;
        cwAngle = ccwAngle + 234;
        // overflow handling
        if (cwAngle > (quadStates - 1)) {
            cwAngle = cwAngle - quadStates;
        }
        angleFlag = true;
    }
    else {
        if (I3 == 1){
            angle = ccwAngle;
        }
        else {
            angle = ccwAngle;
        }
    }
}
 
//Set a given drive state
// motor outputs are Pulse-width modulated so dutyCycle is written to them insteaf of the value 1
void motorOut (int8_t driveState) {
    
    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];
      
    //Turn off first
    if (~driveOut & 0x01) L1L.write(0);
    if (~driveOut & 0x02) L1H.write(dutyCycle);
    if (~driveOut & 0x04) L2L.write(0);
    if (~driveOut & 0x08) L2H.write(dutyCycle);
    if (~driveOut & 0x10) L3L.write(0);
    if (~driveOut & 0x20) L3H.write(dutyCycle);
    
    //Then turn on
    if (driveOut & 0x01) L1L.write(dutyCycle);
    if (driveOut & 0x02) L1H.write(0);
    if (driveOut & 0x04) L2L.write(dutyCycle);
    if (driveOut & 0x08) L2H.write(0);
    if (driveOut & 0x10) L3L.write(dutyCycle);
    if (driveOut & 0x20) L3H.write(0);
    }
    
    //Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState (){ 
    return stateMap[I1 + 2*I2 + 4*I3];
}
 
//Basic synchronisation routine    
int8_t motorHome () {
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0);
    wait(1.0);
    
    //Get the rotor state
    return readRotorState();
}
 
// PID controller loop
void pid () {
    while (1) {
        // capture old values of both current angle and total turns
        angleTemp = angle;
        turnsTemp = turns;
        
        // wait length can be set dependant on the mode
        // to allow for high fbError resolutions at low speeds
        Thread::wait(msCountSteps);
        
        angleTemp = angle - angleTemp;
        
        // check for overflows and correct accordingly
        if (turns > turnsTemp) {
            stepsPerTime = (float) angleTemp + quadStates;
        }
        else if (turns < turnsTemp) {
            stepsPerTime = (float) angleTemp - quadStates;
        }
        else {
            stepsPerTime = (float) angleTemp;
        }
        
        lastError = fbError;
        
        // calculate the feedback error for both clockwise and anti-clockwise
        if (targetSteps > 0) {
            fbError = targetSteps - stepsPerTime;
        }
        else if (targetSteps < 0) {
            fbError = stepsPerTime - targetSteps;
        }
        
        // PID result is added to the current duty cycle
        dutyCycle = dutyCycle + (kp * fbError);
        
        // duty cycle limited to between 0 and 1
        if (dutyCycle > 1) {
            dutyCycle = 1;
        }
        if (dutyCycle < 0) {
            dutyCycle = 0;
        }
    }
}
 
// This function sets the mode that the PID controller operates in.
// at low speeds, the number of steps counted is low, so control is unstable.
// we avoid this by having a second mode in which more steps are counted.
// Given speed in RPS, returns steps per (msCountSteps) ms
inline float modeSet (float inputSpeed) {
    if (abs(inputSpeed) > 7) {
        msCountSteps = msCountStepsHigh;
        kp = 0.0005;
        return (inputSpeed * speedFactorHigh);
    }
    else {
        msCountSteps = msCountStepsLow;
        kp = 0.0002;
        return (inputSpeed * speedFactorLow);
    }
}
 
void positionControl() {
    while (1) {
        if(!infRotate) {
            // calculate the distance to the target position
            anglesGap = angleTarget - (angle + (turns * quadStates));
            
            // check to see if the target has been reached
            if (anglesGap <= 0 && rotationsTarget > 0) {
                pidThread.terminate();
                lead = 0;
                dutyCycle = 0;
                positionControlThread.terminate();
            }
            else if (anglesGap >= 0 && rotationsTarget < 0) {
                pidThread.terminate();
                lead = 0;
                dutyCycle = 0;
                positionControlThread.terminate();
            }
            
            // check to see if the target position is close and decelerate accordingly
            // only decelerate if speedTarget is not too low
            else if ((((stepsPerTime / msCountSteps) * 1750) > anglesGap) && speedTarget > 1.5) {
                // anglesGap and speed are in quadrature states and the input to modeSet is in RPS, so values must be scaled dependant on the mode
                speedTarget = (anglesGap * msCountSteps) / 20475;
                targetSteps = modeSet(speedTarget);
            }
            else if ((((stepsPerTime / msCountSteps) * 1750) < anglesGap) && speedTarget < -1.5) {
                speedTarget = (anglesGap * msCountSteps) / 20475;
                targetSteps = modeSet(speedTarget);
            }
        }
    }
}
 
void serial () {
    pc.printf("Please enter expression for speed and/or rotations. \r\n");
    // regular expression (R-?\d{1,3}(\.\d{1,2})?)?(V\d{1,3}(\.\d{1,3})?)?
    while (1) {
        
        // while the user is inputting 
        while(pc.readable()){
            //takes the input characters into a variable
            inputChar = pc.getc();
            //writes back to the screen so the user can see input
            pc.putc(inputChar);
            // checks if terminating character
            if(inputChar == '\r'){
                pc.printf("\r\n");
                //set print flag high, triggers parsing
                printFlag = true;
            } 
            // otherwise add the input character to the buffer
            // and move through the buffer
            else{
                serialBuffer[index] = inputChar;
                index++;
            }
        }
        
        // if line is ready
        if(printFlag){            
            
            // user input begins with V so only speed set by user
            if(serialBuffer[0] == 'V' || serialBuffer[0] == 'v'){
                // flags set
                infRotate = true;
                goMaxSpeed = false; 
                //the rest of the array is converted into a double
                speedTarget = atof(&serialBuffer[1]);
                pc.printf("User set speed only. Motor will rotate continuously at: %f rps", speedTarget);
                angleTarget = 0;
            }
            
            // statement begins with R, so could be rotations or rotations and speed
            if(serialBuffer[0] == 'R' || serialBuffer[0] == 'r'){
                
                //scan array looking for 'v' indicating speed and rotations have been set
                for(int j=1; j<index; j++){ 
                   if(serialBuffer[j] == 'v' || serialBuffer[j] =='V'){
                        //set flag to say both speed and rotation have been set 
                        speedRotateFlag = true;
                        //store where the speed value starts in buffer 
                        speedIndex = j+1; 
                    }
                    else if(!speedRotateFlag){
                        //otherwise move the values into temp buffer
                        tempBuffer[j-1] = serialBuffer[j];
                    }
                }
                // convert the tempBuffer into a double, 
                //will hold the value of number of rotations 
                rotationsTarget = atof(&tempBuffer[0]);
                angleTarget = (int) rotationsTarget * quadStates;
                
                //if 'v' is found -> both R and V
                if(speedRotateFlag){
                    // convert the speed value from the buffer array into double 
                    speedTarget = atof(&serialBuffer[speedIndex]);
                    pc.printf("User set number of rotations and speed. Motor will rotate %f times at %f rps", rotationsTarget, speedTarget);
                    //reset the flag
                    speedRotateFlag = false;
                    
                    infRotate = false; 
                    goMaxSpeed = false;  
                }
                else{
                    infRotate = false;
                    goMaxSpeed = true;
                    pc.printf("User set number of rotations only. Motor will rotate %f times at full speed", rotationsTarget);
                }
            }
                    
            pc.printf("\r\n");
            
            // reset the buffer
            for(int j =0; j<sizeof(serialBuffer); j++){
                 serialBuffer[j] = NULL;
            }
            for(int j=0; j<sizeof(tempBuffer); j++){
                tempBuffer[j] = NULL;
            } 
            //reset the print flag
            printFlag = false;
            // reset index
            index=0; 
 
            // reset porition and flags
            turns = 0;
            angle = 0;
            angleFlag = false;         
            // pass speed and rotations to processing module 
            pidThread.terminate();
            positionControlThread.terminate();
            if (goMaxSpeed){
                speedTarget = defaultMaxSpeed;
            }
            // ensure that the motor will turn int the direction of the target position regardless of speed polarity
            if ((angleTarget > 0 && speedTarget < 0) || (angleTarget < 0 && speedTarget > 0)) {
                speedTarget = -speedTarget;
            }
            targetSteps = modeSet(speedTarget);
            // set lead and start relevant threads
            if (speedTarget > 0) {
                lead = 1;
                pidThread.start(&pid);
                positionControlThread.start(&positionControl);
            }
            else if (speedTarget < 0) {
                lead = -2;
                pidThread.start(&pid);
                positionControlThread.start(&positionControl);
            }
            else {
                lead = 0;
                dutyCycle = 0;
            }
        }
    }
}
 
//Main
int main () {    
    int8_t orState = 0;    //Rotot offset at motor state 0
    
    int8_t intState = 0;
    int8_t intStateOld = 0;
    pc.printf("\n\r\n\rHello\n\r");
    
    //Run the motor synchronisation
    orState = motorHome();
    dutyCycle = 0;
    pc.printf("Rotor origin: %x\n\r", orState);
    //orState is subtracted from future rotor state inputs to align rotor and motor states
    
    // assign interrupts
    CHA.rise(&Arise);
    CHA.fall(&Afall);
    CHB.rise(&Brise);
    CHB.fall(&Bfall);
    I1.rise(&I1rise);
    
    //set period for PWM
    L1L.period(period);
    L1H.period(period);
    L2L.period(period);
    L2H.period(period);
    L3L.period(period);
    L3H.period(period);
    
    // start serial thread
    serialThread.start(&serial);
    
    //Poll the rotor state and set the motor outputs accordingly to spin the motor
    while (1) {
        intState = readRotorState();
        if (intState != intStateOld) {
            intStateOld = intState;
            motorOut((intState-orState+lead+6)%6); //+6 to make sure the remainder is positive
        }
    }
}
