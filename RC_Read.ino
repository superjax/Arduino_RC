//RC_Read
// 
//Reads multiple inputs from RC Transmitter and forwards them to Brushless motors
// Author:  James Jackson
//
// Based heavily on MultiChannels by Duane B.  at
// www.rcarduino.com
//
// include the pinchangeint library - allows us to make all 20 pins interuppt controlled.
#include <PinChangeInt.h>
#include <Servo.h>

// Assign your channel in pins
#define ROLL_IN_PIN   8
#define PITCH_IN_PIN  9
#define THRUST_IN_PIN 10
#define YAW_IN_PIN    11

// Used to calibrate and arm ESCs
#define MAX_PWM_CMD 2000
#define MIN_PWM_CMD 1000

// Debug Statement Flag
#define DEBUG_RC_READ 1

// Assign your channel out pins
#define MOTOR_1_PIN 4
#define MOTOR_2_PIN 5
#define MOTOR_3_PIN 6
#define MOTOR_4_PIN 7

// Servo objects generate the signals expected by Electronic Speed Controllers and Servos
// We will use the objects to output the signals we read in
// this example code provides a straight pass through of the signal with no custom processing
Servo Motor1;
Servo Motor2;
Servo Motor3;
Servo Motor4;

// These bit flags are set in bUpdateFlagsShared to indicate which
// channels have new signals
#define ROLL_FLAG   1
#define PITCH_FLAG  2
#define THRUST_FLAG 3
#define YAW_FLAG    4

// holds the update flags defined above
volatile uint8_t UpdateFlagsShared;

// shared variables are updated by the ISR (Interrupt Service Routine) and read by loop.
// In loop we immediatley take local copies so that the ISR can keep ownership of the 
// shared ones. To access these in loop
// we first turn interrupts off with noInterrupts
// we take a copy to use in loop and the turn interrupts back on
// as quickly as possible, this ensures that we are always able to receive new signals
volatile uint16_t RollInShared;
volatile uint16_t PitchInShared;
volatile uint16_t ThrustInShared;
volatile uint16_t YawInShared;

// These are used to record the rising edge of a pulse in the calcInput functions
// They do not need to be volatile as they are only used in the ISR. If we wanted
// to refer to these in loop and the ISR then they would need to be declared volatile
uint32_t RollStartEdgeISR;
uint32_t PitchStartEdgeISR;
uint32_t ThrustStartEdgeISR;
uint32_t YawStartEdgeISR;
int count;

void setup()
{
  Serial.begin(115200);
  Serial.println("multiChannels");

  // attach ESCs
  Motor1.attach(MOTOR_1_PIN,  MIN_PWM_CMD,  MAX_PWM_CMD);
  Motor2.attach(MOTOR_2_PIN,  MIN_PWM_CMD,  MAX_PWM_CMD);
  Motor3.attach(MOTOR_3_PIN,  MIN_PWM_CMD,  MAX_PWM_CMD);
  Motor4.attach(MOTOR_4_PIN,  MIN_PWM_CMD,  MAX_PWM_CMD);
  // arm ESCs
  Serial.println("Arming");
  Motor1.writeMicroseconds(MAX_PWM_CMD);
  Motor2.writeMicroseconds(MAX_PWM_CMD);
  Motor3.writeMicroseconds(MAX_PWM_CMD);
  Motor4.writeMicroseconds(MAX_PWM_CMD);
  delay(2000);
  Motor1.writeMicroseconds(MIN_PWM_CMD);
  Motor2.writeMicroseconds(MIN_PWM_CMD);
  Motor3.writeMicroseconds(MIN_PWM_CMD);
  Motor4.writeMicroseconds(MIN_PWM_CMD);
  delay(2000);    
  Serial.println("Done");


  // using the PinChangeInt library, attach the interrupts
  // used to read the channels
  PCintPort::attachInterrupt(ROLL_IN_PIN,   updateRollcmd,    CHANGE); 
  PCintPort::attachInterrupt(PITCH_IN_PIN,  updatePitchcmd,   CHANGE); 
  PCintPort::attachInterrupt(THRUST_IN_PIN, updateThrustcmd,   CHANGE); 
  PCintPort::attachInterrupt(YAW_IN_PIN,    updateYawcmd,     CHANGE);
  count = 0;
}

void loop()
{
  // create local variables to hold a local copies of the channel inputs
  // these are declared static so that thier values will be retained 
  // between calls to loop, but remain limited in scope to loop()
  static uint16_t RollInLocal;
  static uint16_t PitchInLocal;
  static uint16_t ThrustInLocal;
  static uint16_t YawInLocal;

  // local copy of update flags
  static uint8_t UpdateFlagsLocal;


  /******************************/
  /****** RC CHANNELS UPDATE ****/
  /******************************/
  // check shared update flags to see if any channels have a new signal
  if(UpdateFlagsShared)
  {
    noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables
    // take a local copy of which channels were updated
    UpdateFlagsLocal = UpdateFlagsShared;
    // update the local values of the channels based on the update flags    
    if(UpdateFlagsLocal & ROLL_FLAG)
      RollInLocal = RollInShared;
    if(UpdateFlagsLocal & PITCH_FLAG)
      PitchInLocal = PitchInShared;    
    if(UpdateFlagsLocal & THRUST_FLAG)
      ThrustInLocal = ThrustInShared;
    if(UpdateFlagsLocal & YAW_FLAG)
      YawInLocal = YawInShared;
     
    // clear shared copy of updated flags as we have already taken the updates
    // we still have a local copy if we need to use it in UpdateFlagsLocal
    // this needs to be cleared so that we aren't always checking these variables and
    // getting interrupts during the update
    UpdateFlagsShared = 0;
    
    interrupts(); // turn back on interrupts - DONT USE SHARED VARIABLES ANYMORE
    }
  if(count >= 10000)
  {
    Serial.print("Roll: ");
    Serial.print(RollInLocal,DEC);
    Serial.print("\t");
    Serial.print("Pitch: ");
    Serial.print(PitchInLocal,DEC);
    Serial.print("\t");
    Serial.print("Thrust: ");
    Serial.print(ThrustInLocal,DEC);
    Serial.print("\t");
    Serial.print("Yaw: ");
    Serial.print(YawInLocal,DEC);
    Serial.println("\t");
    count = 0;
  }
  else
    count++;

  // do any processing from here onwards
  // only use the local values RollInLocal, PitchInLocal, ThrustInLocal and YawInLocal, the shared
  // variables are always owned by the interrupt routines and should not be used in loop

  /**************************/
  /****   SERIAL TO i7   ****/
  /**************************/

  // send the RC command messages to the i7 via ROS

  /**************************/
  /**** SERIAL FROM i7   ****/
  /**************************/

  // receive the motor output commands from the i7 via ROS

  /**************************/
  /**** OUTPUT TO MOTORS ****/
  /**************************/
   
  // this is a simple pass through mapping RC inputs directly to motors
  
  // Check to see if the channel value has changed, this is indicated  
  // by the flags. For the simple pass through we don't really need this check,
  // but for a more complex project where a new signal requires significant processing,
  // this allows us to only calculate new values when we have new inputs
  if(UpdateFlagsLocal & ROLL_FLAG)
  {
    if(Motor1.readMicroseconds() != RollInLocal)
    {
      Motor1.writeMicroseconds(RollInLocal);
    }
  }
  
  if(UpdateFlagsLocal & PITCH_FLAG)
  {
    if(Motor2.readMicroseconds() != PitchInLocal)
    {
      Motor2.writeMicroseconds(PitchInLocal);
    }
  }
  
  if(UpdateFlagsLocal & THRUST_FLAG)
  {
    if(Motor3.readMicroseconds() != ThrustInLocal)
    {
      Motor3.writeMicroseconds(ThrustInLocal);
    }
  }

    if(UpdateFlagsLocal & YAW_FLAG)
  {
    if(Motor4.readMicroseconds() != YawInLocal)
    {
      Motor4.writeMicroseconds(YawInLocal);
    }
  }

  
  UpdateFlagsLocal = 0;
}


/**************************/
/****        ISR       ****/
/**************************/
// the interrupt service routines.  These are optimized to be as fast as possible and use
// as little memory as possible
// we can use the shared variables 
// UpdateFlagsShared
// RollInShared
// PitchInShared
// ThrustInShared
// YawInShared
// and the ISR specific variables used to store the time of rising edges
// RollStartEdgeISR
// PitchStartEdgeISR
// ThrustStartEdgeISR
// YawStartEdgeISR

void updateRollcmd()
{
  // if the pin is high, its a rising edge of the signal pulse, so lets record its value
  if(digitalRead(ROLL_IN_PIN) == HIGH)
  { 
    RollStartEdgeISR = micros();
  }
  else
  {
    // else it must be a falling edge, so lets get the time and subtract the time of the rising edge
    // this gives use the time between the rising and falling edges i.e. the pulse duration.
    RollInShared = (uint16_t)(micros() - RollStartEdgeISR);
    // set the flag to indicate that a new signal has been received
    UpdateFlagsShared |= ROLL_FLAG;
  }
}

void updatePitchcmd()
{
  if(digitalRead(PITCH_IN_PIN) == HIGH)
  { 
    PitchStartEdgeISR = micros();
  }
  else
  {
    PitchInShared = (uint16_t)(micros() - PitchStartEdgeISR);
    UpdateFlagsShared |= PITCH_FLAG;
  }
}

void updateThrustcmd()
{
  if(digitalRead(THRUST_IN_PIN) == HIGH)
  { 
    ThrustStartEdgeISR = micros();
  }
  else
  {
    ThrustInShared = (uint16_t)(micros() - ThrustStartEdgeISR);
    UpdateFlagsShared |= THRUST_FLAG;
  }
}

void updateYawcmd()
{
  if(digitalRead(YAW_IN_PIN) == HIGH)
  { 
    YawStartEdgeISR = micros();
  }
  else
  {
    YawInShared = (uint16_t)(micros() - YawStartEdgeISR);
    UpdateFlagsShared |= YAW_FLAG;
  }
}
