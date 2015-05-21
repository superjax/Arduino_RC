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
#include <ros.h>
#include <std_msgs/Empty.h>
#include <relative_nav_msgs/Command.h>

#define _PI 3.1415

// Assign your channel in pins
#define ROLL_IN_PIN   8
#define PITCH_IN_PIN  9
#define THRUST_IN_PIN 10
#define YAW_IN_PIN    11

// Used to calibrate and arm ESCs
#define MAX_PWM_CMD 2000
#define MIN_PWM_CMD 900
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
// shared ones.
volatile uint16_t RollInShared;
volatile uint16_t PitchInShared;
volatile uint16_t ThrustInShared;
volatile uint16_t YawInShared;

// These are used to record the rising edge of a pulse in the ISR functions
// They do not need to be volatile as they are only used in the ISR. If we wanted
// to refer to these in loop and the ISR then they would need to be declared volatile
uint32_t RollStartEdgeISR;
uint32_t PitchStartEdgeISR;
uint32_t ThrustStartEdgeISR;
uint32_t YawStartEdgeISR;
int count;

// these variables store the last RC and ROS updates in microseconds - used in determining if we have lost RC
uint32_t LastRCUpdate;
uint32_t LastROSUpdate;

//ROS stuff
volatile uint16_t RollROSPWMShared; // these are variables that are shared between the main loop() function and the ROS node
volatile uint16_t PitchROSPWMShared;
volatile uint16_t ThrustROSPWMShared;
volatile uint16_t YawROSPWMShared;

// Setup node handle
ros::NodeHandle  nh;

// Declare publishers
relative_nav_msgs::Command command_msg;
ros::Publisher chatter("debug", &command_msg);
void commandCb( const relative_nav_msgs::Command& msg);
ros::Subscriber<relative_nav_msgs::Command> sub_command("command", &commandCb, 1);

bool light;
bool lost;

// Some other prototypes
void updateRollcmd();
void updatePitchcmd();
void updateThrustcmd();
void updateYawcmd();
float sat(float input, float poslimit, float neglimit);

void setup()
{
  //Serial.begin(115200);
  //Serial.println("multiChannels");
  pinMode(13,OUTPUT);
  pinMode( 2,OUTPUT);

  // attach ESCs
  Motor1.attach(MOTOR_1_PIN,  MIN_PWM_CMD,  MAX_PWM_CMD);
  Motor2.attach(MOTOR_2_PIN,  MIN_PWM_CMD,  MAX_PWM_CMD);
  Motor3.attach(MOTOR_3_PIN,  MIN_PWM_CMD,  MAX_PWM_CMD);
  Motor4.attach(MOTOR_4_PIN,  MIN_PWM_CMD,  MAX_PWM_CMD);

    // setup ROS node
  //nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub_command);
  nh.advertise(chatter);

  // using the PinChangeInt library, attach the interrupts
  // used to read the channels
  PCintPort::attachInterrupt(ROLL_IN_PIN,   updateRollcmd,    CHANGE); 
  PCintPort::attachInterrupt(PITCH_IN_PIN,  updatePitchcmd,   CHANGE); 
  PCintPort::attachInterrupt(THRUST_IN_PIN, updateThrustcmd,   CHANGE); 
  PCintPort::attachInterrupt(YAW_IN_PIN,    updateYawcmd,     CHANGE);
  count = 0;

  // initialize the LastRCUpdate variable
  LastRCUpdate = millis();
  LastROSUpdate = millis();
  light = 1;
  lost = 0;
}

void loop()
{
  // create local variables to hold a local copies of the RC channel inputs
  // these are declared static so that thier values will be retained 
  // between calls to loop, but remain limited in scope to loop()
  static uint16_t RollInLocal;
  static uint16_t PitchInLocal;
  static uint16_t ThrustInLocal;
  static uint16_t YawInLocal;

  // static master output variables
  static uint16_t RollOutMaster;
  static uint16_t PitchOutMaster;
  static uint16_t ThrustOutMaster;
  static uint16_t YawOutMaster;

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
    // update the local and ros values based on flags
    if(UpdateFlagsLocal & ROLL_FLAG)
    {
      RollInLocal = RollInShared;
    }
    if(UpdateFlagsLocal & PITCH_FLAG)
    {
      PitchInLocal = PitchInShared;
    }
    if(UpdateFlagsLocal & THRUST_FLAG)
    {
      ThrustInLocal = ThrustInShared;
    }
    if(UpdateFlagsLocal & YAW_FLAG)
    {
      YawInLocal = YawInShared;
    }
     
      // clear shared copy of updated flags as we have already taken the updates
      // we still have a local copy if we need to use it in UpdateFlagsLocal
      // this needs to be cleared so that we aren't always checking these variables and
      // run the risk of getting interrupts during the update
      UpdateFlagsShared = 0;

      
      interrupts(); // turn back on interrupts - DO NOT USE SHARED VARIABLES ANYMORE
      // remember when this update happened
      LastRCUpdate = millis();
  }


  // only use the local values RollInLocal, PitchInLocal, ThrustInLocal and YawInLocal, from here on;
  // the shared variables are always owned by the interrupt routines and should not be used in loop

  /**************************/
  /****   SERIAL TO i7   ****/
  /**************************/
  // populate ROS variables so they can be accessed by the node
  // after this *ROSPWM variables will be populated with the correct output
  nh.spinOnce();

  /**************************/
  /**** SELECTION LOGIC  ****/
  /**************************/ 
  // we want to make sure that the output is correct, so these are failsafes to decide when to
  // use RC inputs, or when to listen to ROS. and also how to correctly identify a failsafe.
  // Mix the i7/RC inputs


  // if our last ROS update was more than 100ms ago, we've probably lost our RC, enter failsafe
  // mode.  The Naze32 is looking for a PWM less than 985us, so 900 ought to be low enough
  // Spektrum RC recievers have a failsafe that is built into the Rx.  They drop the throttle to a
  // point determined when you program the reciever, so make sure you program the reciever to give
  // a signal lower than 985 when it loses connection with the transmitter.
  uint32_t now = millis();
  if(now - LastROSUpdate > 500)
  {
	RollOutMaster = 900;
	PitchOutMaster = 900;
	ThrustOutMaster = 900;
	YawOutMaster = 900;
    digitalWrite(13, 1);   
    lost = 1; // flag lost bit so that the program can continue and possibly get another ROS update
  }
  else
  {
    RollOutMaster   = RollROSPWMShared;  
    PitchOutMaster  = PitchROSPWMShared;
	ThrustOutMaster = min(ThrustInLocal, ThrustROSPWMShared); 
    //ThrustOutMaster = ThrustInLocal; 
    YawOutMaster    = YawROSPWMShared; 

	if((abs((int)RollInLocal  - 1500) > 50))
    {
      RollOutMaster   = RollInLocal; 
    }
    if((abs((int)PitchInLocal  - 1500) > 50))
    {
      PitchOutMaster  = PitchInLocal;
    }
	if((abs((int)YawInLocal  - 1500) > 50)) 
    {
      YawOutMaster    = YawInLocal;    
    }

    digitalWrite(13, 0);
  }

  /**************************/
  /**** OUTPUT TO MOTORS ****/
  /**************************/
  // Check to see if the channel value has changed, this is indicated  
  // by the flags. For the simple pass through we don't really need this check,
  // but for a more complex project where a new signal requires significant processing,
  // this allows us to only calculate new values when we have new inputs
  
  if(UpdateFlagsLocal & ROLL_FLAG)
  {
    if(Motor1.readMicroseconds() != RollOutMaster)
    {
      Motor1.writeMicroseconds(RollOutMaster);
    }
  }
  
  if(UpdateFlagsLocal & PITCH_FLAG)
  {
    if(Motor2.readMicroseconds() != PitchOutMaster)
    {
      Motor2.writeMicroseconds(PitchOutMaster);
    }
  }
  
  if(UpdateFlagsLocal & THRUST_FLAG)
  {
    if(Motor3.readMicroseconds() != ThrustOutMaster)
    {
      Motor3.writeMicroseconds(ThrustOutMaster);
    }
  }

    if(UpdateFlagsLocal & YAW_FLAG)
  {
    if(Motor4.readMicroseconds() != YawOutMaster)
    {
      Motor4.writeMicroseconds(YawOutMaster);
    }
  }

  UpdateFlagsLocal = 0;
  

//  throttle serial output (debugging)
  if(count >= 10000)
  {
//    Serial.println("");
//    Serial.print("Roll: ");
//    Serial.print(RollOutMaster,DEC);
//    Serial.print("\t");
//    Serial.print("Pitch: ");
//    Serial.print(PitchOutMaster,DEC);
//    Serial.print("\t");
//    Serial.print("Thrust: ");
//    Serial.print(ThrustOutMaster,DEC);
//    Serial.print("\t");
//    Serial.print("Yaw: ");
//    Serial.print(YawOutMaster,DEC);
//    Serial.print("\t");
//    Serial.print("LastUpdate: ");
//    Serial.print(LastRCUpdate,DEC);
//    Serial.print("\t");
//    Serial.print("Now:");
//    Serial.print(millis(),DEC);
//    Serial.println("\t");
    count = 0;
    
    

    // Populate output message (for debug)  - map back to rad
    command_msg.roll     = RollOutMaster;
    command_msg.pitch    = PitchOutMaster;
    command_msg.thrust   = ThrustOutMaster;
    command_msg.yaw_rate = YawOutMaster;
  
   // Publish output message
    chatter.publish( &command_msg );
  }
  else
    count++;
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


/**************************/
/****    ROS Callback  ****/
/**************************/
void commandCb( const relative_nav_msgs::Command& msg)
{
  static bool toggle = 1;
  if (toggle)
	  toggle = 0;
  else
	  toggle = 1;

  digitalWrite(2, toggle);

  // populate the *OutROS variables with the message from the i7
  // converted to RC-Style PWM
  float roll_limit = 15.0*_PI/180.0;
  float pitch_limit = 15.0*_PI/180.0;
  float yaw_rate_limit = 45.0*_PI/180.0;

  RollROSPWMShared     = sat(msg.roll,-roll_limit,roll_limit)*572.9 + 1500;     		// -50 deg -> 1000us | 50 deg -> 2000us (but in radians)
  PitchROSPWMShared    = sat(msg.pitch,-pitch_limit,pitch_limit)*-572.9 + 1500;  		// -50 deg -> 2000us | 50 deg -> 1000us
  //positive pitch = nose up
  ThrustROSPWMShared   = sat(msg.thrust,20.0,50.0)*8.108 + 1200;    					//  0 N -> 1200us      | 37N -> 1500us
  YawROSPWMShared      = sat(msg.yaw_rate,-yaw_rate_limit,yaw_rate_limit)*159.2 + 1500; //  -pi rad/s -> 1000us | pi rad/s -> 2000us

  LastROSUpdate = millis();
}

// saturation function, limits output to supplied limits
float sat(float input, float lower, float upper)
{
	if(input > upper)
		return upper;
	else if(input < lower)
		return lower;
	return input;
}
