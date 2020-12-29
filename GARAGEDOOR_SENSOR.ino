///////////////////////////////////////////////////////////////////////
//
// This is the ESP32 TTGO Lora Garage Door Sensor Sketch
//
// It senses when the garage door is fully open or fully closed. 
// It also reports on the approximate angle of the garage door.
// I might also be a temperature and humidity gauge for the garage
//
///////////////////////////////////////////////////////////////////////
#include <SPI.h>
#include <ArduinoJson.h>
#include "display.h"
#include "security.h"
#include "gg_lora.h"


// PIN definitions for switches
#define PIN_SW_CLOSE 13
#define PIN_SW_OPEN  21
// to operate the relay that operates the garage remote control
#define PIN_RELAY    17 


// for debouncing
#define DEBOUNCE_DELAY 50

// global variables.
unsigned long CloseSwitchChangeTime = 0;
unsigned long OpenSwitchChangeTime = 0;
int CloseSwitchTriggered = 0;
int OpenSwitchTriggered = 0;

// GARAGE DOOR STATUS -> This is what we think the garage door is doing.
typedef enum {
   DS_ERROR,
   DS_OPEN,
   DS_OPENING,
   DS_CLOSING,
   DS_CLOSED,
   DS_UNKNOWN
} DOORSTATUS;

// I want to record the last stable measurement from the sensors
typedef struct {
  int           intOpen;
  int           intClose;
  DOORSTATUS    status;
  DOORSTATUS    prevstatus;   // this is set, each time we send "status"
  int           repeat;
  int           startUpTime;
  unsigned long timeOfLastChange;
} LASTSTATE;
LASTSTATE lastState;

byte  localAddress = 0xAA;     // address of this device
byte  destination  = 0xBB;     // destination to send to
int   counter      = 0;

// variables for LoRa (time to send message) interupt.
volatile int interruptLoRaCounter = 0;
int          totalInterruptLoRaCounter = 0;
hw_timer_t * timerLoRa = NULL;
portMUX_TYPE timerLoRaMux = portMUX_INITIALIZER_UNLOCKED;

// we plan to transmit LORA message on a sperate thread
TaskHandle_t taskLoRa;

// Onscreen Information
String os_header = "Garage Sensor";
String os_rssi = "";
String os_snr = "";
String os_cmd = "";
String os_ts = "";



// ----------------------------------------------------------------
//
void transmitLoRaMessage( void* parameter )
{
    float angle;
    
    // create an infinite loop
    for(;;) {
        // lets see if it is time to make a sensor measurement...
        if (interruptLoRaCounter > 0) {
          portENTER_CRITICAL(&timerLoRaMux);
          interruptLoRaCounter--;
          portEXIT_CRITICAL(&timerLoRaMux);
      
          totalInterruptLoRaCounter++;

         
          Serial.print("On core ");
          Serial.print(xPortGetCoreID());
          Serial.print(" a LoRa interrupt has occurred. Total number: ");
          Serial.println(totalInterruptLoRaCounter);

          char message[128];

          // but...do I want to send the message. Each state change is sent twice only.
          if ( lastState.status != lastState.prevstatus ) {
            sprintf( message, "{frm:%d,to:%d,cnt:%d,ds:%d,t:%d}", localAddress, destination, counter++, (int)lastState.status, (int)lastState.timeOfLastChange );
            loraSendMessage( message );
            lastState.prevstatus = lastState.status;
            lastState.repeat = true;
          } else {
            if ( lastState.repeat ) {
              sprintf( message, "{frm:%d,to:%d,cnt:%d,ds:%d,t:%d}", localAddress, destination, counter++, (int)lastState.status, (int)lastState.timeOfLastChange );
              loraSendMessage( message );
              lastState.repeat = false;
            }
          }
        }
    }
}


// ----------------------------------------------------------------
//
// Interupt Handler....I'm going to use this to time when to make
// an angle measurement which I don't want to have to do to frequently.
// in normal opperation this only need to be done say, every 2 seconds(?)
// ---------------------------------------------------------------- 
void IRAM_ATTR onLoRaTimer() {
  portENTER_CRITICAL_ISR(&timerLoRaMux);
  interruptLoRaCounter++;
  portEXIT_CRITICAL_ISR(&timerLoRaMux);
}

// ----------------------------------------------------------------
//
// Convert Door Status erum to printable value
// ----------------------------------------------------------------
String doorStatusToString( const DOORSTATUS status )
{
  String retVal;
  switch ( status )
  {
    case DS_ERROR:
       retVal = "Error";
       break;
     case DS_OPEN:
       retVal = "Open";
       break;
     case DS_OPENING:
       retVal = "Opening";
       break;
     case DS_CLOSED:
       retVal = "Closed";
       break;
     case DS_CLOSING:
       retVal = "Closing";
       break;
     default:
       retVal = "Unknown";
       break;    
  }
  return retVal;
}

// will probably eventually do this on an interupt 
void updateDisplay()
{
   String scndLine;
   String thrdLine;
   String frthLine;
   String fithLine;

   scndLine = "Status:  " + doorStatusToString(lastState.status);
   thrdLine = "Last Command: " + os_cmd;
   frthLine = os_ts;
   fithLine = "RSSI: " + os_rssi + " SNR:" + os_snr + "db";
   
   display_Lines( os_header, scndLine, thrdLine, frthLine, fithLine );
}

// ----------------------------------------------------------------
//
// This is called each time the interupt for the closed door switch changes
// due to switch bounce this can be called multiple times for one "click"
// ----------------------------------------------------------------
void handleDoorClosingInterrupt() {
  if (CloseSwitchChangeTime==0)
  {
    CloseSwitchChangeTime = millis();
  } 
  CloseSwitchTriggered++; // this counts the number of switch bounces
  // Serial.println( "Close Triggered" ); <- this appears on rising and falling edges
}


// ----------------------------------------------------------------
//
// This is called each time the interupt for the open door switch changes
// due to switch bounce this can be called multiple times for one "click"
// ----------------------------------------------------------------
void handleDoorOpeningInterrupt() {
  if (OpenSwitchChangeTime==0)
  {
    OpenSwitchChangeTime = millis();
  } 
  OpenSwitchTriggered++; // this counts the number of switch bounces
  // Serial.println( "Open Triggered" );  <- this appears on rising and falling edges
}


void ResetDoorStatus()
{
  // determine initial state of door
  lastState.intClose = digitalRead(PIN_SW_CLOSE);
  lastState.intOpen  = digitalRead(PIN_SW_OPEN);  

  // remember 0 = switch closed. 1 = switch open circuit

  if ( lastState.intOpen == 0 && lastState.intClose == 0 )
    lastState.status = DS_ERROR;

  if ( lastState.intOpen == 1 && lastState.intClose == 1 )
    lastState.status = DS_UNKNOWN;

   if ( lastState.intOpen == 0 && lastState.intClose == 1 )
    lastState.status = DS_OPEN;  

   if ( lastState.intOpen == 1 && lastState.intClose == 0 )
    lastState.status = DS_CLOSED;     

  // the next we need to do is ensure we send this status. This only sends the 1st message once 
  lastState.prevstatus = lastState.status;
  lastState.repeat = false;
  
}

void setup() {
  // put your setup code here, to run once:
  // initialize Serial Monitor
  Serial.begin(115200);
  Serial.println("Runnning...");

  // define the relay output pin
  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, HIGH);
  // Set up switch interrupts
  pinMode( PIN_SW_CLOSE, INPUT_PULLUP);
  pinMode( PIN_SW_OPEN,  INPUT_PULLUP);

  ResetDoorStatus();

  // now show we are ready to go on the display
  init_Display();

  init_Lora();
  
  attachInterrupt(digitalPinToInterrupt(PIN_SW_CLOSE), handleDoorClosingInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_SW_OPEN), handleDoorOpeningInterrupt, CHANGE);

  // lets get the interupt timer started for....
  timerLoRa = timerBegin(1, 80, true);     // this "turns" 80Mz into 1Mhz
  timerAttachInterrupt(timerLoRa, &onLoRaTimer, true);
  timerAlarmWrite(timerLoRa, 5000000, true); // this "turns" 1Mhz into 5,0 seconds.
  timerAlarmEnable(timerLoRa);

  lastState.startUpTime = millis();

  xTaskCreatePinnedToCore(
      transmitLoRaMessage, "xmitLoRa", 10000, 
      NULL,  /* Task input parameter */
      0,  /* Priority of the task */
      &taskLoRa,  /* Task handle. */
      0); /* Core where the task should run */

  Serial.print("Main loop running on core ");
  Serial.println(xPortGetCoreID());

  display_Lines( "READY", doorStatusToString(lastState.status) );
}

// Not used at present....maybe one day
void taskOpenCloseGarageDoor( void* parameter )
{
  return;
}


void InstigateGarageDoorOpenClose()
{
  // we plan to run through the open/close door sequence via multitasking. The task runs and exists so no
  // need to remember the task handle.
  TaskHandle_t taskGG;

  xTaskCreatePinnedToCore(
      taskOpenCloseGarageDoor, "ggOpenClose", 10000, 
      NULL,  /* Task input parameter */
      0,  /* Priority of the task */
      &taskGG,  /* Task handle. */
      0); /* Core where the task should run */

  return;  
}

void loop() {
  // check to see what's happening
  bool bCloseSwitchChanged = false;
  bool bOpenSwitchChanged = false;

  int intLastOpenValue =   lastState.intOpen;
  int intLastCloseValue =  lastState.intClose;


  if ( OpenSwitchChangeTime!=0 && (millis() - OpenSwitchChangeTime > DEBOUNCE_DELAY ) )
  {
    // take a reading and store it.
    lastState.intOpen = digitalRead(PIN_SW_OPEN);
    Serial.print( "Open digitalRead=" );
    Serial.println( lastState.intOpen );      
    OpenSwitchChangeTime = 0;
    // have we detected a change
    if ( lastState.intOpen != intLastOpenValue ) {
      // change detected
      bOpenSwitchChanged = true;
    }
  }
  
  if ( CloseSwitchChangeTime!=0 && (millis() - CloseSwitchChangeTime > DEBOUNCE_DELAY ) )
  {
    // take a reading and store it.
    lastState.intClose = digitalRead(PIN_SW_CLOSE);
    Serial.print( "Close digitalRead=" );
    Serial.println( lastState.intClose );    
    CloseSwitchChangeTime = 0;
    // have we detected a change
    if ( lastState.intClose != intLastCloseValue ) {
      // change detected
      bCloseSwitchChanged = true;
    }
  }
  
  if ( bOpenSwitchChanged ) 
  {
    DOORSTATUS newState = DS_UNKNOWN;
    
    int c = lastState.intClose;
    int o = lastState.intOpen;

    if ( lastState.status == DS_ERROR )
    { 
      ResetDoorStatus();
      newState = lastState.status;
    } else {
      // change detected with the open switch.
      if (o == 0 && c == 0) {
          newState = DS_ERROR; // both switches are closed = error
      } else {
         if ( o == 0 ) 
           newState = DS_OPEN;
        else 
          newState = DS_CLOSING;
      }  
    }
    String msg = "Open door switch caused changed from ";
    msg += doorStatusToString(lastState.status);
    msg += " to ";
    msg += doorStatusToString(newState);
    Serial.println(msg);      
    lastState.status = newState;
    lastState.timeOfLastChange = millis();    
  } // if


  if ( bCloseSwitchChanged ) 
  {
    DOORSTATUS newState = DS_UNKNOWN;
    
    int c = lastState.intClose;
    int o = lastState.intOpen;

    if ( lastState.status == DS_ERROR )
    {
      ResetDoorStatus();
      newState = lastState.status;

    } else {
      // change detected with the closen switch.
      if (c == 0 && o == 0) {
          newState = DS_ERROR; // both switches are closed = error
      } else {
         if ( c == 0 ) 
           newState = DS_CLOSED;
        else 
          newState = DS_OPENING;
      } 
    } 
    String msg = "Close door switch caused changed from ";
    msg += doorStatusToString(lastState.status);
    msg += " to ";
    msg += doorStatusToString(newState);
    Serial.println(msg);      
    lastState.status = newState;
    lastState.timeOfLastChange = millis();    
  } // if  

  delay(100);

  String incoming_msg;
  if (onReceive(LoRa.parsePacket(), incoming_msg, os_rssi, os_snr )) {
     // got something
     // but now need to deserialise decypted message into JSON object and use that to work out what to do next
    DynamicJsonDocument jsonBuffer(512);
    DeserializationError error = deserializeJson(jsonBuffer, incoming_msg); 
    // Test if parsing succeeds.
    if (error) {
      Serial.println("gg_lora: onReceived(): parseObject() failed");
    } else {
  
      // is message for this device?
      // if the recipient isn't this for this device or broadcast
      byte recipient = jsonBuffer[String("to")];
      if (recipient != localAddress ) { // && recipient != 0xe3) {
        Serial.print("warning: This message to '" );
        Serial.print( recipient );
        Serial.println("'; is not for me.");
        //return;                             // skip rest of function
      }
    
      Serial.print( "Command Message Received: ");
      String cmd = jsonBuffer[String("cmd")];
      os_cmd = cmd;
      String ts = jsonBuffer[String("ts")];
      os_ts = ts;

      Serial.println( cmd ); 
      // at this point we have to "handle" the message. E.g. open or close the garage door?
      if (cmd.equals("OPEN") || cmd.equals("CLOSE")) {
        Serial.println( "**Activating remote**" );         
        digitalWrite(PIN_RELAY, LOW);
        delay(400);
        digitalWrite(PIN_RELAY, HIGH);        
      }

    }  
  }

  updateDisplay();

}
