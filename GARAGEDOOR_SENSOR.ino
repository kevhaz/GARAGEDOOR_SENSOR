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
#include <LoRa.h>
#include <SSD1306.h>
//#include <RCSwitch.h>
#include <ArduinoJson.h>
#include <AESLib.h>

// PIN definitions
#define PIN_SW_CLOSE 13
#define PIN_SW_OPEN  21
#define PIN_GY_X 25  // pin for x direction
#define PIN_GY_Y 35  // pin for y direction
#define PIN_GY_Z 34  // pin for z direction

// useful constants

// for debouncing
#define DEBOUNCE_DELAY 50

// for the accellerometer
#define X_SLOPE 0.0025
#define X_INTERCEPT -2.595
#define Y_SLOPE 0.00333333
#define Y_INTERCEPT -4.56
#define Z_SLOPE 0.00248139
#define Z_INTERCEPT -2.707196
#define X_CENTER 1838
#define Y_CENTER 1968
#define Z_CENTER 1897

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

// I want to record the last stable measurements from the sensors
typedef struct {
  int           intOpen;
  int           intClose;
  DOORSTATUS    status;
  int           doorAngle;
  int           startUpTime;
  unsigned long timeOfLastChange;
} LASTSTATE;

#define GY_SAMPLES 4
typedef struct {
  // Settings for the Accellerometer
  float x[GY_SAMPLES];
  float y[GY_SAMPLES];
  float z[GY_SAMPLES];
  int gy_hw = 0;
  int samplesize = 1;
} GYROREADINGS;

static LASTSTATE lastState;
static volatile GYROREADINGS gyroReadings;

// Variables for Gyro Sensoor reading interupt
volatile int interruptGyroCounter = 0;
int          totalInterruptGyroCounter = 0;
hw_timer_t * timerGyro = NULL;
portMUX_TYPE timerGyroMux = portMUX_INITIALIZER_UNLOCKED;

// variables for LoRa (time to send message) interupt.
volatile int interruptLoRaCounter = 0;
int          totalInterruptLoRaCounter = 0;
hw_timer_t * timerLoRa = NULL;
portMUX_TYPE timerLoRaMux = portMUX_INITIALIZER_UNLOCKED;

// Sperate mutext to avoid contention of the gyro readings.
portMUX_TYPE sensorMux = portMUX_INITIALIZER_UNLOCKED;

// we plan to transmit LORA message on a sperate thread
TaskHandle_t taskLoRa;

// ----------------------------------------------------------------
//
// This evidently measure's the ADC's voltage to nearer 1%
// ----------------------------------------------------------------
double ReadVoltage(byte pin){
  double reading = analogRead(pin); // Reference voltage is 3v3 so maximum reading is 3v3 = 4095 in range 0 to 4095
  if(reading < 1 || reading >= 4095)
    return 0;
    
  // return -0.000000000009824 * pow(reading,3) + 0.000000016557283 * pow(reading,2) + 0.000854596860691 * reading + 0.065440348345433;
  return -0.000000000000016 * pow(reading,4) + 0.000000000118171 * pow(reading,3)- 0.000000301211691 * pow(reading,2)+ 0.001109019271794 * reading + 0.034143524634089;
} // Added an improved polynomial, use either, comment out as required


// ----------------------------------------------------------------
//
// Convers Radians to Degrees
// ----------------------------------------------------------------
float rad2deg(float angle)
{
    return 180.0 * (angle / 3.1415926);
}


// ----------------------------------------------------------------
//
// Computes the angle of the accelterometer from it's readings
// ----------------------------------------------------------------
float computeDoorAngle()
{
    float angle_x; 
    float angle_y;
  
    float value_x = 0.0;
    float value_y = 0.0;
    float value_z = 0.0;

    int ss = 0;
      
    // compute average
    portENTER_CRITICAL_ISR(&sensorMux);
    ss = gyroReadings.samplesize;
    for( int i = 0; i< ss; i++ )
    {
      value_x += gyroReadings.x[i];   
      value_y += gyroReadings.y[i];   
      value_z += gyroReadings.z[i];   
    }
    portEXIT_CRITICAL_ISR(&sensorMux); 
    
    value_x = value_x / ss;
    value_y = value_y / ss;    
    value_z = value_z / ss;    
     
    float x2 = value_x * value_x;
    float y2 = value_y * value_y;  
    float z2 = value_z * value_z; 

    //angle_x = sqrt( y2 + z2 );
    //angle_x =  value_x / angle_x;
    //angle_x = rad2deg(atan( angle_x ));

    angle_y = sqrt( x2 + z2 );
    angle_y =  value_y / angle_y;
    angle_y = rad2deg(atan( angle_y ));

    // Convert angley into calibrated value:
    //angle_y = (fabs(angle_y) - 16) / 0.82;
    // now round to nearst 5 degrees
    //div_t dv = div( angle_y, 5);
    //angle_y = dv.quot * 5;

    return(angle_y);
}


// ----------------------------------------------------------------
//
void transmitLoRaMessage( void* parameter )
{
    float angle;
    
    // create an infinate loop
    for(;;) {
        // lets see if it is time to make a sensor measurement...
        if (interruptLoRaCounter > 0) {
          portENTER_CRITICAL(&timerLoRaMux);
          interruptLoRaCounter--;
          portEXIT_CRITICAL(&timerLoRaMux);
      
          totalInterruptLoRaCounter++;

          angle = computeDoorAngle();
          
          Serial.print("On core ");
          Serial.print(xPortGetCoreID());
          Serial.print(" a LoRa interrupt has occurred. Total number: ");
          Serial.print(totalInterruptLoRaCounter);
          Serial.print( " angle is " );
          Serial.println( angle );
         
        }
    }
}


// ----------------------------------------------------------------
//
// Interupt Handler....I'm going to use this to time when to make
// an angle measurement which I don't want to have to do to frequently.
// in normal opperation this only need to be done say, every 2 seconds(?)
// ---------------------------------------------------------------- 
void IRAM_ATTR onGyroTimer() {
  portENTER_CRITICAL_ISR(&timerGyroMux);
  interruptGyroCounter++;
  portEXIT_CRITICAL_ISR(&timerGyroMux);
 
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



// ----------------------------------------------------------------
//
// Thus reads the values from the accellerometer
// we record "GY_SAMPLES" worth of data s owe can compute a smoother average.
// ----------------------------------------------------------------
void readAccellerometer()
{
  portENTER_CRITICAL_ISR(&sensorMux);
  
  gyroReadings.x[gyroReadings.gy_hw] = float(analogRead(PIN_GY_X) - X_CENTER) / X_CENTER; //read from xpin
  gyroReadings.y[gyroReadings.gy_hw] = float(analogRead(PIN_GY_Y) - Y_CENTER) / Y_CENTER;  //read from ypin
  gyroReadings.z[gyroReadings.gy_hw] = float(analogRead(PIN_GY_Z) - Z_CENTER) / Z_CENTER;  //read from zpin
    
    if (gyroReadings.samplesize < GY_SAMPLES)
      gyroReadings.samplesize++;

    gyroReadings.gy_hw++;
    if (gyroReadings.gy_hw >= GY_SAMPLES)
      gyroReadings.gy_hw = 0;

  portEXIT_CRITICAL_ISR(&sensorMux);        
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

  String msg = "Initial door state = ";
  msg += doorStatusToString(lastState.status);
  Serial.println(msg);     
}

void setup() {
  // put your setup code here, to run once:
  // initialize Serial Monitor
  Serial.begin(115200);
  Serial.println("Runnning...");
  
  // Set up switch interrupts
  pinMode( PIN_SW_CLOSE, INPUT_PULLUP);
  pinMode( PIN_SW_OPEN,  INPUT_PULLUP);

  ResetDoorStatus();
  
  attachInterrupt(digitalPinToInterrupt(PIN_SW_CLOSE), handleDoorClosingInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_SW_OPEN), handleDoorOpeningInterrupt, CHANGE);

  // lets get the interupt timer started for the gyroscope sensor measurement (raw data)
  timerGyro = timerBegin(0, 80, true);     // this "turns" 80Mz into 1Mhz
  timerAttachInterrupt(timerGyro, &onGyroTimer, true);
  timerAlarmWrite(timerGyro, 500000, true); // this "turns" 1Mhz into 0.5 seconds.
  timerAlarmEnable(timerGyro);

  // lets get the interupt timer started for the gyroscope sensor measurement (raw data)
  timerLoRa = timerBegin(1, 80, true);     // this "turns" 80Mz into 1Mhz
  timerAttachInterrupt(timerLoRa, &onLoRaTimer, true);
  timerAlarmWrite(timerLoRa, 3500000, true); // this "turns" 1Mhz into 3.5 seconds.
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


  // lets see if it is time to make a sensor measurement...
  if (interruptGyroCounter > 0) {
    portENTER_CRITICAL(&timerGyroMux);
    interruptGyroCounter--;
    portEXIT_CRITICAL(&timerGyroMux);
    
    totalInterruptGyroCounter++;
    readAccellerometer();
    Serial.print("An gyro interrupt has occurred. Total number: ");
    Serial.println(totalInterruptGyroCounter);
  }

  // lets see if it is time to make a sensor measurement...
  if (interruptLoRaCounter > 0) {
    portENTER_CRITICAL(&timerLoRaMux);
    interruptLoRaCounter--;
    portEXIT_CRITICAL(&timerLoRaMux);

    totalInterruptLoRaCounter++;
    Serial.print("An LoRa interrupt has occurred. Total number: ");
    Serial.println(totalInterruptLoRaCounter);
    
  }

  delay(100);
  // now what? 
}
