/*
	Cat Laser
	Reuben Strangelove
	12-22-2014 

	Records the pointing of a laser by human hands then plays the pattern back using two servos.
	
	MCU:
		Arduino Nano (Atmega328p)
	
	Notes:
		The Atmega328p is limited in RAM, therefore the recording time is very short.
		
	IDE:
		Arduino Software (IDE).
		
	Libraries: 
		MPU6050 Library by Jeff Rowberg <jeff@rowberg.net>
		https://github.com/jrowberg/i2cdevlib
		
		Timer Library by multiple contributers
		https://github.com/JChristensen/Timer
	
	TODO:
		Use a more power MCU for better resolution and recording length.
		Update MPU6050 library access methods to latest version.
*/
	
//declare pins
#define LED_PIN 13 		//internal LED indicator
#define LED_RECORD 6
#define LED_PLAY 7
#define LED_STOP 8
#define BUTTON_RECORD 9
#define BUTTON_PLAY 10
#define BUTTON_STOP 11
#define LASER 2
#define SERVO_PITCH_PIN 3
#define SERVO_YAW_PIN 5

//program configuration
#define SERVO_PITCH_CENTER 90 		//servo value that will center laser down the device
#define SERVO_YAW_CENTER 79 		//servo value that will center laser down the device
#define PROCESS_DELAY_MS 20 		//delay between saving and playing data
#define DATA_SIZE 60 				//size of storage array for position data
#define BUTTON_DEBOUNCE_DELAY 20 	//debounce delay in ms

//setup servo and timer
#include <Servo.h> 
Servo servo_yaw, servo_pitch;
#include "Timer.h"
Timer t;

//declare variables
bool blinkState = false;
bool buttonRecordReleaseFlag = true;
bool buttonPlayReleaseFlag = true;
bool buttonStopReleaseFlag = true;
bool firstIntoRecordFlag;
bool firstIntoPlayFlag;   
bool processFlag;

byte dataYaw[DATA_SIZE], dataPitch[DATA_SIZE];
float yaw, pitch, roll, offsetYaw, offsetPitch;
int recordIndex, playIndex;
byte mode = 3; //startup in stop mode 
int debounceCounter;

////////////////////////////////////////////////// 
//MPU6050 Configuration
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// use default I2C device address
MPU6050 mpu;

//define how the MPU6050 library formats the sensor data
#define OUTPUT_READABLE_YAWPITCHROLL

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication   
  Serial.begin(57600);

  ///////////////////////////////////////////////////////////////
  // initialize MPU6050
  Serial.println(F("Initializing MPU6050..."));
  mpu.initialize();
  // verify connection
  Serial.println(F("Testing MPU6050 connection..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  // load and configure the DMP
  Serial.println(F("Initializing MPU6050 DMP..."));
  devStatus = mpu.dmpInitialize();

  //manual gyro offsets  
  mpu.setXGyroOffset(12);
  mpu.setYGyroOffset(-35);
  mpu.setZGyroOffset(-14);
  mpu.setXAccelOffset(-1676);
  mpu.setYAccelOffset(-3643);
  mpu.setZAccelOffset(1171);


  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);    
    mpuIntStatus = mpu.getIntStatus();
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } 
  else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  ///////////////////////////////////////////////////////////////

  //program initialization
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_RECORD, OUTPUT);
  pinMode(LED_PLAY, OUTPUT);
  pinMode(LED_STOP, OUTPUT);
  pinMode(LASER, OUTPUT);

  pinMode(BUTTON_RECORD, INPUT);
  digitalWrite(BUTTON_RECORD, HIGH); // turn on pullup resistor    
  pinMode(BUTTON_PLAY, INPUT);
  digitalWrite(BUTTON_PLAY, HIGH); // turn on pullup resistor    
  pinMode(BUTTON_STOP, INPUT);
  digitalWrite(BUTTON_STOP, HIGH); // turn on pullup resistor

  servo_yaw.attach(SERVO_YAW_PIN);
  servo_pitch.attach(SERVO_PITCH_PIN);

  t.every(1, timerUpdate);
  t.every(PROCESS_DELAY_MS, timerProcessFlag);

  //display LED startup flashes
  for (byte x = 0; x < 3; x++)
  {
    digitalWrite(LED_RECORD, HIGH);
    delay(25);
    digitalWrite(LED_RECORD, LOW);
    delay(25);
    digitalWrite(LED_PLAY, HIGH);
    delay(25);
    digitalWrite(LED_PLAY, LOW);
    delay(25);
    digitalWrite(LED_STOP, HIGH);
    delay(25);
    digitalWrite(LED_STOP, LOW);
    delay(25);
  }
  
  digitalWrite(LED_STOP, HIGH);
}


// ================================================================
// ===                  MPU6050 Update Data                     ===
// ================================================================

void MPU6050_update_data() 
{

  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
 
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) 
  {
    // reset so we can continue cleanly    mpu.resetFIFO();  
    Serial.println(F("FIFO overflow!"));    
  } 
  else if (mpuIntStatus & 0x02) 
  { 
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
 
    mpu.getFIFOBytes(fifoBuffer, packetSize); 
    fifoCount -= packetSize; 
    mpu.dmpGetQuaternion(&q, fifoBuffer);  
    mpu.dmpGetGravity(&gravity, &q);    
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    yaw = ypr[0] * 180/M_PI;
    pitch = ypr[2] * 180/M_PI;
    roll = ypr[1] * 180/M_PI;  

    //manipulate the vaules to fall into our convention
    yaw = -yaw;
    pitch = pitch + 90;   

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}  


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() 
{  
  t.update();

  // if MPU6050 programming failed,  blink indicator (freeze program)
  if (!dmpReady) 
  { 
    while(1){
      digitalWrite(LED_STOP, HIGH);
      delay(200);
      digitalWrite(LED_STOP, LOW);
      delay(200);
    } 
  }

  // check for new data from the MPU6050 (via interrupt or FIFO data)
  if (mpu.getFIFOCount() > packetSize) MPU6050_update_data();

  //check for button pushes
  checkButtons();

  if (mode == 1) modeRecord();  
  if (mode == 2) modePlay();  
  if (mode == 3) modeStop(); 

  if (mode != 1) digitalWrite(LED_RECORD, LOW);  
  if (mode != 2) digitalWrite(LED_PLAY, LOW);  
  if (mode != 3) digitalWrite(LED_STOP, LOW); 

  if (mode == 1) digitalWrite(LASER, HIGH);  
  if (mode == 2) digitalWrite(LASER, HIGH);  
  if (mode == 3) digitalWrite(LASER, LOW); 

}


void checkButtons() {

  if (digitalRead(BUTTON_RECORD) == 0 && buttonRecordReleaseFlag == true) {

    debounceCounter = 0;
    buttonRecordReleaseFlag = false;
    mode = 1;
    firstIntoRecordFlag = true;
  }

  if (digitalRead(BUTTON_PLAY) == 0 && buttonPlayReleaseFlag == true) {

    debounceCounter = 0;
    buttonPlayReleaseFlag = false;
    mode = 2;
    firstIntoPlayFlag = true;
  }

  if (digitalRead(BUTTON_STOP) == 0 && buttonStopReleaseFlag == true) {

    debounceCounter = 0;
    buttonStopReleaseFlag = false;
    mode = 3; //stop
    digitalWrite(LED_STOP, HIGH);  
    delay(100);
  } 

  //release switch flags after the debounce period and the button as been depressed
  if (digitalRead(BUTTON_RECORD) == 1 && debounceCounter > BUTTON_DEBOUNCE_DELAY) buttonRecordReleaseFlag = true;
  if (digitalRead(BUTTON_PLAY) == 1 && debounceCounter > BUTTON_DEBOUNCE_DELAY) buttonPlayReleaseFlag = true;
  if (digitalRead(BUTTON_STOP) == 1 && debounceCounter > BUTTON_DEBOUNCE_DELAY) buttonStopReleaseFlag = true;  

} //end checkButtons


void modeRecord() 
{

  //entering recording mode from button push
  if (firstIntoRecordFlag == true) 
  {
    firstIntoRecordFlag = false;

    //center servos, no servo movement during recording    
    servo_yaw.write(SERVO_YAW_CENTER); 
    servo_pitch.write(SERVO_PITCH_CENTER); 

    digitalWrite(LED_RECORD, HIGH);

    recordIndex = 0;

    offsetYaw = yaw;
    offsetPitch = pitch;

    Serial.println("Begin recording.");

  }

  if (processFlag == true) 
  {
    processFlag = false;

    recordIndex++;

    Serial.print(yaw);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.print(roll);
    Serial.print(" = "); 

    //offset yaw to center position based on yaw at beginning of recording
    yaw = yaw - offsetYaw;

    //protect against going outside the system's physical limites
    if (yaw > 90) yaw = 90.00;
    if (yaw < -90) yaw = -90.00;
    if (pitch > 180) pitch = 180.00;
    if (pitch < 90) pitch = 90.00;

    Serial.print(yaw);
    Serial.print(" ");
    Serial.print(pitch); 
    Serial.print(" = ");
 
    dataYaw[recordIndex] = map(yaw, -90, 90, 0, 180 + (90 - SERVO_PITCH_CENTER) * 2) - 8; 
    dataPitch[recordIndex] = map(pitch, 180, 0, 180 - 8, 0 + (90 - SERVO_YAW_CENTER) * 2); 

    Serial.print( dataYaw[recordIndex] );
    Serial.print(" ");
    Serial.print(dataPitch[recordIndex] ); 
    Serial.println(" ");

    //check if at end of available space
    if (recordIndex == DATA_SIZE - 1) 
	{
      digitalWrite(LED_RECORD, LOW);

      for (byte x = 0; x < 3; x++)
	  {
        digitalWrite(LED_STOP, HIGH);
        delay(200);
        digitalWrite(LED_STOP, LOW);
        delay(200);
      }

      mode = 3; //stop 
    } 
  }
} //end modeRecord


void modePlay() 
{
  //entering play mode from button push
  if (firstIntoPlayFlag == true) 
  { 

    firstIntoPlayFlag = false;  
    digitalWrite(LED_PLAY, HIGH);
    playIndex = 0;        
    Serial.println("Begin playback."); 
  }

  if (processFlag == true) 
  {
    processFlag = false;  
	
    playIndex++;

    servo_yaw.write(dataYaw[playIndex]); 
    servo_pitch.write(dataPitch[playIndex]);     
    
    Serial.print( dataYaw[playIndex] );
    Serial.print(" ");
    Serial.print(dataPitch[playIndex] ); 
    Serial.println(" ");

    //check if at end of available data
    if ((playIndex == DATA_SIZE - 1) || (playIndex >= recordIndex)) 
	{
      Serial.println("End of playback."); 

      playIndex = 0;

      for (byte x = 0; x < 3; x++)
	  {
        digitalWrite(LED_STOP, HIGH);
        //delay(50);
        digitalWrite(LED_STOP, LOW);
        //delay(50);
      }
    } 
  } 
} //end modePlay


void modeStop() 
{
  servo_pitch.write(SERVO_PITCH_CENTER); 
  servo_yaw.write(SERVO_YAW_CENTER);
}

void timerUpdate() 
{
  debounceCounter++; 
  if (debounceCounter > 10000) 
  {
	  debounceCounter = 10000;
  }
}

void timerProcessFlag() 
{
  processFlag = true;
}