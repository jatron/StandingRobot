 #include <math.h>

// Code to talk to the IMU
#include <MPU9250.h>
#include <Wire.h>

//Likely User Modified Variables ******************************

unsigned int deltaT = 1000;         // Sample period in microseconds.
int angleAverages = 3;
int past_size = 3;
// interval for delta, larger=less noise, more delay.
unsigned long transferDt = 20000; // usecs between host updates  
float Kp, Kd, Kbemf, Ku, Ki, directV, desiredAngle;
                                 
// ***************************************************************

// Teensy definitions
#define angleSensorPin  A9
#define pwmVoltagePin A1
#define motorVoltagePin  A2
#define motorOutPWM  9
#define monitorPin 2  
#define dacRes 12   // Teensy 12-bit dac resolution, max val 4095
#define dacMax 4095
#define adcRes 14
#define adcCenter 8192 //Teensy ADC set to 14 bits, 16384/2 = 8192
#define adcMax 16383

// Circuit Specific Definitions.
#define scaleVadc 5.0 // All signals 0->5 volts.
#define vDrive 5.0 // Driver peak voltage.
#define rShunt 1.0 // Resistor from motor to ground.
#define rMotor 1.0 // Motor series resistance.

// Variables for Conversion
#define deg2rad 0.0175    
#define twopi 6.2831853  

// Variables for standing robot
#define ANGULAR_VELOCITIES_COUNT 10

//Rest of Setup:
bool first_time;
String config_message  = "&A~Desired~5&C&S~K_P~P~0~100~0.1&S~K_D~D~0~10~0.05&S~K_I~I~0~100~0.01&S~SumMax~S~0~50~1&S~Direct~O~0~5~0.01&S~Desired~A~-2.5~2.5~0.01&S~alpha~L~0~1~0.01&T~Phi~F4~-2.5~2.5&T~AngleError~F4~-5~5&T~AngularVelocityError~F4~-5~5&T~Sum~F4~-50~50&T~MotorCmd~F4~0~5&H~4&";

float rad2deg = 1.0/deg2rad;        // 180/pi
 
float dTsec = 1.0e-6*deltaT;       // The period in seconds. 
float scaleD = 1.0/(dTsec*past_size); // Divide deltas by interval.   
                             
float angleErrorIntegral = 0;                // Variable for integrating angle errors.
float integralMax = 0;            // Maximum value of the integral

int loopCounter;

// Storage for past values.
float pastErrorV[20];  // Should be larger array than past_size.
char buf[60];  // 

// Variables for loop control
uint32_t loop_counter;
int numSkip;  // Number of loops to skip between host updates.
elapsedMicros loopTime; // Create elapsed time variable to ensure regular loops.
unsigned int headroom;  // Headroom is time left before loop takes too long.
boolean switchFlag;

// Variables for standing robot
MPU9250 imu;
char accell[100];
char gyroo[100];
char magoo[100];
float phi = 0;
char phiString[20];
float angularVelocities[ANGULAR_VELOCITIES_COUNT]; // in rad/s
float alpha;
float ax;
float az;

// Initializes past values.
void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
  // Set up inputs
  analogReadResolution(adcRes);
  pinMode(motorVoltagePin, INPUT);
  pinMode(pwmVoltagePin, INPUT);
  pinMode(angleSensorPin, INPUT);

  // Set up output
  analogWriteResolution(dacRes);
  pinMode(motorOutPWM, OUTPUT);
  analogWriteFrequency(motorOutPWM, 23437.5); // Teensy 3.0 pin 3 also changes to 23437.5 kHz
  analogWrite(motorOutPWM, LOW);
  
  // Frequency Monitor
  pinMode(monitorPin, OUTPUT);

  // Number of loops between transfers to host.
  numSkip = max(int((transferDt+100)/deltaT),1); 

  first_time = false;

  /*************** Initializing gyroscope **********************/
  // Initiate the Wire library and join the I2C bus as a master or slave
  Wire.begin();
  delay(500); //wait 500ms
  byte c = imu.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
  //Serial.println("HIHI");
  //Serial.println("HEY");
  c = imu.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  //Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
  //Serial.print(" I should be "); Serial.println(0x73, HEX);
  if (c == 0x73) // WHO_AM_I   //IF YOU HAVE A BLUE IMU CHANGE TO 0x73
  {
    //Serial.println("MPU9250 is online...");
    imu.MPU9250SelfTest(imu.selfTest);
    // Calibrate gyro and accelerometers, load biases in bias registers
    imu.initMPU9250();
    imu.calibrateMPU9250(imu.gyroBias, imu.accelBias);
    imu.initMPU9250();
    imu.initAK8963(imu.factoryMagCalibration);
    } // if (c == 0x73)
  else
  {
    //Serial.print("Could not connect to MPU9250: 0x");
    //Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }
  pinMode(5,INPUT_PULLUP);
  imu.getAres();
  imu.getGres();
  imu.getMres();

}

void loop() {  // Main code, runs repeatedly
  // Reinitializes or updates from sliders on GUI.
  startup();
  
  // Make sure loop starts deltaT microsecs since last start
  unsigned int newHeadroom = max(int(deltaT) - int(loopTime), 0);
  headroom = min(headroom, newHeadroom);
  
  while (int(loopTime) < int(deltaT)) {};
  loopTime = 0;
  
  // Monitor Output should be a square wave with frequency = 1/(2*deltaT) 
  switchFlag = !switchFlag;
  digitalWrite(monitorPin, switchFlag);

  /*************** IMU Data **********************/
  // acceleration in Gs
  imu.readAccelData(imu.accelCount);
  ax = (-imu.accelCount[0]*imu.aRes)*alpha + ax*(1-alpha);
  az = (-imu.accelCount[2]*imu.aRes)*alpha + az*(1-alpha);
  // gyro readings in degrees per second
  imu.readGyroData(imu.gyroCount);
  float gy = imu.gyroCount[1]*imu.gRes;
  phi = atan2(az, ax);

  // Update angular velocities
  for (uint8_t i=(ANGULAR_VELOCITIES_COUNT-1); i>0; i--) {
    angularVelocities[i] = angularVelocities[i-1];
  }
  angularVelocities[0] = gy * deg2rad;

  // Get average angular velocity
  float angularVelocitySum = 0;
  for (uint8_t i=0; i<ANGULAR_VELOCITIES_COUNT; i++) {
    angularVelocitySum += angularVelocities[i];
  }
  float averageAngularVelocity = angularVelocitySum/ANGULAR_VELOCITIES_COUNT;
 
  /*************** Section of Likely User modifications.**********************/
  // Read Angle, average to reduce noise.
  int angleI = 0;
  for(int i = 0; i < angleAverages; i++) angleI += analogRead(angleSensorPin);
  float angleV = scaleVadc*float(angleI- angleAverages*adcCenter)/float(angleAverages*adcMax);

  // Compute error, and error deltas from past.
  float angleError = (desiredAngle - phi);
  float angularVelocityError = -averageAngularVelocity;
  angleErrorIntegral = min(max(angleErrorIntegral + angleError*dTsec,-1*integralMax),integralMax);

  // Compute and write out motor command.
  float motorCmd = directV + Kp*angleError + Kd*angularVelocityError + Ki*angleErrorIntegral;     
  float motorCmdLim = min(max(motorCmd, 0), vDrive);
  analogWrite(motorOutPWM,int((motorCmdLim/vDrive)*dacMax));
  //analogWrite(A14,int((motorCmdLim/vDrive)*dacMax));
  
  // Update previous errors for next time.
  for (int i = past_size-1; i > 0; i--) pastErrorV[i] = pastErrorV[i-1];
  pastErrorV[0] = angleError;
  
  if (loopCounter == numSkip) {  // Lines below are for debugging.
    packStatus(buf, phi, angleError, angularVelocityError, angleErrorIntegral, float(motorCmd), float(headroom));
    Serial.write(buf,26);
    loopCounter = 0;
    headroom = deltaT;
  } else loopCounter += 1;
}

void init_loop() {
  // Initialize loop variables
  loopCounter = 0; 
  headroom = deltaT;

  // Zero past errors
  for (int i = past_size-1; i >= 0; i--) pastErrorV[i] = 0;
}


// Initializes the loop if this is the first time, or if reconnect sent
// from GUI.  Otherwise, just look for serial event.
void startup(){
  if (first_time) {
    while(Serial.available() > 0) Serial.read(); // Clear out rcvr.
    Serial.println(config_message);  // Send the configuration files.
    while (! Serial.available()) {}  // Wait for serial return.
    while(Serial.available() > 0) {   // Clear out rcvr.
        Serial.read();
        //char inChar = (char) Serial.read(); 
        //if (inChar == '\n') break;
    }
    init_loop();
    first_time = false;
  } else {
    serialEvent();
  }
}


// Simple serial event, only looks for disconnect character, resets loop if found.
void serialEvent() {
  String inputString = ""; 
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    // if the incoming character is a newline, ready to process.
    if (inChar == '\n') {
      processString(inputString);
      inputString = "";
      break;
    }
  }
}

void processString(String inputString) {
char St = inputString.charAt(0);
  inputString.remove(0,1);
  float val = inputString.toFloat();
  switch (St) {
    case 'P': 
      Kp = val;
      break;
    case 'D':
      Kd = val;
      break;  
    case 'E':
      Kbemf = val;
      break;
    case 'U':
      Ku = val;
      break;
    case 'I':
      Ki = val;
      break;  
    case 'S':
      integralMax = val;
      break;
    case 'O':  
      directV = val;
      break;
    case 'A':
      desiredAngle = val;
      break;
    case 'L':
      alpha = val;
      break;
    case '~':
      first_time = true;
      break;
    default:
    break;  
  }
}

// Load the serial output buffer.
void packStatus(char *buf, float a, float b, float c, float d, float e, float f) {
  
  // Start Byte.
  buf[0] = byte(0);
  int n = 1; 
  
  memcpy(&buf[n],&a,sizeof(a));
  n+=sizeof(a);
  memcpy(&buf[n],&b,sizeof(b));
  n+=sizeof(b);
  memcpy(&buf[n],&c,sizeof(c));
  n+=sizeof(c);
  memcpy(&buf[n],&d,sizeof(d));
  n+=sizeof(d);
  memcpy(&buf[n],&e,sizeof(e));
  n+=sizeof(e);
  memcpy(&buf[n],&f,sizeof(f));
  n+=sizeof(f);
  /*
  memcpy(&buf[n],&g,sizeof(g));
   n+=sizeof(g);
   */

  // Stop byte (255 otherwise unused).
  buf[n] = byte(255); 
}

