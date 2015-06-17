
#include <Wire.h> // I2C library, gyroscope

#include <RF12.h>
#include <Ports.h>
#include "IIRfilter.h"

#include <avr/wdt.h>

//#define USE_AHRS_UPDATE

//#define DEBUG

// Accelerometer ADXL345
#define ACC (0x53)    //ADXL345 ACC address
#define A_TO_READ (6)        //num of bytes we are going to read each time (two bytes for each axis)

// Gyroscope ITG3200 
#define GYRO 0x68 // gyro address, binary = 11101001 when AD0 is connected to Vcc (see schematics of your breakout board)
#define G_SMPLRT_DIV 0x15
#define G_DLPF_FS 0x16
#define G_INT_CFG 0x17
#define G_PWR_MGM 0x3E

#define G_TO_READ 8 // 2 bytes for each axis x, y, z

#define DEG2RAD  0.01745329251994329577f

#define RQS_STATE 0x10
#define RQS_CALIBRATE 0x16
#define RQS_VIBRATE 0x20
#define RQS_RESET 0x22
#define RQS_STANDBY 0x23

#define PIN_FEEDBACK 3

#define GY_MY -1.8498942917547f
#define GY_MZ 1.906f

float gyroY_Toff, gyroZ_Toff;

float gyroMeasError = 3.14159265358979f * (5.0f / 180.0f); // gyroscope measurement error in rad/s (shown as 5 deg/s)
float beta = sqrt(3.0f / 4.0f) * gyroMeasError; // compute beta

float deltat; // Sampleperiode

// Magdwick Error correction ///
#define Kp 2.0f		// proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.005f	// integral gain governs rate of convergence of gyroscope biases

float exInt = 0, eyInt = 0, ezInt = 0;	// scaled integral error
///////////////////////////////


// Die Offsets werden jetzt mit zeroCalibrate gemessen
int arr_g_offsets[3];
float gyroData[4];
int accData[3];

float temp_offset; 


// initialize quaternion
// estimated orientation quaternion elements with initial conditions
// SE ->Quaternion: Earth frame relative to Sensor frame
float SEq_1 = 1.0f, SEq_2 = 0.0f, SEq_3 = 0.0f, SEq_4 = 0.0f; 
float dfilter_1[2] = { 
  0.999975456909767f, -0.999975456909767f };
float dfilter_2[2] = { 
  1.0f, -0.999950913819534f };

IIRfilter HPfilterp = IIRfilter(dfilter_1, dfilter_2);      // 0.001Hz 1st order HP filter
IIRfilter HPfilterq = IIRfilter(dfilter_1, dfilter_2);      // 0.001Hz 1st order HP filter
IIRfilter HPfilterr = IIRfilter(dfilter_1, dfilter_2);      // 0.001Hz 1st order HP filter

unsigned long
lastUpdate = 0,
now = 0,
ts_feedback_timeout=0;



// Paket-Struktur zum Versenden
typedef struct {
  char packetType;
  int accX, accY, accZ;
  float gyroX, gyroY, gyroZ;
  float Q[4];
} 
rf12DataPacket;

typedef struct {
  char packetType;
  char lowBatt;
  float temp;
  unsigned int sampleT;
} 
rf12StatePacket;



rf12DataPacket dPacket;
rf12StatePacket sPacket;

unsigned long lastSendTS=0;

MilliTimer sendTimer;

boolean standby=false;
boolean needToSend=false;
boolean sendStatePacket = false;


void initAcc() {
  //Turning on the ADXL345
  writeTo(ACC, 0x2D, 0);      
  writeTo(ACC, 0x2D, 16);
  writeTo(ACC, 0x2D, 8);
  //by default the device is in +-2g range reading
}

void sensorsStandby() {
  //  SLEEP - MEASUREMENT OFF - 2Hz Sleep Reading
  writeTo(ACC, 0x2D, B00001101);
  // Gyro Sleep - (X - Y - Z) Sleep
  writeTo(GYRO, G_PWR_MGM, B01111000);
}

void wakeupSensors() {
  initAcc();
  writeTo(GYRO, G_PWR_MGM, 0x00);
}


void getAccelerometerData(int *ax, int *ay, int *az) {
  int regAddress = 0x32;    //first axis-acceleration-data register on the ADXL345
  byte buff[A_TO_READ];

  readFrom(ACC, regAddress, A_TO_READ, buff); //read the acceleration data from the ADXL345

  //each axis reading comes in 10 bit resolution, ie 2 bytes.  Least Significat Byte first!!
  //thus we are converting both bytes in to one int
  *ax = (((int)buff[1]) << 8) | buff[0];   
  *ay = (((int)buff[3]) << 8) | buff[2];
  *az = (((int)buff[5]) << 8) | buff[4];
}

//initializes the gyroscope
void initGyro()
{
  /*****************************************
   * ITG 3200
   * power management set to:
   * clock select = internal oscillator
   *     no reset, no sleep mode
   *   no standby mode
   * sample rate to = 125Hz
   * parameter to +/- 2000 degrees/sec
   * low pass filter = 5Hz
   * no interrupt
   ******************************************/
  writeTo(GYRO, G_PWR_MGM, 0x00);
  writeTo(GYRO, G_SMPLRT_DIV, 0x07); // EB, 50, 80, 7F, DE, 23, 20, FF
  writeTo(GYRO, G_DLPF_FS, 0x1E); // +/- 2000 dgrs/sec, 1KHz, 1E, 19
  writeTo(GYRO, G_INT_CFG, 0x00);
}


void getGyroscopeData(float *gx, float *gy, float *gz, float *t) {
  int buffer[4];

  readGyroRaw(buffer);

  *t =  35.0 + ((float)buffer[3] + 13200.0f) / 280.0f;

  *gx =  (float)(buffer[0]+arr_g_offsets[0]) / 14.375;
  *gy =  (float)(buffer[1]+arr_g_offsets[1] + (GY_MY * *t + gyroY_Toff)) / 14.375;
  *gz =  (float)(buffer[2]+arr_g_offsets[2] + (GY_MZ * *t + gyroZ_Toff)) / 14.375;

  *gx = HPfilterp.step(*gx);
  *gy = HPfilterq.step(*gy);
  *gz = HPfilterr.step(*gz);
}


void readGyroRaw(int *b) {
  readGyroRaw(b, b+1, b+2, b+3);
}

void readGyroRaw(int *_GyroX, int *_GyroY, int *_GyroZ, int *_Temp){
  int regAddress = 0x1B;
  byte buff[G_TO_READ];

  readFrom(GYRO, regAddress, G_TO_READ, buff);
  *_Temp  = (buff[0] << 8) | buff[1];
  *_GyroX = (buff[2] << 8) | buff[3];
  *_GyroY = (buff[4] << 8) | buff[5]; 
  *_GyroZ = (buff[6] << 8) | buff[7];
}


void zeroCalibrate(unsigned int totSamples, unsigned int sampleDelayMS) {
  int xyzt[4]; 
  float tmpOffsets[] = {
    0,0,0      };

  for (int i = 0;i < totSamples;i++){
    delay(sampleDelayMS);
    readGyroRaw(xyzt);
    tmpOffsets[0] += xyzt[0];
    tmpOffsets[1] += xyzt[1];
    tmpOffsets[2] += xyzt[2];  
  }
  arr_g_offsets[0] = -tmpOffsets[0] / totSamples;
  arr_g_offsets[1] = -tmpOffsets[1] / totSamples;
  arr_g_offsets[2] = -tmpOffsets[2] / totSamples;
  temp_offset = 35.0 + ((float)xyzt[3] + 13200.0f) / 280.0f;

  gyroY_Toff = GY_MY * -temp_offset;
  gyroZ_Toff = GY_MZ * -temp_offset;
}

#ifdef USE_AHRS_UPDATE
void AHRSupdate(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z) {
  float norm; // vector norm
  float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion derrivative from gyroscopes elements
  float f_1, f_2, f_3; // objective function elements
  float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
  float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; // estimated direction of the gyroscope error
  // Axulirary variables to avoid reapeated calcualtions
  float halfSEq_1 = 0.5f * SEq_1;
  float halfSEq_2 = 0.5f * SEq_2;
  float halfSEq_3 = 0.5f * SEq_3;
  float halfSEq_4 = 0.5f * SEq_4;
  float twoSEq_1 = 2.0f * SEq_1;
  float twoSEq_2 = 2.0f * SEq_2;
  float twoSEq_3 = 2.0f * SEq_3;        

  now = millis();
  deltat = (now - lastUpdate) / 1000.0;
  lastUpdate = now;

  // Normalise the accelerometer measurement
  norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
  a_x /= norm;
  a_y /= norm;
  a_z /= norm;

  // Compute the objective function and Jacobian
  f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
  f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
  f_3 = 1.0f - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;
  J_11or24 = twoSEq_3; // J_11 negated in matrix multiplication
  J_12or23 = 2.0f * SEq_4;
  J_13or22 = twoSEq_1; // J_12 negated in matrix multiplication
  J_14or21 = twoSEq_2;
  J_32 = 2.0f * J_14or21; // negated in matrix multiplication
  J_33 = 2.0f * J_11or24; // negated in matrix multiplication

  // Compute the gradient (matrix multiplication)
  SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1;
  SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
  SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
  SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2;

  // Normalise the gradient
  norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
  SEqHatDot_1 /= norm;
  SEqHatDot_2 /= norm;
  SEqHatDot_3 /= norm;
  SEqHatDot_4 /= norm;

  // Compute the quaternion derrivative measured by gyroscopes
  SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
  SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
  SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
  SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;

  // Compute then integrate the estimated quaternion derrivative
  SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat;
  SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat;
  SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat;
  SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat;

  // Normalise quaternion
  norm = sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
  SEq_1 /= norm;
  SEq_2 /= norm;
  SEq_3 /= norm;
  SEq_4 /= norm;
}
#endif
#ifndef USE_AHRS_UPDATE
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az) {
  float norm;
  float vx, vy, vz;
  float ex, ey, ez;         
  
  now = millis();
  deltat = (now - lastUpdate) / 2000.0;
  lastUpdate = now;
  
  // normalise the measurements
  norm = sqrt(ax*ax + ay*ay + az*az);       
  ax = ax / norm;
  ay = ay / norm;
  az = az / norm;      

  // estimated direction of gravity
  vx = 2*(SEq_2*SEq_4 - SEq_1*SEq_3);
  vy = 2*(SEq_1*SEq_2 + SEq_3*SEq_4);
  vz = SEq_1*SEq_1 - SEq_2*SEq_2 - SEq_3*SEq_3 + SEq_4*SEq_4;

  // error is sum of cross product between reference direction of field and direction measured by sensor
  ex = (ay*vz - az*vy);
  ey = (az*vx - ax*vz);
  ez = (ax*vy - ay*vx);

  // integral error scaled integral gain
  exInt = exInt + ex*Ki;
  eyInt = eyInt + ey*Ki;
  ezInt = ezInt + ez*Ki;

  // adjusted gyroscope measurements
  gx = gx + Kp*ex + exInt;
  gy = gy + Kp*ey + eyInt;
  gz = gz + Kp*ez + ezInt;

  // integrate quaternion rate and normalise
  SEq_1 = SEq_1 + (-SEq_2*gx - SEq_3*gy - SEq_4*gz)*deltat;
  SEq_2 = SEq_2 + (SEq_1*gx + SEq_3*gz - SEq_4*gy)*deltat;
  SEq_3 = SEq_3 + (SEq_1*gy - SEq_2*gz + SEq_4*gx)*deltat;
  SEq_4 = SEq_4 + (SEq_1*gz + SEq_2*gy - SEq_3*gx)*deltat;  

  // normalise quaternion
  norm = sqrt(SEq_1*SEq_1 + SEq_2*SEq_2 + SEq_3*SEq_3 + SEq_4*SEq_4);
  SEq_1 = SEq_1 / norm;
  SEq_2 = SEq_2 / norm;
  SEq_3 = SEq_3 / norm;
  SEq_4 = SEq_4 / norm;
}
#endif
void getQ(float * q) {
  // gyro values are expressed in deg/sec, the * M_PI/180 will convert it to radians/sec
  #ifdef USE_AHRS_UPDATE
    AHRSupdate(gyroData[0] * DEG2RAD, gyroData[1] * DEG2RAD, gyroData[2] * DEG2RAD, accData[0], accData[1], accData[2]);
  #endif
  
  #ifndef USE_AHRS_UPDATE
    IMUupdate(gyroData[0] * DEG2RAD, gyroData[1] * DEG2RAD, gyroData[2] * DEG2RAD, accData[0], accData[1], accData[2]);
  #endif
  
  q[0] = SEq_1;
  q[1] = SEq_2;
  q[2] = SEq_3;
  q[3] = SEq_4;
}

void setup() {
  dPacket.packetType = 0x01;
  sPacket.packetType = 0x02;
#ifdef DEBUG
  Serial.begin(57600);
#endif
  pinMode(PIN_FEEDBACK, OUTPUT);
  // ist ein PNP der den Motor steuert
  digitalWrite(PIN_FEEDBACK, HIGH);

  Wire.begin();
  initAcc();
  initGyro();
  zeroCalibrate(64, 5);
  rf12_config();
  standby=false;
  // Watchdog-Timer auf 1s gestellt
  wdt_enable(WDTO_1S);
}


void loop() {
  char str[42];

  uint8_t i,c;

  // Watchdog Still-Alive-Signal
  wdt_reset();

  // Feedback (Motor) abschalten
  if (ts_feedback_timeout != 0 && ts_feedback_timeout < millis()) {
    digitalWrite(PIN_FEEDBACK, HIGH);
  }

  if (!standby) {

    getGyroscopeData(&gyroData[0], &gyroData[1], &gyroData[2], &gyroData[3]);
    getAccelerometerData(&accData[0], &accData[1], &accData[2]);

    dPacket.gyroX = gyroData[0];
    dPacket.gyroY = gyroData[1];
    dPacket.gyroZ = gyroData[2];

    dPacket.accX = accData[0];
    dPacket.accY = accData[1];
    dPacket.accZ = accData[2];

    getQ(dPacket.Q);
  }

  if (sendTimer.poll(25)) {
    needToSend = true;
  }

  if (rf12_recvDone() && rf12_crc == 0) {
    // a packet has been received
#ifdef DEBUG
    Serial.print("REQ ");
    Serial.println(rf12_data[0], HEX);
#endif
    // ACK gewünscht?
    if ((rf12_hdr & RF12_HDR_ACK )== RF12_HDR_ACK) {
      rf12_sendStart(RF12_HDR_CTL, 0, 0);
#ifdef DEBUG
      Serial.println("send ACK");
#endif
    }

    switch (rf12_data[0]) {
    case RQS_STATE:
      sPacket.temp = gyroData[3];
      sPacket.lowBatt = rf12_lowbat();
      sPacket.sampleT = (unsigned int)(deltat*1000.0);
      sendStatePacket = true;
      break;
    case RQS_VIBRATE:
      // zweites Byte ist die Dauer des Feedbacks in 1/10 Sekunden
      digitalWrite(PIN_FEEDBACK, LOW);
      ts_feedback_timeout = millis() + (byte)rf12_data[1] * 10;

#ifdef DEBUG
      Serial.print("VIBRATE: ");
      Serial.print((byte)rf12_data[1]);
#endif

      break;
    case RQS_CALIBRATE:
      SEq_1 = 1.0f, SEq_2 = 0.0f, SEq_3 = 0.0f, SEq_4 = 0.0f;
      zeroCalibrate(64, 5);

#ifdef DEBUG
      Serial.println("Calibrate request executed");
#endif

      break;
    case RQS_RESET:
      SEq_1 = 1.0f, SEq_2 = 0.0f, SEq_3 = 0.0f, SEq_4 = 0.0f; 
      HPfilterp.init(dfilter_1, dfilter_2);
      HPfilterq.init(dfilter_1, dfilter_2);
      HPfilterr.init(dfilter_1, dfilter_2);
      setup();
      break;
    case RQS_STANDBY:
      sensorsStandby();
      standby=true;
#ifdef DEBUG
      Serial.println("Standby");
#endif
      break;
    }


    if (rf12_data[0] != RQS_STANDBY && standby) {
#ifdef DEBUG
      Serial.println("Wakeup");
#endif
      SEq_1 = 1.0f, SEq_2 = 0.0f, SEq_3 = 0.0f, SEq_4 = 0.0f;
      wakeupSensors();
      standby=false;
      lastUpdate=millis();
      delay(1);
    }
  }


  if ((needToSend ||sendStatePacket) && standby==false ){ //&& rf12_canSend()) {
    if (rf12_canSend()) {
      needToSend=false;
      if (sendStatePacket) {
#ifdef DEBUG
        Serial.println("SEND state-packet");
#endif
        rf12_sendStart(0, &sPacket, sizeof(sPacket));
        sendStatePacket=false;
      }
      else {
        // #ifdef DEBUG
        // Serial.println("data");
        // #endif
        rf12_sendStart(0, &dPacket, sizeof(dPacket));
      }
      lastSendTS = millis();
    }
  }
}


//---------------- Functions
//Writes val to address register on ACC
void writeTo(int DEVICE, byte address, byte val) {
  Wire.beginTransmission(DEVICE); //start transmission to ACC 
  Wire.send(address);        // send register address
  Wire.send(val);        // send value to write
  Wire.endTransmission(); //end transmission
}


//reads num bytes starting from address register on ACC in to buff array
void readFrom(int DEVICE, byte address, int num, byte buff[]) {
  Wire.beginTransmission(DEVICE); //start transmission to ACC 
  Wire.send(address);        //sends address to read from
  Wire.endTransmission(); //end transmission

  Wire.beginTransmission(DEVICE); //start transmission to ACC
  Wire.requestFrom(DEVICE, num);    // request 6 bytes from ACC

  int i = 0;
  while(Wire.available())    //ACC may send less than requested (abnormal)
  { 
    buff[i] = Wire.receive(); // receive a byte
    i++;
  }
  Wire.endTransmission(); //end transmission
}
