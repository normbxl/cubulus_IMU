#include <Ports.h>
#include <RF12.h>

#define RQS_STATE 0x10
#define RQS_CALIBRATE 0x16
#define RQS_VIBRATE 0x20
#define RQS_SEND_DATA 0x21
#define RQS_RESET 0x22
#define RQS_STANDBY 0x23

#define PCK_DATA_TYPE 0x01
#define PCK_STATE_TYPE 0x02

#define PIN_LED 8

#define STATE_UPDATE_INTERVAL 10000 // alle 10 Sekunden Status abfragen

#define BINARY_OUTPUT

boolean led_on = false;

int acc[3];
float gyro[4];
float quaternion[4];

float payload_send_ts=0;

char str[64];

unsigned long state_req_ts=0;
unsigned long last_read_ts=0;

boolean standby=false;

byte payload[2] = {0x00, 0x00};
byte rs_buffer[2];
boolean sendPayload=false;

typedef struct {
  char packetType; // 0x01
  int accX, accY, accZ;
  float gyroX, gyroY, gyroZ;
  float Q[4];
} rf12DataPacket;

typedef struct {
  char packetType;  // 0x02
  char lowBatt;
  float temp;
  unsigned int sampleT;
} rf12StatePacket;

rf12DataPacket *data;
rf12StatePacket *stateData;

void setup () {
    rf12_config();
    Serial.begin(57600);
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);
}

void loop () {
  if (rf12_recvDone() && rf12_crc == 0) {
    led_on = led_on == HIGH ? LOW : HIGH;
    digitalWrite(PIN_LED, led_on);
    // a packet has been received -> rf12_data
    
    if (rf12_hdr && (rf12_hdr & RF12_HDR_CTL) == RF12_HDR_CTL) {
      payload_send_ts=0;
    }
    else {
      int c=0;
      boolean isStatePacket = false;
      int pSize;
      //Serial.print("PCD");
      //Serial.println(rf12_data[0], HEX);
      switch ((byte)rf12_data[0]) {
        case PCK_DATA_TYPE:
          data = (rf12DataPacket*) rf12_data;
          pSize = sizeof(rf12DataPacket);
          //sendSerialData();
          
          break;
        case PCK_STATE_TYPE:
          stateData = (rf12StatePacket*) rf12_data;
          pSize = sizeof(rf12StatePacket);
          isStatePacket = true;
          state_req_ts = millis() + STATE_UPDATE_INTERVAL;
          break;
      }
      
      if (isStatePacket) {
        isStatePacket = false;
      }
      // somit wird ein RQS_STANDBY erzwungen
      standby=false;
    }
  }
  else if (payload_send_ts > 0 && payload_send_ts+100 < millis()) {
    sendPayload = true;
  }
  else if (state_req_ts < millis() && !standby) {
    payload[0] = RQS_STATE;
    payload[1]=0x00;
    state_req_ts = millis() + STATE_UPDATE_INTERVAL;
    sendPayload=true;
  }
  if (Serial.available()>0) {
    boolean use_buffer=true;
    rs_buffer[1]=0x00;
    for(int i=0; Serial.available()>0 && i<2; i++) {
      rs_buffer[i]= Serial.read();
      delay(1);
    }
    switch (rs_buffer[0]) {
      case RQS_VIBRATE:
        sendPayload=true;
        break;
      case RQS_SEND_DATA:
        sendSerialData();
        use_buffer=false;
        break;
      case RQS_STANDBY:
        sendPayload=true;
        break;
      case RQS_STATE:
        sendSerialStateData();
        use_buffer=false;
        break;
      case RQS_CALIBRATE:
        sendPayload=true;
        rs_buffer[1] = 0x00;
        sendSerialData();
        break;
    }
    last_read_ts=millis();
    if (standby && rs_buffer[0] != RQS_STANDBY) {
      standby=false;
    }
    if (use_buffer) {
      memcpy(payload, rs_buffer, 2);
    }
    Serial.flush();
  }
  
  if (!standby && millis()-last_read_ts > 10000) {
    payload[0] = RQS_STANDBY;
    payload[1] = 0x00;
    sendPayload=true;
    standby=true;
  }
  
  if (sendPayload) {
    if (rf12_canSend()) {
      if ( payload[0] == RQS_STATE) {
        rf12_sendStart(0, &payload, 2);
        payload_send_ts = 0;
      }
      else {
        rf12_sendStart(RF12_HDR_ACK, &payload, 2);
        payload_send_ts = millis();
      }
      sendPayload=false;
    }
  }
}

#ifdef BINARY_OUTPUT
void sendSerialData() {
  byte* sendPacket = (byte*) data->Q;
  Serial.write(0x01);
  Serial.write(sendPacket, 16);
}
void sendSerialStateData() {
  Serial.write(0x02);
  Serial.write((byte)stateData->lowBatt);
  Serial.write((byte*)&stateData->temp,4);
}
#endif
#ifndef BINARY_OUTPUT
void sendSerialData() {
  sprintf(str, ",%d,%d,%d,", data->accX, data->accY, data->accZ);
  Serial.print(0x01);
  Serial.print(str);
  printFloat(&data->gyroX, ',');
  printFloat(&data->gyroY, ',');
  printFloat(&data->gyroZ, ',');
  
  printFloat(&data->Q[0], ',');
  printFloat(&data->Q[1], ',');
  printFloat(&data->Q[2], ',');
  printFloat(&data->Q[3], ',');
  Serial.print('\n');
}
void sendSerialStateData() {
  Serial.print(0x02, DEC);
  Serial.print(',');
  Serial.print(stateData->lowBatt, DEC);
  Serial.print(',');
  Serial.println(stateData->temp);
}
#endif
