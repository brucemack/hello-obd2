/*
Very simple OBD2 scanner for ISO 9141-2 vehicles
Bruce MacKinnon 05-Mar-2023

Uses the Teensy 3.2 since we need harware serial (UART).

This was tested on a 2004 Toyota Corolla and a 1999 Honda Civic.

Example RPM Query
Sent:
68 6a f1 1 c d0

Recd:
48 6b 10 41 c 18 f 37
*/

// The serial UART TX and RX pins used to communicate with the vehicle.
const byte rxPin = 0;
const byte txPin = 1;
// This is another pin that can pull the K-Line low independently of the UART
const byte ctlPin = 2;

const byte LED_PIN = 13;

const long RX_TIMEOUT_MS = 100;
const long W1 = 20;
const long W4 = 30;

int state = 0;
long stamp0 = 0;
long lastActivityStamp = 0;
byte kw0 = 0;
byte kw1 = 0;

bool diag = false;

// Number of received byte to ignore - used to eliminate
// process of bytes that we send on the K-Line (i.e. echo
// ignore).
int ignoreCount = 0;

// This is the buffer that we use to accumulate received data
// from the vehicle.
const int MAX_RX_MSG_LEN = 256;
byte rxMsg[MAX_RX_MSG_LEN];
int rxMsgLen = 0;

// Used for looping through multiple queries
int cycle = 0;

void doReboot() {
  SCB_AIRCR = 0x05FA0004;
}

/**
 * Sends the ISO9141-2 compliant 5-baud initilization 
 * sequence.  This is a hex 33 with one start bit and one stop bit.
 * LSB is transmitted first. 
 * 
 * Total time is 5.5 seconds to force EC2 timeout and 2 seconds to send sequence.
 */
static void generateFiveBaudInit() {

  int logLow = 1;
  int logHigh = 0;
  
  // Let things idle long enough to time out the ECU
  delay(5500);
  // Start bit
  digitalWrite(ctlPin, logLow);
  delay(200);
  // Send 00110011
  digitalWrite(ctlPin, logHigh);
  delay(200);
  delay(200);
  digitalWrite(ctlPin, logLow);
  delay(200);
  delay(200);
  digitalWrite(ctlPin, logHigh);
  delay(200);
  delay(200);
  digitalWrite(ctlPin, logLow);
  delay(200);
  delay(200);
  // Stop bit (idle state)
  digitalWrite(ctlPin, logHigh);
  delay(200);
}

// Inspired by SternOBDII\code\checksum.c
static byte iso_checksum(byte *data, byte len) {
  byte crc = 0;
  for (byte i = 0; i < len; i++)
    crc = crc + data[i];
  return crc;
}

// ----- PID formatting functions -------------------------------------------------

// PIDS supported
static void format_01_00(const byte* m, int mLen, char* buf, int basePid) {
  int p = 0;
  int pid = basePid;  
  for (int i = 0; i < 4; i++) {
    for (int k = 0; k < 8; k++) {
      if (m[5 + i] & 0x80) {
        char l[3];
        sprintf(l, "%02x", pid);
        buf[p] = l[0];
        buf[p+1] = l[1];
        buf[p+2] = ' ';
        p += 3;
      }
      k = k << 1;
      pid++;
    }
  }
  buf[p] = 0;
}

// Monitor status
static void format_01_01(const byte* m, int mLen, char* buf) {
  unsigned int a = m[5];
  unsigned int b = m[6];
  unsigned int c = m[7];
  unsigned int d = m[8];
  sprintf(buf,"%02x %02x %02x %02x", a, b, c, d);
}

// Coolant temp
static void format_01_05(const byte* m, int mLen, char* buf) {
  int a = m[5];
  int d = a - 50;
  sprintf(buf,"%d degrees C", (int)d);
}

// Engine speed
static void format_01_0c(const byte* m, int mLen, char* buf) {
  unsigned int rpm = ((m[5] * 256) + m[6]) / 4;
  sprintf(buf,"%d rpm",(int)rpm);
}

// Vehicle speed
static void format_01_0d(const byte* m, int mLen, char* buf) {
  int a = m[5];
  sprintf(buf,"%d km/h", (int)a);
}

// Timing advance
static void format_01_0e(const byte* m, int mLen, char* buf) {
  int a = m[5];
  int d = (a / 2) - 64;
  sprintf(buf,"%d degrees", (int)d);
}

// Throttle position
static void format_01_11(const byte* m, int mLen, char* buf) {
  int a = m[5];
  int d = (a * 100) / 255;
  sprintf(buf,"%d percent", (int)d);
}

void configure() {

  Serial.println("INFO: Connecting to vehicle");

  // Send the 5-baud init
  generateFiveBaudInit();
    
  // Clear any junk received on the serial port during the W1 interval
  long s0 = millis();
  while (millis() - s0 < W1) {
    if (Serial1.available()) {
      Serial1.read();
    }
  }
        
  state = 0;
  stamp0 = millis();    
  lastActivityStamp = millis();
  ignoreCount = 0;
  cycle = 0;
  rxMsgLen = 0;
}

void unconfigure() {
  Serial.println("INFO: Reset");
}

void setup() {

  // Console
  Serial.begin(9600);

  pinMode(ctlPin, OUTPUT);
  // Idle state for CTL is low (inverted!)
  digitalWrite(ctlPin, LOW);
  pinMode(LED_PIN, OUTPUT);
  // Idle state for the LED pin
  digitalWrite(LED_PIN, LOW);

  // Stobe LED so that we know things are running
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
  delay(500);
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
  delay(500);

  Serial.println("INFO: OBD2 Diagnostic Scanner V2.03");

  // Set the baud rate for the ISO9141 serial port
  Serial1.begin(10400);

  configure();
}

const int introRequestCount = 7;

byte introRequests[7][7] = {
  { 6,  0x68, 0x6a, 0xf1, 0x1, 0x01, 0x00 },
  { 6,  0x68, 0x6a, 0xf1, 0x1, 0x00, 0x00 },
  { 6,  0x68, 0x6a, 0xf1, 0x1, 0x20, 0x00 },
  { 6,  0x68, 0x6a, 0xf1, 0x1, 0x40, 0x00 },
  { 6,  0x68, 0x6a, 0xf1, 0x1, 0x60, 0x00 },  
  { 6,  0x68, 0x6a, 0xf1, 0x1, 0x80, 0x00 },
  // Request DTCs
  { 5,  0x68, 0x6a, 0xf1, 0x3, 0x00, 0x00 }
};

void loop() {

  long now = millis();

  // Waiting for the 0x55
  if (state == 0) {
    if ((now - lastActivityStamp) > 2000) {
      Serial.println("ERROR 0: Timed out waiting for 0x55");
      state = 99;  
    }
  }
  
  // Waiting to send initialization ACK
  else if (state == 3) {
      // Wait W4
      if (now - lastActivityStamp < W4) {
        // PAUSE AFTER RECEIVING THE KW1
      } 
      else {
        // Send ACK
        Serial1.write(~kw1);        
        ignoreCount++;
        state = 5;
        lastActivityStamp = now;
      }
  }

  // Waiting to send a command to the vehicle
  else if (state == 6) {

    // We only generate a command during quiet periods
    if ((now - lastActivityStamp) < 2000) {
      // PAUSE
    }
    else {
      // Intro messages
      if (cycle < introRequestCount) {
        // The intro messages have variable lengths, so use the first byte 
        // to determine how longh the message is
        int len = introRequests[cycle][0];
        // WATCH OUT!  The first byte doesn't get sent
        introRequests[cycle][len] = iso_checksum(introRequests[cycle] + 1, len - 1);
        Serial1.write(introRequests[cycle] + 1, len);
        ignoreCount = len;
        cycle++;
        lastActivityStamp = now;
      }
    }
  }

  // Check to see if we have any data from the K-Line
  if (Serial1.available() > 0) {

    // Read a byte from the K-Line
    int r = Serial1.read();
    
    // The K-Line has transmit and receive data so check to see 
    // if we should ignore our own transmission.
    if (ignoreCount > 0) {
      ignoreCount--;
    } 
    else {

      /*
      // TEMP
      {
        char buf[16];
        sprintf(buf,"%x",(int)r);
        Serial.println(buf);
      }
      */
      
      // Waiting for 0x55 after initialization
      if (state == 0) {
        if (r == 0x55) {
          state = 1;        
        }
        else {
          Serial.println("ERROR 1: Bad 0x55 ACK");
          state = 99;
        }
      }
      // Waiting for KW0
      else if (state == 1) {
        kw0 = (byte)r;
        state = 2;    
      }
      // Waiting for KW1
      else if (state == 2) {
        kw1 = (byte)r;
        state = 3;     
      }
      // Waiting for ack
      else if (state == 5) {
        // This is the inverse of 0x33
        if (r == 0xcc) {
            Serial.println("INFO: ECU connection was successful");
            state = 6;
        } else {
            Serial.println("ERROR 5: Bad 0xcc ACK");
            state = 99;
        }
      }
  
      // Generic data accumulation state.  When a full message 
      // is received we process and reset the accumulator.
      else if (state == 6) {

        if (diag) {
          char buf[16];
          sprintf(buf,"%x ",(int)r);
          Serial.print(buf);
        }

        // If we've had a significant pause since the 
        // laste byte then reset the accumulation counter
        // and assume this was a partial message fragment.
        if (rxMsgLen > 0 && (now - lastActivityStamp) > RX_TIMEOUT_MS) {
          rxMsgLen = 0;
          Serial.println("WARN: Discarded partial message");
        }

        // Accumulate and wrap if necessary
        rxMsg[rxMsgLen++] = (byte)r;
        rxMsgLen = rxMsgLen % MAX_RX_MSG_LEN;

        // Look for a complete message
        if (rxMsg[0] == 0x48 && rxMsg[1] == 0x6b) {
          // Supported PIDs
          if (rxMsgLen == 10 && rxMsg[4] == 0x00) {
            char buf[128];
            format_01_00(rxMsg, rxMsgLen, buf, 0x01);
            Serial.print("Supported PIDs [01-20]: ");
            Serial.println(buf);
            // Reset the accumulator
            rxMsgLen = 0;
          }
          else if (rxMsgLen == 10 && rxMsg[4] == 0x20) {
            char buf[128];
            format_01_00(rxMsg, rxMsgLen, buf, 0x21);
            Serial.print("Supported PIDs [21-40]: ");
            Serial.println(buf);
            // Reset the accumulator
            rxMsgLen = 0;
          }
          else if (rxMsgLen == 10 && rxMsg[4] == 0x40) {
            char buf[128];
            format_01_00(rxMsg, rxMsgLen, buf, 0x41);
            Serial.print("Supported PIDs [41-60]: ");
            Serial.println(buf);
            // Reset the accumulator
            rxMsgLen = 0;
          }
          else if (rxMsgLen == 10 && rxMsg[4] == 0x60) {
            char buf[128];
            format_01_00(rxMsg, rxMsgLen, buf, 0x61);
            Serial.print("Supported PIDs [61-80]: ");
            Serial.println(buf);
            // Reset the accumulator
            rxMsgLen = 0;
          }
          else if (rxMsgLen == 10 && rxMsg[4] == 0x80) {
            char buf[128];
            format_01_00(rxMsg, rxMsgLen, buf, 0x81);
            Serial.print("Supported PIDs [81-A0]: ");
            Serial.println(buf);
            // Reset the accumulator
            rxMsgLen = 0;
          }
          // Monitor status
          else if (rxMsgLen == 10 && rxMsg[4] == 0x01) {
            char buf[64];
            format_01_01(rxMsg, rxMsgLen, buf);
            Serial.print("Monitor Status: ");
            Serial.println(buf);
            // Reset the accumulator
            rxMsgLen = 0;
          }
          // Coolant temp
          else if (rxMsgLen == 7 && rxMsg[4] == 0x05) {
            char buf[64];
            format_01_05(rxMsg, rxMsgLen, buf);
            Serial.print("Coolant Temperature: ");
            Serial.println(buf);
            // Reset the accumulator
            rxMsgLen = 0;
          }
          // RPM Message
          else if (rxMsgLen == 8 && rxMsg[4] == 0x0c) {
            char buf[64];
            format_01_0c(rxMsg, rxMsgLen, buf);
            Serial.print("Tach: ");
            Serial.println(buf);
            // Reset the accumulator
            rxMsgLen = 0;
          }
          //  Vehicle speed
          else if (rxMsgLen == 7 && rxMsg[4] == 0x0d) {
            char buf[64];
            format_01_0d(rxMsg, rxMsgLen, buf);
            Serial.print("Speed: ");
            Serial.println(buf);
            // Reset the accumulator
            rxMsgLen = 0;
          }
          //  Timing advance
          else if (rxMsgLen == 7 && rxMsg[4] == 0x0e) {
            char buf[64];
            format_01_0e(rxMsg, rxMsgLen, buf);
            Serial.print("Timing Advance: ");
            Serial.println(buf);
            // Reset the accumulator
            rxMsgLen = 0;
          }
          //  Throttle
          else if (rxMsgLen == 7 && rxMsg[4] == 0x11) {
            char buf[64];
            format_01_11(rxMsg, rxMsgLen, buf);
            Serial.print("Throttle: ");
            Serial.println(buf);
            // Reset the accumulator
            rxMsgLen = 0;
          }
          //  PID LIST
          else if (rxMsgLen == 10 && rxMsg[4] == 0x00) {
            Serial.println("DATA: LIST");
            // Reset the accumulator
            rxMsgLen = 0;
          }
        }
      }
    }
    
    // Keep track of the last time we got activity on the 
    // K-Line so that we can detect quiet times.
    lastActivityStamp = now;
  }

  // Check for data from the serial port
  if (Serial.available()) {
    int r = Serial.read();
    if (r == 'r') {
      unconfigure();
      configure();
    } else if (r == 'a') {
      Serial1.write('a');
    } else if (r == 'd') {
      // Toggle diag
      diag = true;
    } else if (r == 's') {
      Serial.println("INFO: Generating delay");
      delay(10000);
    }
  }
}
