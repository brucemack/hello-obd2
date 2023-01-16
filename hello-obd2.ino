const byte rxPin = 0;
const byte txPin = 1;
const byte LED_PIN = 13;

int zeroState = LOW;
int oneState = HIGH;

void generateInit() {

  // Start bin
  digitalWrite(txPin, zeroState);
  delay(200);
  // Send 00110011
  digitalWrite(txPin, oneState);
  delay(200);
  delay(200);
  digitalWrite(txPin, zeroState);
  delay(200);
  delay(200);
  digitalWrite(txPin, oneState);
  delay(200);
  delay(200);
  digitalWrite(txPin, zeroState);
  delay(200);
  delay(200);
  // Stop bit (idle state)
  digitalWrite(txPin, oneState);
  delay(200);
}

// inspired by SternOBDII\code\checksum.c
static byte iso_checksum(byte *data, byte len)
{
  byte crc=0;
  for(byte i=0; i<len; i++)
    crc=crc+data[i];
  return crc;
}

const long W1 = 20;
const long W4 = 30;

int state = 0;
long stamp0 = 0;
long stamp3 = 0;
byte kw0 = 0;
byte kw1 = 0;

void setup() {

    Serial.begin(9600);
    // Define pin modes for TX and RX
    pinMode(rxPin, INPUT);
    pinMode(txPin, OUTPUT);
    // Idle state for TX
    digitalWrite(txPin, HIGH);
    pinMode(LED_PIN, OUTPUT);
    // Idle state for the LED pin
    digitalWrite(LED_PIN, LOW);

    // Stobe LED
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(500);
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(500);

    Serial.println("OBD2 Diagnostic Scanner");
   
    delay(10000);

    // Send the 5-baud init
    Serial.println("Connecting to ECU ...");

    generateInit();
    
    // Set the baud rate for the serial port
    Serial1.begin(10400);
        
    state = 0;
    stamp0 = millis();    
}

int diagState = 0;
int ignoreCount = 0;

byte rxMsg[16];
int rxMsgLen = 0;

void loop() {

  long now = millis();

  // Waiting to send initialization ACK
  if (state == 3) {
      // Wait W4
      if (now - stamp3 < W4) {
      } 
      else {
        state = 5;
        // Send ACK
        Serial1.write(~kw1);        
        ignoreCount++;
      }
  }

  // Waiting to send command
  else if (state == 6) {
    delay(500);
    // RPM query
    //byte msg[6] = { 0x68, 0x6a, 0xf1, 0x1, 0x0c, 0x0 };
    //msg[5] = iso_checksum(msg, 5);
    // 00
    byte msg[6] = { 0x68, 0x6a, 0xf1, 0x1, 0x00, 0xc4 };
    ignoreCount = 6;
    Serial1.write(msg, 6);
    state = 7;
    Serial.println("Sent query");
  }

  if (Serial1.available() > 0) {

    // Read a byte from the vehicle
    int r = Serial1.read();

    // TEMP
    {
      char buf[16];
      sprintf(buf,"%x",(int)r);
      Serial.println(buf);
    }
    
    // The K-Line has transmit and receive data so check to see 
    // if we should ignore our own transmission
    if (ignoreCount > 0) {
      ignoreCount--;
      Serial.println("(Ignored)");
    } 
    else {
      
      // Waiting for 0x55 after initialization
      if (state == 0) {
        // Throw away garbage during W1
        if (now - stamp0 < W1) {
          Serial.println("(Garbage ignored)");
        } 
        else {
          if (r == 0x55) {
            state = 1;        
          }
          else {
            state = 99;
            Serial.println("ERROR 0: Bad 0x55 ACK");
            char buf[16];
            sprintf(buf,"%x",(int)r);
            Serial.println(buf);
          }
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
        stamp3 = millis();
      }
      // Waiting for ack
      else if (state == 5) {
        // This is the inverse of 0x33
        if (r == 0xcc) {
            Serial.println("ECU connection was successful!");
            state = 6;
        } else {
            Serial.println("ERROR 5: Bad 0xcc ACK");
            state = 99;
        }
      }
  
      // Generic display
      else if (state == 7) {
        Serial.println("Accumlated");
        /*
        // DEBUG DISPLAY
        char buf[16];
        sprintf(buf,"%x ",(int)r);
        Serial.print(buf);
        */
        /*
        // Accumulate
        rxMsg[rxMsgLen++] = (byte)r;

        // Look for a complete message
        if (rxMsgLen == 8) {
          unsigned int rpm = ((rxMsg[5] * 256) + rxMsg[6]) / 4;
          rxMsgLen = 0;
          char buf[16];
          sprintf(buf,"RPM %d",(int)rpm);
          Serial.println();
          Serial.println(buf);
          // Switch batch for a new query
          state = 6;
        }
        */
      }
    }
  }
}
