// UNO_modbus_chaining.ino
//
// NonBlockingModbusMaster on UNO using software serial
//AltSoftSerial on Arduino Uno       TX pin 9        RX pin 8        PWM used pin 10

/**
  (c)2025 Forward Computing and Control Pty. Ltd.
  NSW Australia, www.forward.com.au
  This code is not warranted to be fit for any purpose. You may only use it at your own risk.
  This code may be freely used for both private and commercial use
  Provide this copyright is maintained.
  Also see LICENSE.txt for original ModbusMaster license.
*/

#include "AltSoftSerial.h"
#include "NonBlockingModbusMaster.h"
#include "loopTimer.h"
#include "millisDelay.h"

AltSoftSerial altSerial;
const long baudrate = 9600;
NonBlockingModbusMaster nbModbusMaster;
millisDelay samplingDelay;
unsigned long sampleInterval_ms = 3000; // timeout is 2000

bool readAddr_0();
bool readAddr_2();

const int MAX_RETRIES = 1; // retry once if timeout

unsigned long start_ms = 0;

void setup() {
  Serial.begin(115200);
  for (int i = 10; i > 0; i--) {
    Serial.print(i); Serial.print(' ');
    delay(500);
  }
  Serial.println();

  Serial.println("NonBlockingModbusMaster on UNO using AltSoftSerial ");
  Serial.println("  reading Slave ID: 1, Holding Register Address:0, Qty 2 ");
  Serial.println("  and then readHoldingRegister Address:2, Qty 2 and then finish." );
  altSerial.begin(baudrate);

  // MODBUS over serial line specification and implementation guide V1.02
  // Paragraph 2.5.1.1 MODBUS Message RTU Framing
  // https://modbus.org/docs/Modbus_over_serial_line_V1_02.pdf
  float bitduration = 1.f / baudrate;
  float charlen = 10.0f; // 8 bits + 1 stop, parity ?
  float preDelayBR = bitduration * charlen * 3.5f * 1e6  + 1; // in us
  float postDelayBR = bitduration * charlen * 3.5f * 1e6 + 1; // in us
  // ~3.28ms (4ms) for 9600 baud, or 0.84ms (1ms) for 34800 baud

  nbModbusMaster.initialize(altSerial, preDelayBR, postDelayBR); // default timeout 2000ms (2sec)
  //  nbModbusMaster.initialize(altSerial, preDelayBR, postDelayBR, 1000); // for 1000ms (1sec) timeout
  samplingDelay.start(sampleInterval_ms);
}

void processAddr_0(NonBlockingModbusMaster &mb) {
  static int retryCount = 0;
  Serial.println(" in processAddr_0()");
  // check for errors
  int err = mb.getError(); // 0 for OK
  if (err) {
    Serial.print("Error: "); mb.printHex(err, Serial); Serial.println();
    if ((err == nbModbusMaster.ku8MBResponseTimedOut) && (retryCount < MAX_RETRIES)) {
      retryCount++;
      Serial.println(" Retry");
      mb.retry(); // send same cmd again
    }
  } else {
    retryCount = 0; // success
  }
  if (!err) {
    Serial.print("response Len "); Serial.print(mb.getResponseBufferLength()); Serial.print("  ");
    for (int i = 0; i < mb.getResponseBufferLength(); i++) {
      mb.printHex(mb.getResponseBuffer(i), Serial); Serial.print(" ");
    }
    Serial.println();
  }
  // else err continue or not?? can stop here by not calling readAddr_2()

  readAddr_2(); // chain to read from addr 2 qty 2, this call will not fail as last cmd has finished
}

void processAddr_2(NonBlockingModbusMaster &mb) {
  static int retryCount = 0;
  Serial.println(" in processAddr_2()");
  // check for errors
  int err = mb.getError(); // 0 for OK
  if (err) {
    Serial.print("Error: "); mb.printHex(err, Serial); Serial.println();
    if ((err == nbModbusMaster.ku8MBResponseTimedOut) && (retryCount < MAX_RETRIES)) {
      retryCount++;
      Serial.println(" Retry");
      mb.retry(); // send same cmd again
    }
  } else {
    retryCount = 0; // success
  }
  if (!err) {
    Serial.print("response Len "); Serial.print(mb.getResponseBufferLength()); Serial.print("  ");
    for (int i = 0; i < mb.getResponseBufferLength(); i++) {
      mb.printHex(mb.getResponseBuffer(i), Serial); Serial.print(" ");
    }
    Serial.println();
  }
  // finish here
}

bool readAddr_0() {
  return nbModbusMaster.readHoldingRegisters(1, 0, 2, processAddr_0); // add processing fn
}

bool readAddr_2() {
  return nbModbusMaster.readHoldingRegisters(1, 2, 2, processAddr_2); // add processing fn
}

void loop() {
  // loopTimer.check(Serial);

  if (samplingDelay.justFinished()) {
    samplingDelay.restart();
    Serial.println(" ++ start block reads ");
    readAddr_0();
    // this will return false and be ignored if nbModbusMaster is still processing the last cmd
    // i.e. if nbModbusMaster.isProcessing() returns true
  }

  if (nbModbusMaster.justFinished()) {
    // all reads finished
    Serial.println(" -- Finished all reads.");
  }

}
