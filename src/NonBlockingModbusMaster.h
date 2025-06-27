#ifndef NonBlockingModbusMaster_h
#define NonBlockingModbusMaster_h

// NonBlockingModbusMaster.h

/**
modifications:
(c)2025 Forward Computing and Control Pty. Ltd.  
NSW Australia, www.forward.com.au  
This code is not warranted to be fit for any purpose. You may only use it at your own risk.  
This code may be freely used for both private and commercial use  
Provide this copyright is maintained.  
Also see LICENSE.txt for original ModbusMaster license.  
*/

/**
  @file
  Arduino library for communicating with Modbus slaves over RS232/485 (via RTU protocol).

  @defgroup setup NonBlockingModbusMaster Object Instantiation/Initialization
  @defgroup buffer NonBlockingModbusMaster Buffer Management
  @defgroup discrete Modbus Function Codes for Discrete Coils/Inputs
  @defgroup register Modbus Function Codes for Holding/Input Registers
  @defgroup constant Modbus Function Codes, Exception Codes
*/
/*

  NonBlockingModbusMaster.h - A Non Blocking Arduino library for communicating with Modbus slaves
  over RS232/485 (via RTU protocol).

  Library:: NonBlockingModbusMaster

  Copyright:: 2009-2016 Doc Walker
  Modifications Copyright:: 2025 Matthew Ford

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

*/




/* _____STANDARD INCLUDES____________________________________________________ */
// include types & constants of Wiring core API
#include "Arduino.h"

/* _____UTILITY MACROS_______________________________________________________ */
// functions to calculate Modbus Application Data Unit CRC
#include "util/crc16.h"

// functions to manipulate words
#include "util/word.h"


// Note for JSY modules the 32bit registers have the high order 16bits in the first
// lower numbered register and the low 16bits in the second register.
// so call this with
//   Import_100xkWh = make_32bit_from_2x16bit_words(nbModbusMaster.getResponseBuffer(0), nbModbusMaster.getResponseBuffer(1));
uint32_t make_32bit_from_2x16bit_words(uint16_t hi_word, uint16_t lo_word);
// for other conversion methods to and from float and 64bit data see  the ArudinoModbus library, ArduinoModbus\src\libmodbus\modbus-data.c

class NonBlockingModbusMaster; // forward reference
// type def for result handler functions which are passed a reference to the modbus class that call them
typedef void (*ResultHandler)(NonBlockingModbusMaster &mb);

/* _____CLASS DEFINITIONS____________________________________________________ */
/**
  A Non Blocking Arduino class library for communicating with Modbus slaves over
  RS232/485 (via RTU protocol).
*/
class NonBlockingModbusMaster {
  public:
    NonBlockingModbusMaster();
    void initialize(Stream &serial, uint16_t preDelay_us, uint16_t postDelay_us, uint16_t timeout_ms = 2000);
    bool justFinished(); // returns true when request complete or on error. check getError();
    bool isIdle(); // return true if IDLE
    bool isProcessing(); // return true when not IDLE
    uint8_t getError(); // 0 is OK else error
    bool retry(); // returns true if retry started, else false, some other command still running, reruns the last cmd with same args, used for timeout reties
    uint16_t getResponseBuffer(uint8_t idx); // get 16bit word at this index in response buffer
    void     clearResponseBuffer();
    uint16_t getResponseBufferLength(); // number of values in response buffer is 0 on error
    uint8_t getSlaveId();
    uint16_t getAddress();
    uint16_t getQty();
    uint8_t getFunction();
    void oneTimeDelay(uint16_t delay_ms); // adds this delay before next cmd is sent, but only the next one
    void printHex(uint8_t i, Print& out); //prints 8bit int as hex prepending 0x and padding with leading zero if necessary
    void printHex(uint8_t i, Print* outPtr);
    void preTransmission(void (*)());
    void postTransmission(void (*)());

    // Modbus exception codes
    /**
      Modbus protocol illegal function exception.

      The function code received in the query is not an allowable action for
      the server (or slave). This may be because the function code is only
      applicable to newer devices, and was not implemented in the unit
      selected. It could also indicate that the server (or slave) is in the
      wrong state to process a request of this type, for example because it is
      unconfigured and is being asked to return register values.

      @ingroup constant
    */
    static const uint8_t ku8MBIllegalFunction            = 0x01;

    /**
      Modbus protocol illegal data address exception.

      The data address received in the query is not an allowable address for
      the server (or slave). More specifically, the combination of reference
      number and transfer length is invalid. For a controller with 100
      registers, the ADU addresses the first register as 0, and the last one
      as 99. If a request is submitted with a starting register address of 96
      and a quantity of registers of 4, then this request will successfully
      operate (address-wise at least) on registers 96, 97, 98, 99. If a
      request is submitted with a starting register address of 96 and a
      quantity of registers of 5, then this request will fail with Exception
      Code 0x02 "Illegal Data Address" since it attempts to operate on
      registers 96, 97, 98, 99 and 100, and there is no register with address
      100.

      @ingroup constant
    */
    static const uint8_t ku8MBIllegalDataAddress         = 0x02;

    /**
      Modbus protocol illegal data value exception.

      A value contained in the query data field is not an allowable value for
      server (or slave). This indicates a fault in the structure of the
      remainder of a complex request, such as that the implied length is
      incorrect. It specifically does NOT mean that a data item submitted for
      storage in a register has a value outside the expectation of the
      application program, since the MODBUS protocol is unaware of the
      significance of any particular value of any particular register.

      @ingroup constant
    */
    static const uint8_t ku8MBIllegalDataValue           = 0x03;

    /**
      Modbus protocol slave device failure exception.

      An unrecoverable error occurred while the server (or slave) was
      attempting to perform the requested action.

      @ingroup constant
    */
    static const uint8_t ku8MBSlaveDeviceFailure         = 0x04;

    // Class-defined success/exception codes
    /**
      NonBlockingModbusMaster success.

      Modbus transaction was successful; the following checks were valid:
      - slave ID
      - function code
      - response code
      - data
      - CRC

      @ingroup constant
    */
    static const uint8_t ku8MBSuccess                    = 0x00;

    /**
      NonBlockingModbusMaster invalid response slave ID exception.

      The slave ID in the response does not match that of the request.

      @ingroup constant
    */
    static const uint8_t ku8MBInvalidSlaveID             = 0xE0;

    /**
      NonBlockingModbusMaster invalid response function exception.

      The function code in the response does not match that of the request.

      @ingroup constant
    */
    static const uint8_t ku8MBInvalidFunction            = 0xE1;

    /**
      NonBlockingModbusMaster response timed out exception.

      The entire response was not received within the timeout period,
      NonBlockingModbusMaster::ku8MBResponseTimeout.

      @ingroup constant
    */
    static const uint8_t ku8MBResponseTimedOut           = 0xE2;

    /**
      NonBlockingModbusMaster invalid response CRC exception.

      The CRC in the response does not match the one calculated.

      @ingroup constant
    */
    static const uint8_t ku8MBInvalidCRC                 = 0xE3;

    /**
      NonBlockingModbusMaster data larger than tx buffer (128)
    */
    static const uint8_t ku8MBDataToLong                 = 0xE4;

    /**
      NonBlockingModbusMaster response truncated
    */
    static const uint8_t ku8MBResponseToShort            = 0xE5;

    /**
      NonBlockingModbusMaster error is state machine (should not happen)
    */
    static const uint8_t ku8MBStateError               = 0xE6;


    bool readCoils(uint8_t slaveId, uint16_t address, uint16_t qty, ResultHandler handlerFn = NULL);
    bool readDiscreteInputs(uint8_t slaveId, uint16_t address, uint16_t qty, ResultHandler handlerFn = NULL);
    bool readHoldingRegisters(uint8_t slaveId, uint16_t address, uint16_t qty, ResultHandler handlerFn = NULL); // defaults to none
    bool readInputRegisters(uint8_t slaveId, uint16_t address, uint8_t qty, ResultHandler handlerFn = NULL);
    bool writeSingleCoil(uint8_t slaveId, uint16_t address, uint8_t state, ResultHandler handlerFn = NULL);
    bool writeSingleRegister(uint8_t slaveId, uint16_t address, uint16_t qty, ResultHandler handlerFn = NULL);
    bool writeMultipleCoils(uint8_t slaveId, uint16_t address, uint16_t qty, ResultHandler handlerFn = NULL);
    bool writeMultipleCoils(uint8_t slaveId, ResultHandler handlerFn = NULL);
    bool writeMultipleRegisters(uint8_t slaveId, uint16_t address, uint16_t qty, ResultHandler handlerFn = NULL);
    bool writeMultipleRegisters(uint8_t slaveId, ResultHandler handlerFn = NULL);
    bool maskWriteRegister(uint8_t slaveId, uint16_t, uint16_t, uint16_t, ResultHandler handlerFn = NULL);
    bool readWriteMultipleRegisters(uint8_t slaveId, uint16_t readAddress, uint16_t readQty, uint16_t writeAddress, uint16_t writeQty, ResultHandler handlerFn = NULL);
    bool readWriteMultipleRegisters(uint8_t slaveId, uint16_t readAddress, uint16_t readQty, ResultHandler handlerFn = NULL);


    uint8_t  setTransmitBuffer(uint8_t, uint16_t);
    void     clearTransmitBuffer();


    // these methods do not actually send anything, just sets data in _u16TransmitBuffer
    void sendBit(bool);
    void send(uint8_t);
    void send(uint16_t);
    void send(uint32_t);

    // read data in uint16_t words from response buffer
    uint8_t available(void); // number of words available to read
    uint16_t receive(void);  // read next word and decrement number available


  protected:
    bool init(uint8_t slaveId, ResultHandler fn = NULL); // returns false if isProcessing() true
    typedef enum MODBUS_STATE_ENUM { MB_START,  MB_PRE_DELAY, MB_SEND, MB_POST_DELAY, MB_RECEIVE, MB_END, MB_IDLE, MB_ERROR} modbusStateEnum;
    modbusStateEnum modbusState;
    uint8_t u8MBStatus;
    uint8_t u8BytesLeft;
    uint32_t u32StartTime;
    static  const uint8_t maxTXLen = 128;
    // use ESP32 setTxBufferSize( ) before begin,  default is 128 if this needs to be larger
    uint8_t u8ModbusADU[maxTXLen];
    uint8_t u8ModbusADUSize;// = 0;
    uint8_t u8MBFunction;
    uint16_t _oneTimeDelay_ms;



  private:
    Stream* _serial;                                             ///< reference to serial port object
    uint8_t  _u8MBSlave;                                         ///< Modbus slave (1..255) initialized in begin()
    static const uint8_t ku8MaxBufferSize                = 64;   ///< size of response/transmit buffers
    uint16_t _u16ReadAddress;                                    ///< slave register from which to read
    uint16_t _u16ReadQty;                                        ///< quantity of words to read
    uint16_t _u16ResponseBuffer[ku8MaxBufferSize];               ///< buffer to store Modbus slave response; read via GetResponseBuffer()
    uint16_t _u16WriteAddress;                                   ///< slave register to which to write
    uint16_t _u16WriteQty;                                       ///< quantity of words to write
    uint16_t _u16TransmitBuffer[ku8MaxBufferSize];               ///< buffer containing data to transmit to Modbus slave; set via SetTransmitBuffer()
    uint8_t _u8TransmitBufferIndex;
    uint16_t u16TransmitBufferLength;
    uint16_t* rxBuffer; // from Wire.h -- need to clean this up Rx
    uint8_t _u8ResponseBufferIndex;
    uint8_t _u8ResponseBufferLength;

    // Modbus function codes for bit access
    static const uint8_t ku8MBReadCoils                  = 0x01; ///< Modbus function 0x01 Read Coils
    static const uint8_t ku8MBReadDiscreteInputs         = 0x02; ///< Modbus function 0x02 Read Discrete Inputs
    static const uint8_t ku8MBWriteSingleCoil            = 0x05; ///< Modbus function 0x05 Write Single Coil
    static const uint8_t ku8MBWriteMultipleCoils         = 0x0F; ///< Modbus function 0x0F Write Multiple Coils

    // Modbus function codes for 16 bit access
    static const uint8_t ku8MBReadHoldingRegisters       = 0x03; ///< Modbus function 0x03 Read Holding Registers
    static const uint8_t ku8MBReadInputRegisters         = 0x04; ///< Modbus function 0x04 Read Input Registers
    static const uint8_t ku8MBWriteSingleRegister        = 0x06; ///< Modbus function 0x06 Write Single Register
    static const uint8_t ku8MBWriteMultipleRegisters     = 0x10; ///< Modbus function 0x10 Write Multiple Registers
    static const uint8_t ku8MBMaskWriteRegister          = 0x16; ///< Modbus function 0x16 Mask Write Register
    static const uint8_t ku8MBReadWriteMultipleRegisters = 0x17; ///< Modbus function 0x17 Read Write Multiple Registers

    // Modbus timeout [milliseconds]
    uint16_t ku16MBResponseTimeout;//        = 1000; ///< Modbus timeout [milliseconds]
    uint16_t _preDelay_us;
    uint16_t _postDelay_us;

    // master function that conducts Modbus transactions
    void NonBlockingModbusMasterTransaction(uint8_t u8MBFunction);

    // callback to handle results
    ResultHandler resultHandler;  //void (*resultHandler)(NonBlockingModbusMaster &mb);

    // preTransmission callback function; gets called before writing a Modbus message
    void (*_preTransmission)();
    // postTransmission callback function; gets called after a Modbus message has been sent
    void (*_postTransmission)();
};
#endif
