/**
  @file
  Arduino library for communicating with Modbus slaves over RS232/485 (via RTU protocol).
*/
/**
modifications:
(c)2025 Forward Computing and Control Pty. Ltd.  
NSW Australia, www.forward.com.au  
This code is not warranted to be fit for any purpose. You may only use it at your own risk.  
This code may be freely used for both private and commercial use  
Provide this copyright is maintained.  
Also see LICENSE.txt for original ModbusMaster license.  
*/

/*

  NonBlockingModbusMaster.cpp - A Non Blocking Arduino library for communicating with Modbus slaves
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

/* _____PROJECT INCLUDES_____________________________________________________ */
#include "NonBlockingModbusMaster.h"

//#define DEBUG_SERIAL

/* _____GLOBAL VARIABLES_____________________________________________________ */
#ifdef DEBUG_SERIAL
static Print* modbusDebugPtr = &Serial;
#else
static Print* modbusDebugPtr = NULL;
#endif

// 2 x 16bit to 32bit
static union BITS16_32 {
  uint32_t word_32bit;
  uint16_t words_16bit[2];
} bits16_32;

uint32_t make_32bit_from_2x16bit_words(uint16_t hi_word, uint16_t lo_word) {
  bits16_32.words_16bit[0] = lo_word;
  bits16_32.words_16bit[1] = hi_word;
  return bits16_32.word_32bit;
}

void NonBlockingModbusMaster::printHex(uint8_t i, Print& out) {
  out.print("0x");
  if (i < 16) {
    out.print("0");
  }
  out.print(i, HEX);
  out.print(" ");
}
void NonBlockingModbusMaster::printHex(uint8_t i, Print* outPtr) {
  if (!outPtr) {
    return;
  }
  printHex(i,*outPtr);
}

/* _____PUBLIC FUNCTIONS_____________________________________________________ */
/**
  Constructor.

  Creates class object; initialize it using NonBlockingModbusMaster::begin().

  @ingroup setup
*/
NonBlockingModbusMaster::NonBlockingModbusMaster(void) {
  _serial = NULL;
  _u8MBSlave = 0;
  _u16ReadAddress = 0;
  _u16ReadQty =  0;
  _u16WriteAddress = 0;
  _u16WriteQty = 0;
  _u8TransmitBufferIndex = 0;
  u16TransmitBufferLength = 0;
  _u8ResponseBufferIndex = 0;
  _u8ResponseBufferLength = 0;
  resultHandler = NULL;
  ku16MBResponseTimeout = 2000; // default value
  modbusState = MB_IDLE;
  _oneTimeDelay_ms = 0;
  _preTransmission = 0;
  _postTransmission = 0;
}

/**
  Initialize class object.

  Call once class has been instantiated, typically within setup().

  @param slave Modbus slave ID (1..255)
  @param &serial reference to serial port object (Serial, Serial1, ... Serial3)
  @ingroup setup
*/
// timeout_ms defaults to 2000 ms if not specified
void NonBlockingModbusMaster::initialize(Stream &serial, uint16_t preDelay_us, uint16_t postDelay_us, uint16_t timeout_ms) {
  _serial = &serial;
  _preDelay_us = preDelay_us;
  _postDelay_us = postDelay_us;
  ku16MBResponseTimeout = timeout_ms;
}

// adds this delay before next cmd is sent, but only the next one
void NonBlockingModbusMaster::oneTimeDelay(uint16_t delay_ms) {
  _oneTimeDelay_ms = delay_ms;
}

// sets slaveId and inits buffers
// sets slaveId and sets state to MB_START if not processing
bool NonBlockingModbusMaster::init(uint8_t slaveId, ResultHandler fn) {
  if (isProcessing()) {
    return false; // leave error and results unchanged
  }
  modbusState = MB_START;
  u8MBStatus = ku8MBSuccess;
  _u8MBSlave = slaveId;
  resultHandler = fn;
  return true;
}

uint8_t NonBlockingModbusMaster::getError() {
  return u8MBStatus;
}


bool NonBlockingModbusMaster::isIdle() {
  // return true when IDLE
  return (modbusState == MB_IDLE);
}

bool NonBlockingModbusMaster::isProcessing() {
  // return true when not IDLE
  return (modbusState != MB_IDLE);
}

uint8_t NonBlockingModbusMaster::getSlaveId() {
  return _u8MBSlave;
}
uint16_t NonBlockingModbusMaster::getAddress() {
  return _u16ReadAddress;
}
uint16_t NonBlockingModbusMaster::getQty() {
  return _u16ReadQty;
}
uint8_t NonBlockingModbusMaster::getFunction() {
  return u8MBFunction;
}

void NonBlockingModbusMaster::sendBit(bool data) {
  uint8_t txBitIndex = u16TransmitBufferLength % 16;
  if ((u16TransmitBufferLength >> 4) < ku8MaxBufferSize)
  {
    if (0 == txBitIndex)
    {
      _u16TransmitBuffer[_u8TransmitBufferIndex] = 0;
    }
    bitWrite(_u16TransmitBuffer[_u8TransmitBufferIndex], txBitIndex, data);
    u16TransmitBufferLength++;
    _u8TransmitBufferIndex = u16TransmitBufferLength >> 4;
  }
}


void NonBlockingModbusMaster::send(uint16_t data) {
  if (_u8TransmitBufferIndex < ku8MaxBufferSize)
  {
    _u16TransmitBuffer[_u8TransmitBufferIndex++] = data;
    u16TransmitBufferLength = _u8TransmitBufferIndex << 4;
  }
}


void NonBlockingModbusMaster::send(uint32_t data) {
  send(lowWord(data));
  send(highWord(data));
}


void NonBlockingModbusMaster::send(uint8_t data) {
  send(word(data));
}

// ==============================================================================
// these methods let your collect the 16bit response data one 16bit word at a time

// the number of words available
uint8_t NonBlockingModbusMaster::available(void) {
  return _u8ResponseBufferLength - _u8ResponseBufferIndex;
}

// get next work,  returns -1 (0xFFFF) if no more
uint16_t NonBlockingModbusMaster::receive(void) {
  if (_u8ResponseBufferIndex < _u8ResponseBufferLength)   {
    return _u16ResponseBuffer[_u8ResponseBufferIndex++];
  }  else   {
    return 0xFFFF;
  }
}

// number of values in response buffer
uint16_t NonBlockingModbusMaster::getResponseBufferLength() {
  return _u8ResponseBufferLength;
}

/**
Set pre-transmission callback function.

This function gets called just before a Modbus message is sent over serial.
Typical usage of this callback is to enable an RS485 transceiver's
Driver Enable pin, and optionally disable its Receiver Enable pin.

*/
void NonBlockingModbusMaster::preTransmission(void (*preTransmission)())
{
  _preTransmission = preTransmission;
}

/**
Set post-transmission callback function.

This function gets called after a Modbus message has finished sending
(i.e. after all data has been physically transmitted onto the serial
bus).

Typical usage of this callback is to enable an RS485 transceiver's
Receiver Enable pin, and disable its Driver Enable pin.

*/
void NonBlockingModbusMaster::postTransmission(void (*postTransmission)())
{
  _postTransmission = postTransmission;
}

/**
  Retrieve data from response buffer via index
  returns -1 (0xFFFF) if accessed past getResponseBufferLength()

  @see NonBlockingModbusMaster::clearResponseBuffer()
  @param u8Index index of response buffer array (0x00..0x3F)
  @return value in position u8Index of response buffer (0x0000..0xFFFF)
  @ingroup buffer
*/
uint16_t NonBlockingModbusMaster::getResponseBuffer(uint8_t u8Index) {
  if (u8Index < _u8ResponseBufferLength)  {
    return _u16ResponseBuffer[u8Index];
  } else {
    return 0xFFFF;
  }
}


/**
  Clear Modbus response buffer.

  @see NonBlockingModbusMaster::getResponseBuffer(uint8_t u8Index)
  @ingroup buffer
*/
void NonBlockingModbusMaster::clearResponseBuffer() {
  uint8_t i;
  for (i = 0; i < ku8MaxBufferSize; i++) {
    _u16ResponseBuffer[i] = 0;
  }
  _u8ResponseBufferLength = 0;
}


/**
  Place data in transmit buffer.

  @see NonBlockingModbusMaster::clearTransmitBuffer()
  @param u8Index index of transmit buffer array (0x00..0x3F)
  @param u16Value value to place in position u8Index of transmit buffer (0x0000..0xFFFF)
  @return 0 on success; exception number on failure
  @ingroup buffer
*/
uint8_t NonBlockingModbusMaster::setTransmitBuffer(uint8_t u8Index, uint16_t u16Value) {
  if (u8Index < ku8MaxBufferSize)  {
    _u16TransmitBuffer[u8Index] = u16Value;
    return ku8MBSuccess;
  }   else  {
    return ku8MBIllegalDataAddress;
  }
}


/**
  Clear Modbus transmit buffer.

  @see NonBlockingModbusMaster::setTransmitBuffer(uint8_t u8Index, uint16_t u16Value)
  @ingroup buffer
*/
void NonBlockingModbusMaster::clearTransmitBuffer() {
  uint8_t i;
  for (i = 0; i < ku8MaxBufferSize; i++)  {
    _u16TransmitBuffer[i] = 0;
  }
}

// returns true if retry started
bool NonBlockingModbusMaster::retry() {
  if (!init(_u8MBSlave, resultHandler)) {
    return false;
  }
  // else
  NonBlockingModbusMasterTransaction(u8MBFunction); // rerun is save args
  // NOTE this has only been checked for readInputRegisters
  return true;
}

/**
  Modbus function 0x01 Read Coils.

  This function code is used to read from 1 to 2000 contiguous status of
  coils in a remote device. The request specifies the starting address,
  i.e. the address of the first coil specified, and the number of coils.
  Coils are addressed starting at zero.

  The coils in the response buffer are packed as one coil per bit of the
  data field. Status is indicated as 1=ON and 0=OFF. The LSB of the first
  data word contains the output addressed in the query. The other coils
  follow toward the high order end of this word and from low order to high
  order in subsequent words.

  If the returned quantity is not a multiple of sixteen, the remaining
  bits in the final data word will be padded with zeros (toward the high
  order end of the word).

  @param u16ReadAddress address of first coil (0x0000..0xFFFF)
  @param u16BitQty quantity of coils to read (1..2000, enforced by remote device)
  @return 0 on success; exception number on failure
  @ingroup discrete
*/
bool NonBlockingModbusMaster::readCoils(uint8_t slaveId, uint16_t u16ReadAddress, uint16_t u16BitQty, ResultHandler handlerFn) {
  if (!init(slaveId, handlerFn)) {
    return false;
  }
  _u16ReadAddress = u16ReadAddress;
  _u16ReadQty = u16BitQty;
  NonBlockingModbusMasterTransaction(ku8MBReadCoils);
  return true;
}


/**
  Modbus function 0x02 Read Discrete Inputs.

  This function code is used to read from 1 to 2000 contiguous status of
  discrete inputs in a remote device. The request specifies the starting
  address, i.e. the address of the first input specified, and the number
  of inputs. Discrete inputs are addressed starting at zero.

  The discrete inputs in the response buffer are packed as one input per
  bit of the data field. Status is indicated as 1=ON; 0=OFF. The LSB of
  the first data word contains the input addressed in the query. The other
  inputs follow toward the high order end of this word, and from low order
  to high order in subsequent words.

  If the returned quantity is not a multiple of sixteen, the remaining
  bits in the final data word will be padded with zeros (toward the high
  order end of the word).

  @param u16ReadAddress address of first discrete input (0x0000..0xFFFF)
  @param u16BitQty quantity of discrete inputs to read (1..2000, enforced by remote device)
  @return 0 on success; exception number on failure
  @ingroup discrete
*/
bool NonBlockingModbusMaster::readDiscreteInputs(uint8_t slaveId, uint16_t u16ReadAddress,   uint16_t u16BitQty, ResultHandler handlerFn) {
  if (!init(slaveId, handlerFn)) {
    return false;
  }
  _u16ReadAddress = u16ReadAddress;
  _u16ReadQty = u16BitQty;
  NonBlockingModbusMasterTransaction(ku8MBReadDiscreteInputs);
  return true;
}


/**
  Modbus function 0x03 Read Holding Registers.

  This function code is used to read the contents of a contiguous block of
  holding registers in a remote device. The request specifies the starting
  register address and the number of registers. Registers are addressed
  starting at zero.

  The register data in the response buffer is packed as one word per
  register.

  @param u16ReadAddress address of the first holding register (0x0000..0xFFFF)
  @param u16ReadQty quantity of holding registers to read (1..125, enforced by remote device)
  @return 0 on success; exception number on failure
  @ingroup register
*/
bool NonBlockingModbusMaster::readHoldingRegisters(uint8_t slaveId, uint16_t u16ReadAddress, uint16_t u16ReadQty, ResultHandler handlerFn) {
  if (!init(slaveId, handlerFn)) {
    return false;
  }
  _u16ReadAddress = u16ReadAddress;
  _u16ReadQty = u16ReadQty;
  NonBlockingModbusMasterTransaction(ku8MBReadHoldingRegisters);
  return true;
}


/**
  Modbus function 0x04 Read Input Registers.

  This function code is used to read from 1 to 125 contiguous input
  registers in a remote device. The request specifies the starting
  register address and the number of registers. Registers are addressed
  starting at zero.

  The register data in the response buffer is packed as one word per
  register.

  @param u16ReadAddress address of the first input register (0x0000..0xFFFF)
  @param u16ReadQty quantity of input registers to read (1..125, enforced by remote device)
  @return 0 on success; exception number on failure
  @ingroup register
*/
bool NonBlockingModbusMaster::readInputRegisters(uint8_t slaveId, uint16_t u16ReadAddress,  uint8_t u16ReadQty, ResultHandler handlerFn)  {
  if (!init(slaveId, handlerFn)) {
    return false;
  }
  _u16ReadAddress = u16ReadAddress;
  _u16ReadQty = u16ReadQty;
  NonBlockingModbusMasterTransaction(ku8MBReadInputRegisters);
  return true;
}


/**
  Modbus function 0x05 Write Single Coil.

  This function code is used to write a single output to either ON or OFF
  in a remote device. The requested ON/OFF state is specified by a
  constant in the state field. A non-zero value requests the output to be
  ON and a value of 0 requests it to be OFF. The request specifies the
  address of the coil to be forced. Coils are addressed starting at zero.

  @param u16WriteAddress address of the coil (0x0000..0xFFFF)
  @param u8State 0=OFF, non-zero=ON (0x00..0xFF)
  @return 0 on success; exception number on failure
  @ingroup discrete
*/
bool NonBlockingModbusMaster::writeSingleCoil(uint8_t slaveId, uint16_t u16WriteAddress, uint8_t u8State, ResultHandler handlerFn) {
  if (!init(slaveId, handlerFn)) {
    return false;
  }
  _u16WriteAddress = u16WriteAddress;
  _u16WriteQty = (u8State ? 0xFF00 : 0x0000);
  NonBlockingModbusMasterTransaction(ku8MBWriteSingleCoil);
  return true;
}


/**
  Modbus function 0x06 Write Single Register.

  This function code is used to write a single holding register in a
  remote device. The request specifies the address of the register to be
  written. Registers are addressed starting at zero.

  @param u16WriteAddress address of the holding register (0x0000..0xFFFF)
  @param u16WriteValue value to be written to holding register (0x0000..0xFFFF)
  @return 0 on success; exception number on failure
  @ingroup register
*/
bool NonBlockingModbusMaster::writeSingleRegister(uint8_t slaveId, uint16_t u16WriteAddress, uint16_t u16WriteValue, ResultHandler handlerFn) {
  if (!init(slaveId, handlerFn)) {
    return false;
  }
  _u16WriteAddress = u16WriteAddress;
  _u16WriteQty = 0;
  _u16TransmitBuffer[0] = u16WriteValue;
  NonBlockingModbusMasterTransaction(ku8MBWriteSingleRegister);
  return true;
}


/**
  Modbus function 0x0F Write Multiple Coils.

  This function code is used to force each coil in a sequence of coils to
  either ON or OFF in a remote device. The request specifies the coil
  references to be forced. Coils are addressed starting at zero.

  The requested ON/OFF states are specified by contents of the transmit
  buffer. A logical '1' in a bit position of the buffer requests the
  corresponding output to be ON. A logical '0' requests it to be OFF.

  @param u16WriteAddress address of the first coil (0x0000..0xFFFF)
  @param u16BitQty quantity of coils to write (1..2000, enforced by remote device)
  @return 0 on success; exception number on failure
  @ingroup discrete
*/
bool NonBlockingModbusMaster::writeMultipleCoils(uint8_t slaveId, uint16_t u16WriteAddress, uint16_t u16BitQty, ResultHandler handlerFn) {
  if (!init(slaveId, handlerFn)) {
    return false;
  }
  _u16WriteAddress = u16WriteAddress;
  _u16WriteQty = u16BitQty;
  NonBlockingModbusMasterTransaction(ku8MBWriteMultipleCoils);
  return true;
}

bool NonBlockingModbusMaster::writeMultipleCoils(uint8_t slaveId, ResultHandler handlerFn) {
  if (!init(slaveId, handlerFn)) {
    return false;
  }
  _u16WriteQty = u16TransmitBufferLength;
  NonBlockingModbusMasterTransaction(ku8MBWriteMultipleCoils);
  return true;
}


/**
  Modbus function 0x10 Write Multiple Registers.

  This function code is used to write a block of contiguous registers (1
  to 123 registers) in a remote device.

  The requested written values are specified in the transmit buffer. Data
  is packed as one word per register.

  @param u16WriteAddress address of the holding register (0x0000..0xFFFF)
  @param u16WriteQty quantity of holding registers to write (1..123, enforced by remote device)
  @return 0 on success; exception number on failure
  @ingroup register
*/
bool NonBlockingModbusMaster::writeMultipleRegisters(uint8_t slaveId, uint16_t u16WriteAddress, uint16_t u16WriteQty, ResultHandler handlerFn) {
  if (!init(slaveId, handlerFn)) {
    return false;
  }
  _u16WriteAddress = u16WriteAddress;
  _u16WriteQty = u16WriteQty;
  NonBlockingModbusMasterTransaction(ku8MBWriteMultipleRegisters);
  return true;
}

// new version based on Wire.h
bool NonBlockingModbusMaster::writeMultipleRegisters(uint8_t slaveId, ResultHandler handlerFn) {
  if (!init(slaveId, handlerFn)) {
    return false;
  }
  _u16WriteQty = _u8TransmitBufferIndex;
  NonBlockingModbusMasterTransaction(ku8MBWriteMultipleRegisters);
  return true;
}

/**
  Modbus function 0x16 Mask Write Register.

  This function code is used to modify the contents of a specified holding
  register using a combination of an AND mask, an OR mask, and the
  register's current contents. The function can be used to set or clear
  individual bits in the register.

  The request specifies the holding register to be written, the data to be
  used as the AND mask, and the data to be used as the OR mask. Registers
  are addressed starting at zero.

  The function's algorithm is:

  Result = (Current Contents && And_Mask) || (Or_Mask && (~And_Mask))

  @param u16WriteAddress address of the holding register (0x0000..0xFFFF)
  @param u16AndMask AND mask (0x0000..0xFFFF)
  @param u16OrMask OR mask (0x0000..0xFFFF)
  @return 0 on success; exception number on failure
  @ingroup register
*/
bool NonBlockingModbusMaster::maskWriteRegister(uint8_t slaveId, uint16_t u16WriteAddress, uint16_t u16AndMask, uint16_t u16OrMask, ResultHandler handlerFn) {
  if (!init(slaveId, handlerFn)) {
    return false;
  }
  _u16WriteAddress = u16WriteAddress;
  _u16TransmitBuffer[0] = u16AndMask;
  _u16TransmitBuffer[1] = u16OrMask;
  NonBlockingModbusMasterTransaction(ku8MBMaskWriteRegister);
  return true;
}


/**
  Modbus function 0x17 Read Write Multiple Registers.

  This function code performs a combination of one read operation and one
  write operation in a single MODBUS transaction. The write operation is
  performed before the read. Holding registers are addressed starting at
  zero.

  The request specifies the starting address and number of holding
  registers to be read as well as the starting address, and the number of
  holding registers. The data to be written is specified in the transmit
  buffer.

  @param u16ReadAddress address of the first holding register (0x0000..0xFFFF)
  @param u16ReadQty quantity of holding registers to read (1..125, enforced by remote device)
  @param u16WriteAddress address of the first holding register (0x0000..0xFFFF)
  @param u16WriteQty quantity of holding registers to write (1..121, enforced by remote device)
  @return 0 on success; exception number on failure
  @ingroup register
*/
bool NonBlockingModbusMaster::readWriteMultipleRegisters(uint8_t slaveId, uint16_t u16ReadAddress, uint16_t u16ReadQty, uint16_t u16WriteAddress, uint16_t u16WriteQty, ResultHandler handlerFn) {
  if (!init(slaveId, handlerFn)) {
    return false;
  }
  _u16ReadAddress = u16ReadAddress;
  _u16ReadQty = u16ReadQty;
  _u16WriteAddress = u16WriteAddress;
  _u16WriteQty = u16WriteQty;
  NonBlockingModbusMasterTransaction(ku8MBReadWriteMultipleRegisters);
  return true;
}
bool NonBlockingModbusMaster::readWriteMultipleRegisters(uint8_t slaveId, uint16_t u16ReadAddress,   uint16_t u16ReadQty, ResultHandler handlerFn) {
  if (!init(slaveId, handlerFn)) {
    return false;
  }
  _u16ReadAddress = u16ReadAddress;
  _u16ReadQty = u16ReadQty;
  _u16WriteQty = _u8TransmitBufferIndex;
  NonBlockingModbusMasterTransaction(ku8MBReadWriteMultipleRegisters);
  return true;
}


/* _____PRIVATE FUNCTIONS____________________________________________________ */
/**
  Modbus transaction engine.
  Sequence:
  - assemble Modbus Request Application Data Unit (ADU),

  @param u8MBFunction Modbus function (0x01..0xFF)
*/
void NonBlockingModbusMaster::NonBlockingModbusMasterTransaction(uint8_t _u8MBFunction) {
  u8MBFunction = _u8MBFunction;
  uint8_t i, u8Qty;
  uint16_t u16CRC;
  u8ModbusADUSize = 0;
  u32StartTime = millis();
  u8BytesLeft = 8;
  // u8MBStatus set to ku8MBSuccess; by init
  // modbusState set to MB_START by init


  // assemble Modbus Request Application Data Unit
  u8ModbusADU[u8ModbusADUSize++] = _u8MBSlave;
  u8ModbusADU[u8ModbusADUSize++] = u8MBFunction;

  switch (u8MBFunction)  {
    case ku8MBReadCoils:
    case ku8MBReadDiscreteInputs:
    case ku8MBReadInputRegisters:
    case ku8MBReadHoldingRegisters:
    case ku8MBReadWriteMultipleRegisters:
      u8ModbusADU[u8ModbusADUSize++] = highByte(_u16ReadAddress);
      u8ModbusADU[u8ModbusADUSize++] = lowByte(_u16ReadAddress);
      u8ModbusADU[u8ModbusADUSize++] = highByte(_u16ReadQty);
      u8ModbusADU[u8ModbusADUSize++] = lowByte(_u16ReadQty);
      break;
  }

  switch (u8MBFunction)  {
    case ku8MBWriteSingleCoil:
    case ku8MBMaskWriteRegister:
    case ku8MBWriteMultipleCoils:
    case ku8MBWriteSingleRegister:
    case ku8MBWriteMultipleRegisters:
    case ku8MBReadWriteMultipleRegisters:
      u8ModbusADU[u8ModbusADUSize++] = highByte(_u16WriteAddress);
      u8ModbusADU[u8ModbusADUSize++] = lowByte(_u16WriteAddress);
      break;
  }

  switch (u8MBFunction)  {
    case ku8MBWriteSingleCoil:
      u8ModbusADU[u8ModbusADUSize++] = highByte(_u16WriteQty);
      u8ModbusADU[u8ModbusADUSize++] = lowByte(_u16WriteQty);
      break;

    case ku8MBWriteSingleRegister:
      u8ModbusADU[u8ModbusADUSize++] = highByte(_u16TransmitBuffer[0]);
      u8ModbusADU[u8ModbusADUSize++] = lowByte(_u16TransmitBuffer[0]);
      break;

    case ku8MBWriteMultipleCoils:
      u8ModbusADU[u8ModbusADUSize++] = highByte(_u16WriteQty);
      u8ModbusADU[u8ModbusADUSize++] = lowByte(_u16WriteQty);
      u8Qty = (_u16WriteQty % 8) ? ((_u16WriteQty >> 3) + 1) : (_u16WriteQty >> 3);
      u8ModbusADU[u8ModbusADUSize++] = u8Qty;
      for (i = 0; i < u8Qty; i++)  {
        if (u8ModbusADUSize >= maxTXLen) {
          u8MBStatus = ku8MBDataToLong;
          return;
        }
        switch (i % 2)
        {
          case 0: // i is even
            u8ModbusADU[u8ModbusADUSize++] = lowByte(_u16TransmitBuffer[i >> 1]);
            break;

          case 1: // i is odd
            u8ModbusADU[u8ModbusADUSize++] = highByte(_u16TransmitBuffer[i >> 1]);
            break;
        }
      }
      break;

    case ku8MBWriteMultipleRegisters:
    case ku8MBReadWriteMultipleRegisters:
      u8ModbusADU[u8ModbusADUSize++] = highByte(_u16WriteQty);
      u8ModbusADU[u8ModbusADUSize++] = lowByte(_u16WriteQty);
      u8ModbusADU[u8ModbusADUSize++] = lowByte(_u16WriteQty << 1);

      for (i = 0; i < lowByte(_u16WriteQty); i++)  {
        if (u8ModbusADUSize >= maxTXLen) {
          u8MBStatus = ku8MBDataToLong;
          return;
        }
        u8ModbusADU[u8ModbusADUSize++] = highByte(_u16TransmitBuffer[i]);
        u8ModbusADU[u8ModbusADUSize++] = lowByte(_u16TransmitBuffer[i]);
      }
      break;

    case ku8MBMaskWriteRegister:
      u8ModbusADU[u8ModbusADUSize++] = highByte(_u16TransmitBuffer[0]);
      u8ModbusADU[u8ModbusADUSize++] = lowByte(_u16TransmitBuffer[0]);
      u8ModbusADU[u8ModbusADUSize++] = highByte(_u16TransmitBuffer[1]);
      u8ModbusADU[u8ModbusADUSize++] = lowByte(_u16TransmitBuffer[1]);
      break;
  }

  // append CRC
  u16CRC = 0xFFFF;
  for (i = 0; i < u8ModbusADUSize; i++)   {
    u16CRC = crc16_update(u16CRC, u8ModbusADU[i]);
  }
  u8ModbusADU[u8ModbusADUSize++] = lowByte(u16CRC);
  u8ModbusADU[u8ModbusADUSize++] = highByte(u16CRC);
  u8ModbusADU[u8ModbusADUSize] = 0;
}



bool NonBlockingModbusMaster::justFinished() {
  if (modbusState == MB_IDLE) {
    return false;
  }
  // handle start up errors
  if ((modbusState == MB_ERROR) || (modbusState == MB_END)) {
    modbusState = MB_IDLE;
    if (resultHandler) {
      resultHandler(*this); // can start another cmd
    }
    _u8ResponseBufferIndex = 0; // reset for justFinished to use again
    if (modbusState == MB_IDLE) {
      return true;
    } else {
      return false;
    }
  }
  // else handle preDelay
  if (modbusState == MB_START) {
    u32StartTime = micros();
    modbusState = MB_PRE_DELAY;
    return false;
  }
  if (modbusState == MB_PRE_DELAY) {
    if ((micros() - u32StartTime) > (_preDelay_us + (_oneTimeDelay_ms * 1000))) {
      modbusState = MB_SEND; // continue below
      _oneTimeDelay_ms = 0; // clear it now
    } else {
      return false;
    }
  }

  if (modbusState == MB_SEND) {
    int i = 0;
    // flush receive buffer before transmitting request
    if (modbusDebugPtr) {
      modbusDebugPtr->println();
      modbusDebugPtr->print("Flush input : ");
    }
    while (1) {
      int i = _serial->read();
      if (i != -1) {
        if (modbusDebugPtr) {
          printHex(i,modbusDebugPtr);
        }
      } else {
        break;
      }
    }
    if (modbusDebugPtr) {
      modbusDebugPtr->println(" -- end flush -- ");
    }
    if (modbusDebugPtr) {
      modbusDebugPtr->print("Send Cmd : ");
    }
    if (_preTransmission)
    {
      _preTransmission();
    }
    for (i = 0; i < u8ModbusADUSize; i++) {
      _serial->write(u8ModbusADU[i]);
      if (modbusDebugPtr) {
        printHex(u8ModbusADU[i],modbusDebugPtr);
      }
    }
    if (_postTransmission)
    {
      _postTransmission();
    }
    if (modbusDebugPtr) {
      modbusDebugPtr->println();
    }
    u8ModbusADUSize = 0;
    u32StartTime = micros();
    modbusState = MB_POST_DELAY;
    return false;
  }

  if (modbusState == MB_POST_DELAY) {
    if ((micros() - u32StartTime) > _postDelay_us) {
      modbusState = MB_RECEIVE; // continue below
      u32StartTime = millis();
      if (modbusDebugPtr) {
        modbusDebugPtr->print("Received : ");
      }
    } else {
      return false;
    }
  }

  if (modbusState == MB_RECEIVE) {
    int i = 0;
    if ((millis() - u32StartTime) > ku16MBResponseTimeout) {
      modbusState = MB_ERROR;
      u8MBStatus = ku8MBResponseTimedOut;
      return false;
    }
    // else
    // loop until we run out of time or bytes, or an error occurs
    while (u8BytesLeft) {
      if (_serial->available()) {
        u8ModbusADU[u8ModbusADUSize++] = _serial->read();
        if (modbusDebugPtr) {
          printHex(u8ModbusADU[u8ModbusADUSize - 1],modbusDebugPtr);
        }
        u8BytesLeft--;
      } else {
        return false; // no more available for now
      }

      // evaluate slave ID, function code once enough bytes have been read
      if (u8ModbusADUSize == 5) {
        // verify response is for correct Modbus slave
        if (u8ModbusADU[0] != _u8MBSlave)     {
          modbusState = MB_ERROR;
          u8MBStatus = ku8MBInvalidSlaveID;
          return false;
        }

        // verify response is for correct Modbus function code (mask exception bit 7)
        if ((u8ModbusADU[1] & 0x7F) != u8MBFunction)  {
          modbusState = MB_ERROR;
          u8MBStatus = ku8MBInvalidFunction;
          return false;
        }

        // check whether Modbus exception occurred; return Modbus Exception Code
        if (bitRead(u8ModbusADU[1], 7)) {
          u8MBStatus = u8ModbusADU[2];
          modbusState = MB_ERROR;
          return false;
        }

        // evaluate returned Modbus function code
        switch (u8ModbusADU[1]) {
          case ku8MBReadCoils:
          case ku8MBReadDiscreteInputs:
          case ku8MBReadInputRegisters:
          case ku8MBReadHoldingRegisters:
          case ku8MBReadWriteMultipleRegisters:
            u8BytesLeft = u8ModbusADU[2];
            break;

          case ku8MBWriteSingleCoil:
          case ku8MBWriteMultipleCoils:
          case ku8MBWriteSingleRegister:
          case ku8MBWriteMultipleRegisters:
            u8BytesLeft = 3;
            break;

          case ku8MBMaskWriteRegister:
            u8BytesLeft = 5;
            break;
        }
      } // end if (u8ModbusADUSize == 5) {
    } // end while (u8BytesLeft) {

    uint16_t u16CRC;
    // verify response is large enough to inspect further
    if (u8ModbusADUSize >= 5) {
      // calculate CRC
      u16CRC = 0xFFFF;
      for (i = 0; i < (u8ModbusADUSize - 2); i++) {
        u16CRC = crc16_update(u16CRC, u8ModbusADU[i]);
      }
      // verify CRC
      if ((lowByte(u16CRC) != u8ModbusADU[u8ModbusADUSize - 2] ||
           highByte(u16CRC) != u8ModbusADU[u8ModbusADUSize - 1])) {
        u8MBStatus = ku8MBInvalidCRC;
        modbusState = MB_ERROR;
        return false;
      }
    } else { // not >= 5
      u8MBStatus = ku8MBResponseToShort;
      modbusState = MB_ERROR;
      return false;
    }

    // disassemble ADU into words
    // evaluate returned Modbus function code
    switch (u8ModbusADU[1]) {
      case ku8MBReadCoils:
      case ku8MBReadDiscreteInputs:
        // load bytes into word; response bytes are ordered L, H, L, H, ...
        for (i = 0; i < (u8ModbusADU[2] >> 1); i++)  {
          if (i < ku8MaxBufferSize) {
            _u16ResponseBuffer[i] = word(u8ModbusADU[2 * i + 4], u8ModbusADU[2 * i + 3]);
          }
          _u8ResponseBufferLength = i + 1; // was just i in ModbusMaster
        }

        // in the event of an odd number of bytes, load last byte into zero-padded word
        if (u8ModbusADU[2] % 2) {
          if (i < ku8MaxBufferSize) {
            _u16ResponseBuffer[i] = word(0, u8ModbusADU[2 * i + 3]);
          }
          _u8ResponseBufferLength = i + 1; 
        }
        break;

      case ku8MBReadInputRegisters:
      case ku8MBReadHoldingRegisters:
      case ku8MBReadWriteMultipleRegisters:
        // load bytes into word; response bytes are ordered H, L, H, L, ...
        for (i = 0; i < (u8ModbusADU[2] >> 1); i++) {
          if (i < ku8MaxBufferSize) {
            _u16ResponseBuffer[i] = word(u8ModbusADU[2 * i + 3], u8ModbusADU[2 * i + 4]);
          }
          _u8ResponseBufferLength = i + 1; // was just i in ModbusMaster
        }
        break;
    }
    // finished collecting response
    _u8TransmitBufferIndex = 0;
   // u16TransmitBufferLength = 0; keep the Tx buffer intack for retries
    _u8ResponseBufferIndex = 0;
    if (modbusDebugPtr) {
      modbusDebugPtr->println();
    }
    modbusState = MB_END;
    return false; // u8MBStatus saved // next call will return true and set to IDLE if not running another chained cmd
  }  // end  if (mobusState == MB_RECEIVE) {

  // else
  u8MBStatus = ku8MBStateError;
  modbusState = MB_ERROR;
  return false;
}
