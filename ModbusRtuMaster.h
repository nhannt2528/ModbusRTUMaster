#ifndef _MODBUSRTUMASTER_H_
#define _MODBUSRTUMASTER_H_
/*
Modbus farme Request
[0] SLAVE ID         1 byte
[1] Funcion Code     1 byte
[3] START ADDR       2 bytes
[5] Quantily         2 bytes
[7] CRC              2 bytes

modbus farme Response
[0] SLAVE ID        1 byte
[1] Funcion Code    1 byte
[2]

*/

#define MAX_PACKED_RTU 500
#define MB_FC_READ_COILS 0x01
#define MB_FC_READ_DISCRETE_INPUTS 0x02
#define MB_FC_READ_HOLDING_REGISTERS 0x03
#define MB_FC_READ_INPUT_REGISTERS 0x04
#define MB_FC_WRITE_SINGLE_COIL 0x05
#define MB_FC_WRITE_SINGLE_REGISTER 0x06
#define MB_FC_WRITE_MULTIPLE_COILS 0x0F
#define MB_FC_WRITE_MULTIPLE_REGISTERS 0x10

#define MB_RESP_OK 0x00
#define MB_EX_ILLEGAL_FUNCTION 0x01
#define MB_EX_ILLEGAL_DATA_ADDRESS 0x02
#define MB_EX_ILLEGAL_DATA_VALUE 0x03
#define MB_EX_SERVER_DEVICE_FAILURE 0x04
#define MB_EX_ACKNOWLEDGE 0x05
#define MB_EX_SERVER_DEVICE_BUSY 0x06
#define MB_EX_MEMORY_PARITY_ERROR 0x08
#define MB_EX_GATEWAY_PATH_UNAVAILABLE 0x0A
#define MB_EX_DEVICE_FAILED_TO_RESPOND 0x0B

#define CRC_ERROR 0x0C

#define TIMEOUT_REQUEST 5000 // thoi gian timeout
#define RS485_TX 14
#define RS485_RX 12
#include <Arduino.h>
#include "HardwareSerial.h"



typedef enum
{
    READ_SUCCESS,
    READ_ERROR,
    WRITE_SUCCESS,
    WRITE_ERROR

} modbus_status_reponse_t;

class ML_ModbusRtuMaster
{
private:
    uint16_t _data[MAX_PACKED_RTU];
    bool _rxComplete = false;
    Stream *_port;
    bool responseRtu(uint16_t *data, int &len);
    uint16_t ModRTU_CRC(byte buf[], int len);
    byte _txBuff[8];
    byte errorCode = MB_RESP_OK;
    bool returnErrorCode();
    modbus_status_reponse_t readBoolValue(bool *data, int qty);
    modbus_status_reponse_t readUint16Value(uint16_t *data, int qty);
    modbus_status_reponse_t readResopnseWrite();
    void writeOutBuff(byte buf[], int len);

public:
    void begin(Stream *port);
    modbus_status_reponse_t readCoils(uint8_t slaveID, uint16_t regAddr, uint16_t qty, bool *data);
    modbus_status_reponse_t readDiscreteInputs(uint8_t slaveID, uint16_t regAddr, uint16_t qty, bool *data);
    modbus_status_reponse_t readHoldingRegisters(uint8_t slaveID, uint16_t regAddr, uint16_t qty, uint16_t *data);
    modbus_status_reponse_t readInputRegisters(uint8_t slaveID, uint16_t regAddr, uint16_t qty, uint16_t *data);
    modbus_status_reponse_t writeSingleCoil(uint8_t slaveID, uint16_t regAddr, bool value);
    modbus_status_reponse_t writeSingleRegister(uint8_t slaveID, uint16_t regAddr, uint16_t value);
    modbus_status_reponse_t writeMultipleCoils(uint8_t slaveID, uint16_t regAddr, bool *value);
    modbus_status_reponse_t writeMultipleRegister(uint8_t slaveID, uint16_t regAddr, uint16_t *value);
};

// extern ML_ModbusRtuMaster ML_RtuMaster;
#endif