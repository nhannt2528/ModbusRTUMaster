#include "ModbusRtuMaster.h"
// ML_ModbusRtuMaster ML_RtuMaster;
void ML_ModbusRtuMaster::begin(Stream *port)
{
    _port = port;
}
modbus_status_reponse_t ML_ModbusRtuMaster::readCoils(uint8_t slaveID, uint16_t regAddr, uint16_t qty, bool *data)
{
    _txBuff[0] = slaveID;
    _txBuff[1] = MB_FC_READ_COILS;
    _txBuff[2] = highByte(regAddr);
    _txBuff[3] = lowByte(regAddr);
    _txBuff[4] = highByte(qty);
    _txBuff[5] = lowByte(qty);
    writeOutBuff(_txBuff, 6);
    return readBoolValue(data, qty);
}
modbus_status_reponse_t ML_ModbusRtuMaster::readDiscreteInputs(uint8_t slaveID, uint16_t regAddr, uint16_t qty, bool *data)
{
    _txBuff[0] = slaveID;
    _txBuff[1] = MB_FC_READ_DISCRETE_INPUTS;
    _txBuff[2] = highByte(regAddr);
    _txBuff[3] = lowByte(regAddr);
    _txBuff[4] = highByte(qty);
    _txBuff[5] = lowByte(qty);
    writeOutBuff(_txBuff, 6);
    return readBoolValue(data, qty);
}
modbus_status_reponse_t ML_ModbusRtuMaster::readHoldingRegisters(uint8_t slaveID, uint16_t regAddr, uint16_t qty, uint16_t *data)
{
    _txBuff[0] = slaveID;
    _txBuff[1] = MB_FC_READ_HOLDING_REGISTERS;
    _txBuff[2] = highByte(regAddr);
    _txBuff[3] = lowByte(regAddr);
    _txBuff[4] = highByte(qty);
    _txBuff[5] = lowByte(qty);
    writeOutBuff(_txBuff, 6);
    return readUint16Value(data, qty);
}
modbus_status_reponse_t ML_ModbusRtuMaster::readInputRegisters(uint8_t slaveID, uint16_t regAddr, uint16_t qty, uint16_t *data)
{
    _txBuff[0] = slaveID;
    _txBuff[1] = MB_FC_READ_INPUT_REGISTERS;
    _txBuff[2] = highByte(regAddr);
    _txBuff[3] = lowByte(regAddr);
    _txBuff[4] = highByte(qty);
    _txBuff[5] = lowByte(qty);
    writeOutBuff(_txBuff, 6);
    return readUint16Value(data, qty);
}

modbus_status_reponse_t ML_ModbusRtuMaster::writeSingleCoil(uint8_t slaveID, uint16_t regAddr, bool value)
{
    byte state = 0x00;
    value == true ? state = 0xFF : state = 0x00;
    _txBuff[0] = slaveID;
    _txBuff[1] = MB_FC_WRITE_SINGLE_COIL;
    _txBuff[2] = highByte(regAddr);
    _txBuff[3] = lowByte(regAddr);
    _txBuff[4] = state;
    _txBuff[5] = 0x00;
    writeOutBuff(_txBuff, 6);
   return readResopnseWrite();
}

modbus_status_reponse_t ML_ModbusRtuMaster::writeSingleRegister(uint8_t slaveID, uint16_t regAddr, uint16_t value)
{

    int len = 1;
    _txBuff[0] = slaveID;
    _txBuff[1] = MB_FC_WRITE_SINGLE_REGISTER;
    _txBuff[2] = highByte(regAddr);
    _txBuff[3] = lowByte(regAddr);
    _txBuff[4] = highByte(value);
    _txBuff[5] = lowByte(value);
    writeOutBuff(_txBuff, 6);
   return readResopnseWrite();
}

modbus_status_reponse_t ML_ModbusRtuMaster::writeMultipleCoils(uint8_t slaveID, uint16_t regAddr, bool *value)
{
}
modbus_status_reponse_t ML_ModbusRtuMaster::writeMultipleRegister(uint8_t slaveID, uint16_t regAddr, uint16_t *value)
{
}
bool ML_ModbusRtuMaster::responseRtu(uint16_t *data, int &len)
{

    bool reponse_ok = false;
    len = 0;
    int rxBuffSize = _port->available();
    byte rxBuff[rxBuffSize];

    if (rxBuffSize == 0)
    {
        reponse_ok = false;
    }
    else
    {
        while (_port->available())
        {
            _port->readBytes(rxBuff, rxBuffSize);
            _port->flush();
            _rxComplete = true;
        }
        uint16_t CRC_rxBuff = (rxBuff[rxBuffSize - 1] << 8) + (rxBuff[rxBuffSize - 2]);
        uint16_t CRC_rxCheck = ModRTU_CRC(rxBuff, rxBuffSize - 2);

        if (CRC_rxBuff != CRC_rxCheck)
        {
            errorCode = CRC_ERROR;
            returnErrorCode();
        }
        else
        {
            if (rxBuffSize == 5)
            {
                errorCode = rxBuff[2];
                returnErrorCode();
            }
            else
            {

                switch (rxBuff[1])
                {
                case MB_FC_READ_DISCRETE_INPUTS:
                case MB_FC_READ_COILS:
                {

                    len = rxBuff[2];

                    for (int i = 0; i < len; i++)
                    {
                        if (len == 1)
                        {
                            //  data[i] = (rxBuff[(i * 2) + 4] << 8) + rxBuff[(i * 2) + 3];
                            data[i] = rxBuff[3];
                        }
                        else
                        {
                            data[i] = (rxBuff[(i * 2) + 4] << 8) + rxBuff[(i * 2) + 3];
                        }
                    }
                    break;
                }
                case MB_FC_READ_INPUT_REGISTERS:
                case MB_FC_READ_HOLDING_REGISTERS:
                {
                    len = rxBuff[2] / 2;
                    for (int i = 0; i < len; i++)
                    {
                        data[i] = (rxBuff[(i * 2) + 3] << 8) + rxBuff[(i * 2) + 4];
                    }
                    break;
                }
                case MB_FC_WRITE_SINGLE_COIL:
                case MB_FC_WRITE_SINGLE_REGISTER:
                {
                    // Serial.println("RESPONSE OK");
                    break;
                }
                }
            }
            reponse_ok = true;
        }
    }
    return reponse_ok;
}

void ML_ModbusRtuMaster::writeOutBuff(byte buf[], int len)
{
    uint16_t CRC = ModRTU_CRC(buf, len);
    buf[len] = lowByte(CRC);
    buf[len + 1] = highByte(CRC);
    _port->write(_txBuff, len + 2);
}

uint16_t ML_ModbusRtuMaster::ModRTU_CRC(byte buf[], int len)
{
    uint16_t crc = 0xFFFF;
    for (int pos = 0; pos < len; pos++)
    {
        crc ^= (uint16_t)buf[pos]; // XOR byte into least sig. byte of crc

        for (int i = 8; i != 0; i--)
        { // Loop over each bit
            if ((crc & 0x0001) != 0)
            {              // If the LSB is set
                crc >>= 1; // Shift right and XOR 0xA001
                crc ^= 0xA001;
            }
            else           // Else LSB is not set
                crc >>= 1; // Just shift right
        }
    }
    return crc;
}
bool ML_ModbusRtuMaster::returnErrorCode()
{
    bool fail = true;
    switch (errorCode)
    {
    case MB_RESP_OK:
    {
        fail = false;
        break;
    }
    case MB_EX_ILLEGAL_FUNCTION:
    {
        Serial.println("ILLEGAL_FUNCTION");

        break;
    }
    case MB_EX_ILLEGAL_DATA_ADDRESS:
    {
        Serial.println("ILLEGAL_DATA_ADDRESS");

        break;
    }
    case MB_EX_ILLEGAL_DATA_VALUE:
    {
        Serial.println("ILLEGAL_DATA_VALUE");

        break;
    }
    case MB_EX_SERVER_DEVICE_FAILURE:
    {
        Serial.println("SERVER_DEVICE_FAILURE");

        break;
    }
    case CRC_ERROR:
    {
        Serial.println("CRC_ERROR");
        break;
    }
    }
    return fail;
}
modbus_status_reponse_t ML_ModbusRtuMaster::readBoolValue(bool *data, int qty)
{
    modbus_status_reponse_t resual_reponse = READ_SUCCESS;
    uint16_t data_u16[MAX_PACKED_RTU];
    unsigned long preTime = millis();
    int len = 0;
    int rx_length;
    if (qty % 8 != 0)
    {
        rx_length = (qty / 8) + 1;
    }
    else
    {
        rx_length = qty / 8;
    }
    while ((_port->available() < rx_length + 5))
    {

        if (millis() - preTime > TIMEOUT_REQUEST)
        {
            resual_reponse = READ_ERROR;
            Serial.println("REQUEST TIMEOUT");
            break;
        }
    }
    responseRtu(data_u16, len);
    for (int i = 0; i < len; i++)
    {
        for (int j = 0; j < 16; j++)
        {
            data[(16 * i) + j] = bitRead(data_u16[i], j);
            // data[(16 * i) + j]=1;
            // Serial.print(data[(j * i) + 16]);
        }
    }
    return resual_reponse;
}
modbus_status_reponse_t ML_ModbusRtuMaster::readUint16Value(uint16_t *data, int qty)
{
    modbus_status_reponse_t result_reponse = READ_SUCCESS;
    unsigned long preTime = millis();
    int len = 0;

    while (_port->available() < (qty * 2) + 5)
    {
        if (millis() - preTime > TIMEOUT_REQUEST)
        {
            result_reponse = READ_ERROR;
            Serial.println("REQUEST TIMEOUT");
            break;
        }
    }
    responseRtu(data, len);
    return result_reponse;
}
modbus_status_reponse_t ML_ModbusRtuMaster::readResopnseWrite()
{
    modbus_status_reponse_t result_reponse = WRITE_SUCCESS;
    unsigned long preTime = millis();
    int len = 0;

    while (_port->available() < 8)
    {
        if (millis() - preTime > TIMEOUT_REQUEST)
        {
            result_reponse = WRITE_ERROR;
            Serial.println("REQUEST TIMEOUT");
            break;
        }
    }
    responseRtu(NULL, len);
    return result_reponse;
}