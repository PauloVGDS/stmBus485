/**
 * ModbusRTU.cpp - Implementação da biblioteca Modbus RTU
 */

#include "ModbusRTU.h"

// Tabela CRC pré-calculada para melhor performance
static const uint16_t crcTable[] PROGMEM = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

/**
 * Calcula CRC-16 Modbus usando tabela
 */
uint16_t modbusCRC16(uint8_t* data, uint8_t len) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t index = (crc ^ data[i]) & 0xFF;
        crc = (crc >> 8) ^ pgm_read_word(&crcTable[index]);
    }
    return crc;
}

// ==================== MODBUS SLAVE ====================

ModbusSlave::ModbusSlave(HardwareSerial& serial, uint8_t dePin, uint8_t slaveAddr)
    : _serial(serial), _dePin(dePin), _slaveAddr(slaveAddr),
      _bufferLen(0), _lastByteTime(0),
      _readCoilCallback(nullptr), _writeCoilCallback(nullptr) {
}

void ModbusSlave::begin(unsigned long baudRate) {
    _serial.begin(baudRate);
    pinMode(_dePin, OUTPUT);
    setTxMode(false);  // Começa em modo recepção
}

void ModbusSlave::setTxMode(bool transmit) {
    digitalWrite(_dePin, transmit ? HIGH : LOW);
    if (transmit) {
        delayMicroseconds(100);  // Tempo para estabilizar
    }
}

void ModbusSlave::onReadCoil(bool (*callback)(uint16_t coilAddr)) {
    _readCoilCallback = callback;
}

void ModbusSlave::onWriteCoil(void (*callback)(uint16_t coilAddr, bool value)) {
    _writeCoilCallback = callback;
}

void ModbusSlave::poll() {
    // Lê bytes disponíveis
    while (_serial.available()) {
        if (_bufferLen < MODBUS_BUFFER_SIZE) {
            _buffer[_bufferLen++] = _serial.read();
            _lastByteTime = millis();
        } else {
            _serial.read();  // Descarta se buffer cheio
        }
    }

    // Verifica timeout de frame (3.5 caracteres ~ 4ms a 9600bps)
    if (_bufferLen > 0 && (millis() - _lastByteTime) > 4) {
        processMessage();
        _bufferLen = 0;
    }
}

void ModbusSlave::processMessage() {
    // Frame mínimo: Addr(1) + FC(1) + Data(2) + CRC(2) = 6 bytes
    if (_bufferLen < 6) return;

    // Verifica endereço (0 = broadcast, nosso endereço)
    if (_buffer[0] != _slaveAddr && _buffer[0] != 0) return;

    // Verifica CRC
    uint16_t receivedCRC = _buffer[_bufferLen - 2] | (_buffer[_bufferLen - 1] << 8);
    uint16_t calculatedCRC = modbusCRC16(_buffer, _bufferLen - 2);
    if (receivedCRC != calculatedCRC) return;

    uint8_t functionCode = _buffer[1];

    switch (functionCode) {
        case FC_READ_COILS: {
            // Frame: Addr(1) + FC(1) + StartAddr(2) + Quantity(2) + CRC(2) = 8 bytes
            if (_bufferLen != 8) return;

            uint16_t startAddr = (_buffer[2] << 8) | _buffer[3];
            uint16_t quantity = (_buffer[4] << 8) | _buffer[5];

            // Valida quantidade (1-16 coils para simplificar)
            if (quantity < 1 || quantity > 16) {
                sendException(functionCode, EX_ILLEGAL_VALUE);
                return;
            }

            if (_readCoilCallback == nullptr) {
                sendException(functionCode, EX_ILLEGAL_FUNCTION);
                return;
            }

            // Prepara resposta
            uint8_t byteCount = (quantity + 7) / 8;
            uint8_t response[3 + byteCount];
            response[0] = _slaveAddr;
            response[1] = functionCode;
            response[2] = byteCount;

            // Lê cada coil
            for (uint8_t i = 0; i < byteCount; i++) {
                response[3 + i] = 0;
            }

            for (uint16_t i = 0; i < quantity; i++) {
                bool value = _readCoilCallback(startAddr + i);
                if (value) {
                    response[3 + (i / 8)] |= (1 << (i % 8));
                }
            }

            sendResponse(response, 3 + byteCount);
            break;
        }

        case FC_WRITE_SINGLE_COIL: {
            // Frame: Addr(1) + FC(1) + CoilAddr(2) + Value(2) + CRC(2) = 8 bytes
            if (_bufferLen != 8) return;

            uint16_t coilAddr = (_buffer[2] << 8) | _buffer[3];
            uint16_t value = (_buffer[4] << 8) | _buffer[5];

            // Valida valor (deve ser 0xFF00 ou 0x0000)
            if (value != COIL_ON && value != COIL_OFF) {
                sendException(functionCode, EX_ILLEGAL_VALUE);
                return;
            }

            if (_writeCoilCallback == nullptr) {
                sendException(functionCode, EX_ILLEGAL_FUNCTION);
                return;
            }

            // Executa a escrita
            _writeCoilCallback(coilAddr, value == COIL_ON);

            // Echo da requisição como resposta (padrão Modbus)
            uint8_t response[6];
            response[0] = _slaveAddr;
            response[1] = functionCode;
            response[2] = _buffer[2];  // CoilAddr High
            response[3] = _buffer[3];  // CoilAddr Low
            response[4] = _buffer[4];  // Value High
            response[5] = _buffer[5];  // Value Low

            sendResponse(response, 6);
            break;
        }

        default:
            sendException(functionCode, EX_ILLEGAL_FUNCTION);
            break;
    }
}

void ModbusSlave::sendResponse(uint8_t* data, uint8_t len) {
    // Não responde a broadcast
    if (_buffer[0] == 0) return;

    // Calcula CRC
    uint16_t crc = modbusCRC16(data, len);

    // Entra em modo transmissão
    setTxMode(true);

    // Envia dados
    _serial.write(data, len);
    _serial.write(crc & 0xFF);         // CRC Low
    _serial.write((crc >> 8) & 0xFF);  // CRC High

    // Aguarda transmissão completar
    _serial.flush();

    // Volta para modo recepção
    setTxMode(false);
}

void ModbusSlave::sendException(uint8_t functionCode, uint8_t exceptionCode) {
    uint8_t response[3];
    response[0] = _slaveAddr;
    response[1] = functionCode | 0x80;  // Bit 7 indica exceção
    response[2] = exceptionCode;

    sendResponse(response, 3);
}

// ==================== MODBUS MASTER ====================

ModbusMaster::ModbusMaster(HardwareSerial& serial, uint8_t dePin)
    : _serial(serial), _dePin(dePin), _lastError(0) {
}

void ModbusMaster::begin(unsigned long baudRate) {
    _serial.begin(baudRate);
    pinMode(_dePin, OUTPUT);
    setTxMode(false);  // Começa em modo recepção
}

void ModbusMaster::setTxMode(bool transmit) {
    digitalWrite(_dePin, transmit ? HIGH : LOW);
    if (transmit) {
        delayMicroseconds(100);  // Tempo para estabilizar
    }
}

uint8_t ModbusMaster::getLastError() {
    return _lastError;
}

void ModbusMaster::sendRequest(uint8_t* data, uint8_t len) {
    // Calcula CRC
    uint16_t crc = modbusCRC16(data, len);

    // Limpa buffer de recepção
    while (_serial.available()) _serial.read();

    // Entra em modo transmissão
    setTxMode(true);

    // Envia dados
    _serial.write(data, len);
    _serial.write(crc & 0xFF);         // CRC Low
    _serial.write((crc >> 8) & 0xFF);  // CRC High

    // Aguarda transmissão completar
    _serial.flush();

    // Volta para modo recepção
    setTxMode(false);
}

int ModbusMaster::receiveResponse(uint8_t* buffer, uint8_t maxLen, unsigned long timeout) {
    unsigned long startTime = millis();
    uint8_t len = 0;
    unsigned long lastByteTime = 0;

    while ((millis() - startTime) < timeout) {
        if (_serial.available()) {
            if (len < maxLen) {
                buffer[len++] = _serial.read();
                lastByteTime = millis();
            } else {
                _serial.read();  // Descarta
            }
        }

        // Frame completo se timeout de caractere (4ms a 9600bps)
        if (len > 0 && (millis() - lastByteTime) > 4) {
            break;
        }
    }

    return len;
}

bool ModbusMaster::writeSingleCoil(uint8_t slaveAddr, uint16_t coilAddr, bool value) {
    _lastError = 0;

    uint8_t request[6];
    request[0] = slaveAddr;
    request[1] = FC_WRITE_SINGLE_COIL;
    request[2] = (coilAddr >> 8) & 0xFF;  // Addr High
    request[3] = coilAddr & 0xFF;          // Addr Low
    request[4] = value ? 0xFF : 0x00;      // Value High
    request[5] = 0x00;                     // Value Low

    sendRequest(request, 6);

    // Aguarda resposta
    uint8_t response[MODBUS_BUFFER_SIZE];
    int len = receiveResponse(response, MODBUS_BUFFER_SIZE, MODBUS_TIMEOUT);

    // Resposta mínima: 8 bytes (echo) ou 5 bytes (exceção + CRC)
    if (len < 5) {
        _lastError = 0xFF;  // Timeout/Sem resposta
        return false;
    }

    // Verifica CRC
    uint16_t receivedCRC = response[len - 2] | (response[len - 1] << 8);
    uint16_t calculatedCRC = modbusCRC16(response, len - 2);
    if (receivedCRC != calculatedCRC) {
        _lastError = 0xFE;  // Erro de CRC
        return false;
    }

    // Verifica exceção
    if (response[1] & 0x80) {
        _lastError = response[2];
        return false;
    }

    // Verifica echo correto
    if (len != 8) {
        _lastError = 0xFD;  // Resposta inválida
        return false;
    }

    return true;
}

bool ModbusMaster::readCoils(uint8_t slaveAddr, uint16_t startAddr, uint16_t quantity, uint8_t* result) {
    _lastError = 0;

    if (quantity < 1 || quantity > 16) {
        _lastError = EX_ILLEGAL_VALUE;
        return false;
    }

    uint8_t request[6];
    request[0] = slaveAddr;
    request[1] = FC_READ_COILS;
    request[2] = (startAddr >> 8) & 0xFF;  // Addr High
    request[3] = startAddr & 0xFF;          // Addr Low
    request[4] = (quantity >> 8) & 0xFF;    // Quantity High
    request[5] = quantity & 0xFF;           // Quantity Low

    sendRequest(request, 6);

    // Aguarda resposta
    uint8_t response[MODBUS_BUFFER_SIZE];
    int len = receiveResponse(response, MODBUS_BUFFER_SIZE, MODBUS_TIMEOUT);

    // Resposta mínima: 5 bytes (addr + fc + count + data + crc)
    if (len < 5) {
        _lastError = 0xFF;  // Timeout/Sem resposta
        return false;
    }

    // Verifica CRC
    uint16_t receivedCRC = response[len - 2] | (response[len - 1] << 8);
    uint16_t calculatedCRC = modbusCRC16(response, len - 2);
    if (receivedCRC != calculatedCRC) {
        _lastError = 0xFE;  // Erro de CRC
        return false;
    }

    // Verifica exceção
    if (response[1] & 0x80) {
        _lastError = response[2];
        return false;
    }

    // Extrai dados
    uint8_t byteCount = response[2];
    for (uint8_t i = 0; i < byteCount; i++) {
        result[i] = response[3 + i];
    }

    return true;
}
