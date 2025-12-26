/**
 * ModbusRTU.h - Biblioteca simplificada para Modbus RTU
 *
 * Suporta:
 * - FC01: Read Coils
 * - FC05: Write Single Coil
 *
 * Para uso com STM32F103C8 e MAX485
 */

#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#include <Arduino.h>

// Códigos de função Modbus
#define FC_READ_COILS          0x01
#define FC_WRITE_SINGLE_COIL   0x05

// Valores para coils
#define COIL_ON   0xFF00
#define COIL_OFF  0x0000

// Códigos de exceção Modbus
#define EX_ILLEGAL_FUNCTION    0x01
#define EX_ILLEGAL_ADDRESS     0x02
#define EX_ILLEGAL_VALUE       0x03

// Tamanho máximo do buffer
#define MODBUS_BUFFER_SIZE     64

// Timeout para recepção (ms)
#define MODBUS_TIMEOUT         100

/**
 * Classe para dispositivo Modbus RTU Escravo
 */
class ModbusSlave {
public:
    /**
     * Construtor
     * @param serial Referência para HardwareSerial (ex: Serial1)
     * @param dePin Pino para controle DE/RE do MAX485
     * @param slaveAddr Endereço do escravo (1-247)
     */
    ModbusSlave(HardwareSerial& serial, uint8_t dePin, uint8_t slaveAddr);

    /**
     * Inicializa a comunicação
     * @param baudRate Velocidade da serial (padrão 9600)
     */
    void begin(unsigned long baudRate = 9600);

    /**
     * Processa mensagens recebidas (chamar no loop)
     */
    void poll();

    /**
     * Define callback para leitura de coils
     * @param callback Função que retorna o estado do coil (0 ou 1)
     */
    void onReadCoil(bool (*callback)(uint16_t coilAddr));

    /**
     * Define callback para escrita de coils
     * @param callback Função chamada quando um coil é escrito
     */
    void onWriteCoil(void (*callback)(uint16_t coilAddr, bool value));

private:
    HardwareSerial& _serial;
    uint8_t _dePin;
    uint8_t _slaveAddr;
    uint8_t _buffer[MODBUS_BUFFER_SIZE];
    uint8_t _bufferLen;
    unsigned long _lastByteTime;

    bool (*_readCoilCallback)(uint16_t);
    void (*_writeCoilCallback)(uint16_t, bool);

    void processMessage();
    void sendResponse(uint8_t* data, uint8_t len);
    void sendException(uint8_t functionCode, uint8_t exceptionCode);
    void setTxMode(bool transmit);
};

/**
 * Classe para dispositivo Modbus RTU Mestre
 */
class ModbusMaster {
public:
    /**
     * Construtor
     * @param serial Referência para HardwareSerial (ex: Serial1)
     * @param dePin Pino para controle DE/RE do MAX485
     */
    ModbusMaster(HardwareSerial& serial, uint8_t dePin);

    /**
     * Inicializa a comunicação
     * @param baudRate Velocidade da serial (padrão 9600)
     */
    void begin(unsigned long baudRate = 9600);

    /**
     * Escreve um único coil
     * @param slaveAddr Endereço do escravo
     * @param coilAddr Endereço do coil (0-based)
     * @param value true=ON, false=OFF
     * @return true se sucesso
     */
    bool writeSingleCoil(uint8_t slaveAddr, uint16_t coilAddr, bool value);

    /**
     * Lê múltiplos coils
     * @param slaveAddr Endereço do escravo
     * @param startAddr Endereço inicial do coil
     * @param quantity Quantidade de coils a ler (1-16)
     * @param result Array para armazenar resultado (cada bit = 1 coil)
     * @return true se sucesso
     */
    bool readCoils(uint8_t slaveAddr, uint16_t startAddr, uint16_t quantity, uint8_t* result);

    /**
     * Retorna o último código de erro
     */
    uint8_t getLastError();

private:
    HardwareSerial& _serial;
    uint8_t _dePin;
    uint8_t _buffer[MODBUS_BUFFER_SIZE];
    uint8_t _lastError;

    void sendRequest(uint8_t* data, uint8_t len);
    int receiveResponse(uint8_t* buffer, uint8_t expectedLen, unsigned long timeout);
    void setTxMode(bool transmit);
};

/**
 * Calcula CRC-16 Modbus
 * @param data Ponteiro para os dados
 * @param len Tamanho dos dados
 * @return CRC-16 (LSB first)
 */
uint16_t modbusCRC16(uint8_t* data, uint8_t len);

#endif
