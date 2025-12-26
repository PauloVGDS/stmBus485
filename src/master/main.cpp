/**
 * Modbus RTU Mestre - STM32F103C8
 *
 * Envia comandos para escravo controlar LEDs via optoacoplador PC817
 * Comunicação RS485 usando MAX485
 *
 * Conexões:
 * - PA9  (TX)  -> MAX485 DI
 * - PA10 (RX)  -> MAX485 RO
 * - PA8  (DE)  -> MAX485 DE+RE (juntos)
 * - PC13 (LED) -> LED onboard (debug)
 *
 * Sequência de demonstração:
 * 1. Liga LED 1, aguarda 2s
 * 2. Liga LED 2, aguarda 2s
 * 3. Desliga LED 1, aguarda 2s
 * 4. Desliga LED 2, aguarda 2s
 * 5. Liga ambos, aguarda 2s
 * 6. Desliga ambos, aguarda 2s
 * 7. Repete
 */

#include <Arduino.h>
#include <ModbusRTU.h>

// Configuração de pinos
#define RS485_DE_PIN    PA8    // Controle DE/RE do MAX485
#define LED_ONBOARD     PC13   // LED da placa (ativo baixo)

// Configuração Modbus
#define SLAVE_ADDRESS   1      // Endereço do escravo
#define BAUD_RATE       9600

// Endereços dos coils no escravo
#define COIL_LED1       0
#define COIL_LED2       1

// Instância do mestre Modbus (usando Serial1 = UART1)
ModbusMaster modbus(Serial1, RS485_DE_PIN);

/**
 * Liga ou desliga um LED no escravo
 * Pisca LED onboard para indicar sucesso/erro
 */
bool setLed(uint16_t coil, bool state) {
    Serial.print("[MASTER] LED ");
    Serial.print(coil + 1);
    Serial.print(" -> ");
    Serial.print(state ? "ON" : "OFF");

    bool success = modbus.writeSingleCoil(SLAVE_ADDRESS, coil, state);

    if (success) {
        Serial.println(": OK");
        // Sucesso: pisca rápido
        digitalWrite(LED_ONBOARD, LOW);
        delay(50);
        digitalWrite(LED_ONBOARD, HIGH);
    } else {
        Serial.print(": ERRO (");
        Serial.print(modbus.getLastError(), HEX);
        Serial.println(")");
        // Erro: pisca lento 3 vezes
        for (int i = 0; i < 3; i++) {
            digitalWrite(LED_ONBOARD, LOW);
            delay(200);
            digitalWrite(LED_ONBOARD, HIGH);
            delay(200);
        }
    }

    return success;
}

/**
 * Lê o estado dos LEDs no escravo
 */
bool readLeds(bool* led1, bool* led2) {
    Serial.print("[MASTER] Lendo LEDs... ");
    uint8_t coilData[1];

    if (modbus.readCoils(SLAVE_ADDRESS, COIL_LED1, 2, coilData)) {
        *led1 = (coilData[0] & 0x01) != 0;
        *led2 = (coilData[0] & 0x02) != 0;
        Serial.print("LED1=");
        Serial.print(*led1 ? "ON" : "OFF");
        Serial.print(" LED2=");
        Serial.println(*led2 ? "ON" : "OFF");
        return true;
    }

    Serial.print("ERRO (");
    Serial.print(modbus.getLastError(), HEX);
    Serial.println(")");
    return false;
}

void setup() {
    // Configura LED onboard
    pinMode(LED_ONBOARD, OUTPUT);
    digitalWrite(LED_ONBOARD, HIGH);  // Desligado (ativo baixo)

    // Inicializa Serial USB para logs
    Serial.begin(115200);
    delay(100);
    Serial.println();
    Serial.println("[MASTER] ========================");
    Serial.println("[MASTER] Modbus RTU Mestre");
    Serial.println("[MASTER] ========================");

    // Inicializa comunicação Modbus
    modbus.begin(BAUD_RATE);
    Serial.println("[MASTER] Modbus iniciado (9600 bps)");

    // Aguarda escravo inicializar
    Serial.println("[MASTER] Aguardando escravo...");
    delay(1000);

    // Sinaliza que o mestre está pronto (5 piscadas rápidas)
    for (int i = 0; i < 5; i++) {
        digitalWrite(LED_ONBOARD, LOW);
        delay(50);
        digitalWrite(LED_ONBOARD, HIGH);
        delay(50);
    }

    // Garante que ambos LEDs comecem desligados
    Serial.println("[MASTER] Inicializando LEDs...");
    setLed(COIL_LED1, false);
    delay(100);
    setLed(COIL_LED2, false);
    delay(500);
    Serial.println("[MASTER] Pronto! Iniciando sequencia...");
    Serial.println();
}

void loop() {
    // Sequência de demonstração

    // 1. Liga LED 1
    setLed(COIL_LED1, true);
    delay(2000);

    // 2. Liga LED 2
    setLed(COIL_LED2, true);
    delay(2000);

    // 3. Desliga LED 1
    setLed(COIL_LED1, false);
    delay(2000);

    // 4. Desliga LED 2
    setLed(COIL_LED2, false);
    delay(2000);

    // 5. Liga ambos
    setLed(COIL_LED1, true);
    delay(100);
    setLed(COIL_LED2, true);
    delay(2000);

    // 6. Lê estado atual (verificação)
    bool led1State, led2State;
    if (readLeds(&led1State, &led2State)) {
        // Leitura bem sucedida - pisca confirmação
        digitalWrite(LED_ONBOARD, LOW);
        delay(100);
        digitalWrite(LED_ONBOARD, HIGH);
    }
    delay(1000);

    // 7. Desliga ambos
    setLed(COIL_LED1, false);
    delay(100);
    setLed(COIL_LED2, false);
    delay(2000);

    // Repete a sequência
}
