/**
 * Modbus RTU Escravo - STM32F103C8
 *
 * Controla 2 LEDs via optoacoplador PC817
 * Comunicação RS485 usando MAX485
 *
 * Conexões:
 * - PA9  (TX)  -> MAX485 DI
 * - PA10 (RX)  -> MAX485 RO
 * - PA8  (DE)  -> MAX485 DE+RE (juntos)
 * - PB0  (OUT) -> PC817 Canal 1 (LED 1)
 * - PB1  (OUT) -> PC817 Canal 2 (LED 2)
 * - PC13 (LED) -> LED onboard (debug)
 */

#include <Arduino.h>
#include <ModbusRTU.h>

// Configuração de pinos
#define RS485_DE_PIN    PA8    // Controle DE/RE do MAX485
#define LED1_PIN        PB0    // Optoacoplador Canal 1
#define LED2_PIN        PB1    // Optoacoplador Canal 2
#define LED_ONBOARD     PC13   // LED da placa (ativo baixo)

// Configuração Modbus
#define SLAVE_ADDRESS   1      // Endereço do escravo
#define BAUD_RATE       9600

// Mapeamento de coils
#define COIL_LED1       0      // Coil 0 = LED 1
#define COIL_LED2       1      // Coil 1 = LED 2
#define NUM_COILS       2

// Estado dos coils (LEDs)
bool coilState[NUM_COILS] = {false, false};

// Instância do escravo Modbus (usando Serial1 = UART1)
ModbusSlave modbus(Serial1, RS485_DE_PIN, SLAVE_ADDRESS);

/**
 * Callback para leitura de coils
 * Retorna o estado atual do LED solicitado
 */
bool readCoilCallback(uint16_t coilAddr) {
    if (coilAddr < NUM_COILS) {
        Serial.print("[SLAVE] Leitura: Coil ");
        Serial.print(coilAddr);
        Serial.print(" = ");
        Serial.println(coilState[coilAddr] ? "ON" : "OFF");
        return coilState[coilAddr];
    }
    return false;
}

/**
 * Callback para escrita de coils
 * Atualiza o estado do LED e aciona a saída
 */
void writeCoilCallback(uint16_t coilAddr, bool value) {
    if (coilAddr < NUM_COILS) {
        coilState[coilAddr] = value;

        Serial.print("[SLAVE] Escrita: Coil ");
        Serial.print(coilAddr);
        Serial.print(" = ");
        Serial.println(value ? "ON" : "OFF");

        // Atualiza saída física
        switch (coilAddr) {
            case COIL_LED1:
                digitalWrite(LED1_PIN, value ? HIGH : LOW);
                break;
            case COIL_LED2:
                digitalWrite(LED2_PIN, value ? HIGH : LOW);
                break;
        }

        // Pisca LED onboard para indicar atividade
        digitalWrite(LED_ONBOARD, LOW);   // Liga (ativo baixo)
        delay(50);
        digitalWrite(LED_ONBOARD, HIGH);  // Desliga
    }
}

void setup() {
    // Configura pinos de saída para os LEDs
    pinMode(LED1_PIN, OUTPUT);
    pinMode(LED2_PIN, OUTPUT);
    pinMode(LED_ONBOARD, OUTPUT);

    // Estado inicial: LEDs desligados
    digitalWrite(LED1_PIN, LOW);
    digitalWrite(LED2_PIN, LOW);
    digitalWrite(LED_ONBOARD, HIGH);  // LED onboard desligado (ativo baixo)

    // Inicializa Serial USB para logs
    Serial.begin(115200);
    delay(100);
    Serial.println();
    Serial.println("[SLAVE] ========================");
    Serial.println("[SLAVE] Modbus RTU Escravo");
    Serial.print("[SLAVE] Endereco: ");
    Serial.println(SLAVE_ADDRESS);
    Serial.println("[SLAVE] ========================");

    // Inicializa comunicação Modbus
    modbus.begin(BAUD_RATE);
    Serial.println("[SLAVE] Modbus iniciado (9600 bps)");

    // Registra callbacks
    modbus.onReadCoil(readCoilCallback);
    modbus.onWriteCoil(writeCoilCallback);

    // Sinaliza que o escravo está pronto (3 piscadas)
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_ONBOARD, LOW);
        delay(100);
        digitalWrite(LED_ONBOARD, HIGH);
        delay(100);
    }

    Serial.println("[SLAVE] Pronto! Aguardando comandos...");
    Serial.println();
}

void loop() {
    // Processa mensagens Modbus
    modbus.poll();
}
