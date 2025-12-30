# Modbus RTU Master/Slave com Optoacopladores

Projeto de comunicação Modbus RTU entre dois microcontroladores STM32F103C8, utilizando RS485 (MAX485) para controle de LEDs através de optoacopladores PC817.

## Hardware

- **MCU**: STM32F103C8 (Blue Pill)
- **Comunicação**: RS485 via MAX485
- **Isolamento**: Optoacopladores PC817 (2 canais)
- **Baud Rate**: 9600 bps

### Pinagem

| Pino | Função |
|------|--------|
| PA9  | TX → MAX485 DI |
| PA10 | RX → MAX485 RO |
| PA8  | MAX485 DE/RE |
| PC13 | LED onboard |
| PB0  | LED 1 (slave) |
| PB1  | LED 2 (slave) |

## Estrutura do Projeto

```
├── src/
│   ├── master/main.cpp    # Firmware do mestre
│   └── slave/main.cpp     # Firmware do escravo
├── lib/
│   └── ModbusRTU/         # Biblioteca Modbus RTU customizada
└── platformio.ini
```

## Funcionalidades

- **Master**: Envia comandos para controlar 2 LEDs e lê status
- **Slave**: Recebe comandos Modbus e aciona optoacopladores
- **Funções Modbus**: FC01 (Read Coils), FC05 (Write Single Coil)

## Compilação

```bash
# Compilar firmware do Master
pio run -e master

# Compilar firmware do Slave
pio run -e slave
```

## Licença

MIT
