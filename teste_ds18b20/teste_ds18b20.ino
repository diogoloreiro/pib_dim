  /*
 * teste_ds18b20.ino
 * Teste simples do sensor DS18B20 com Arduino UNO
 *
 * Bibliotecas necessárias (instalar pelo Library Manager):
 *   OneWire           – "OneWire" by Paul Stoffregen
 *   DallasTemperature – "DallasTemperature" by Miles Burton
 *
 * Ligação:
 *
 *   DS18B20 (flat side facing you)
 *   ┌─────────────┐
 *   │ GND  DATA  VCC │
 *   └──┬────┬────┬──┘
 *      │    │    │
 *     GND  D2   5V
 *           │
 *          4.7kΩ (pull-up entre DATA e 5V)
 *
 * Serial Monitor: 9600 baud
 */

#include <OneWire.h>
#include <DallasTemperature.h>

#define DS18B20_PIN  8 // pino de dados do sensor

OneWire           oneWire(DS18B20_PIN);
DallasTemperature sensors(&oneWire);

void setup() {
    Serial.begin(9600);
    Serial.println(F("=== Teste DS18B20 ==="));

    sensors.begin();

    // Verifica se encontrou algum sensor no barramento
    int total = sensors.getDeviceCount();
    Serial.print(F("Sensores encontrados: "));
    Serial.println(total);

    if (total == 0) {
        Serial.println(F("ERRO: nenhum sensor encontrado."));
        Serial.println(F("Verifique:"));
        Serial.println(F("  1. Resistor de 4.7k entre DATA e 5V"));
        Serial.println(F("  2. Cabo no pino correto (D2)"));
        Serial.println(F("  3. VCC no 5V, GND no GND"));
        while (true);   // trava aqui — não tem o que fazer sem sensor
    }

    // Imprime o endereço do primeiro sensor (útil se tiver mais de um)
    DeviceAddress addr;
    if (sensors.getAddress(addr, 0)) {
        Serial.print(F("Endereço do sensor 0: "));
        for (uint8_t i = 0; i < 8; i++) {
            if (addr[i] < 16) Serial.print('0');
            Serial.print(addr[i], HEX);
            if (i < 7) Serial.print(':');
        }
        Serial.println();
    }

    // Resolução: 9=0.5°C/94ms | 10=0.25°C/188ms | 11=0.125°C/375ms | 12=0.0625°C/750ms
    sensors.setResolution(11);
    Serial.println(F("Resolução: 11 bits (0.125 °C)"));
    Serial.println(F("----------------------------"));
}

void loop() {
    // Pede a conversão e aguarda (modo bloqueante — simples para teste)
    sensors.requestTemperatures();

    float tempC = sensors.getTempCByIndex(0);

    if (tempC == DEVICE_DISCONNECTED_C) {
        Serial.println(F("ERRO: sensor desconectado durante leitura."));
    } else {
        Serial.print(F("Temperatura: "));
        Serial.print(tempC, 3);          // 3 casas — ex: 25.125
        Serial.print(F(" °C  |  "));
        Serial.print(tempC * 9.0 / 5.0 + 32.0, 1);
        Serial.println(F(" °F"));
    }

    delay(1000);   // lê a cada 1 segundo
}
