/*
 * ensaio_dimmer.ino
 * Ensaio sem controlador: Dimmer (ZC + DIM) + ACS712 + DS18B20
 * Plataforma: Arduino MEGA 2560
 *
 * Bibliotecas (Library Manager):
 *   OneWire            – "OneWire" by Paul Stoffregen
 *   DallasTemperature  – "DallasTemperature" by Miles Burton
 *
 * Ligações:
 *   ZC_PIN      = 3   → sinal de cruzamento por zero (INT1)
 *   DIM_PIN     = 10  → gatilho do TRIAC (MOC3021 + BT137)
 *   ACS_PIN     = A8  → ACS712 (saída analógica)
 *   DS18B20_PIN = 26  → DS18B20 data  + 4,7 kΩ pull-up para 5 V
 *
 * ── Garantias de timing ────────────────────────────────────────────────────
 *
 *  1. ESCRITA ATÔMICA DE 16 BITS
 *     ATmega2560 é 8 bits: uma escrita uint16_t são DOIS stores separados.
 *     Se o ZC ISR disparar entre os dois stores, lê um valor corrompido.
 *     Solução: noInterrupts()/interrupts() ao atualizar g_fireTicks no loop.
 *
 *  2. RESET FORÇADO DO PINO DIM NO ZC
 *     Se o ZC chegar antes do COMPB terminar o pulso anterior,
 *     o pino DIM ficaria HIGH para sempre.
 *     Solução: digitalWrite(DIM_PIN, LOW) SEMPRE no início do ZC ISR.
 *
 *  3. ATRASO MÍNIMO (evita OCR1A = 0)
 *     Com OCR1A = 0, o Timer1 dá match imediato antes de estabilizar.
 *     Solução: delay mínimo de MIN_DELAY_US (≈ 1° elétrico ≈ potência máx real).
 *
 *  4. CAPTURA LOCAL NO ZC ISR
 *     A ISR captura g_fireTicks em variável local antes de qualquer operação,
 *     assim eventuais atualizações feitas pelo loop principal só valem no
 *     próximo semiciclo — comportamento correto e determinístico.
 *
 * Protocolo Serial (115200 baud, '\n'):
 *   Recebe:  SET:<pct>    → potência desejada (0.0–100.0 %); Arduino converte via NR
 *            START | STOP | CAL
 *            SENS:<mV/A>  → ajusta sensibilidade ACS712 sem recompilar (ex: SENS:212.75)
 *   Envia:   DATA:<millis>,<delay_us>,<irms_A>,<temp_C>,<urms_V>,<prms_W>
 *            ACK:<delay_us> | CAL_DONE:<offset> | READY | WARN:...
 */

#include <Arduino.h>
#include <math.h>       // libm: sinf, cosf, fabsf (float — mais rápido no AVR)
#include <OneWire.h>
#include <DallasTemperature.h>

// ─── Pinos ────────────────────────────────────────────────────────────────
#define ZC_PIN       3
#define DIM_PIN      10
#define ACS_PIN      A8
#define DS18B20_PIN  26

// ─── ACS712 ───────────────────────────────────────────────────────────────
// Sensibilidade: 5A→185.0  |  20A→100.0  |  30A→66.0  (mV/A)
// Ajustável em runtime via comando SENS:<valor> (sem recompilar)
#define ACS_SENS_mVpA   185.0f  // ACS712-5A datasheet: 185 mV/A
#define ADC_VREF_mV    5000.0f
#define ADC_BITS        1024
#define N_SAMPLES        480    // 480 × 104µs ≈ 49.9ms = 3 ciclos de 60Hz (reduz ruído por √3)

// ─── Dimmer ───────────────────────────────────────────────────────────────
// Rede elétrica: 50 Hz → 10000 µs  |  60 Hz → 8333 µs
#define HALF_CYCLE_US    8333UL
#define PULSE_US           150UL   // largura do pulso de gatilho no TRIAC
#define SAFETY_MARGIN_US   400UL   // guarda no fim do semiciclo (não dispara)
// Atraso mínimo: evita OCR1A=0 e garante que o ZC ISR já encerrou
// 500 µs ≈ 9° elétricos ≈ potência máxima efetiva ≈ 99,9 %
#define MIN_DELAY_US       500UL

// Timer1, prescaler 8, 16 MHz → 0,5 µs / tick
#define TICKS_PER_US  2UL

// ─── DS18B20 ─────────────────────────────────────────────────────────────
OneWire           oneWire(DS18B20_PIN);
DallasTemperature sensors(&oneWire);
float             g_temperature     = -127.0f;
unsigned long     g_lastTempRequest = 0;
// 11 bits → 0,125 °C, conversão ~375 ms
#define TEMP_RESOLUTION          11
#define TEMP_CONV_MS            400UL
#define TEMP_REQUEST_INTERVAL   1000UL

// ─── Estado global ────────────────────────────────────────────────────────
// IMPORTANTE: leitura/escrita de g_fireTicks fora do ISR deve usar
// noInterrupts()/interrupts() (ver Garantia #1).
volatile uint16_t g_fireTicks =
    (uint16_t)((HALF_CYCLE_US - SAFETY_MARGIN_US) * TICKS_PER_US); // off

// Monitor de ZC: ISR incrementa; loop lê e zera a cada 1 s.
// Espera-se 100 semiciclos/s em 50 Hz. Fora de [85, 115] → WARN.
volatile uint16_t g_zcCount    = 0;
unsigned long     g_lastZCCheck = 0;

// Último número de amostras ADC válidas (sem spike) — setado em measureRMS().
// Loop emite WARN:ADC_SPIKE se < 50 % das amostras foram aceitas.
uint16_t      g_lastValidSamples = 0;

uint16_t      g_acsOffset = 512;
float         g_acsSens  = ACS_SENS_mVpA;  // ajustável via SENS:<mV/A>
bool          g_sending   = false;
unsigned long g_lastSend  = 0;
#define       SEND_MS  50

// ─── Buffer serial estático (evita heap fragmentation do String) ──────────
#define CMD_BUF_LEN 32
char          g_cmdBuf[CMD_BUF_LEN];
uint8_t       g_cmdIdx = 0;

// Retorna true quando uma linha completa ('\n') foi acumulada em g_cmdBuf.
bool serialReadLine() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (g_cmdIdx > 0) {
                g_cmdBuf[g_cmdIdx] = '\0';
                g_cmdIdx = 0;
                return true;
            }
        } else if (g_cmdIdx < CMD_BUF_LEN - 1) {
            g_cmdBuf[g_cmdIdx++] = c;
        }
    }
    return false;
}

// ─── Conversão potência (%) → ângulo de disparo (rad) ────────────────────
// Inverte P(α) = (π − α + sin(2α)/2) / π via Newton-Raphson.
// Ref: Michel Loureiro, jun/2026.
//
// f(α)  = (π − α + sin(2α)/2) / π − P_ref
// f'(α) = (cos(2α) − 1) / π
// α₀   = π(1 − P_ref)
//
// Passo limitado a ±0.3 rad evita divergência onde f'→0 (extremos 0% e 100%).
// Converge em 4–6 iterações para 10–90 %; até 20 nos extremos.

#define ALPHA_MIN  0.18850f   // 500us  / 8333us × π  (potência máxima)
#define ALPHA_MAX  2.99252f   // 7933us / 8333us × π  (potência mínima)

float potencia_para_alpha(float pct) {
    float p = constrain(pct / 100.0f, 0.0f, 1.0f);
    float a = constrain(PI * (1.0f - p), ALPHA_MIN, ALPHA_MAX);
    for (uint8_t i = 0; i < 20; i++) {
        float f  = (PI - a + sinf(2.0f * a) / 2.0f) / PI - p;
        float df = (cosf(2.0f * a) - 1.0f) / PI;
        if (fabsf(df) < 1e-10f) break;
        float da = constrain(f / df, -0.3f, 0.3f);
        a = constrain(a - da, ALPHA_MIN, ALPHA_MAX);
        if (fabsf(da) < 1e-6f) break;
    }
    return a;
}

uint32_t alpha_para_delay_us(float alpha_rad) {
    uint32_t d = (uint32_t)(alpha_rad / PI * (float)HALF_CYCLE_US);
    return constrain(d, MIN_DELAY_US, HALF_CYCLE_US - SAFETY_MARGIN_US);
}

// ─── Medição RMS ─────────────────────────────────────────────────────────
// Spike do TRIAC: disparo capacitivo pode empurrar ADC para saturação.
// Rejeita amostras com desvio > 900 LSB (≈ 4.4 A em 185 mV/A, 5 V ref).
#define ACS_SPIKE_LIMIT  900

float measureRMS() {
    double   sumSq = 0.0;
    uint16_t valid = 0;
    for (uint16_t i = 0; i < N_SAMPLES; i++) {
        int raw = analogRead(ACS_PIN);
        int dev = raw - (int)g_acsOffset;
        if (dev > ACS_SPIKE_LIMIT || dev < -ACS_SPIKE_LIMIT) continue;
        float V = (float)dev * (ADC_VREF_mV / (float)ADC_BITS);
        float I = V / g_acsSens;
        sumSq  += (double)I * I;
        valid++;
    }
    g_lastValidSamples = valid;
    if (valid < 10) return 0.0f;
    return (float)sqrt(sumSq / valid);
}

uint16_t calibrateOffset() {
    uint32_t s = 0;
    for (uint16_t i = 0; i < 1024; i++) s += analogRead(ACS_PIN);
    return (uint16_t)(s >> 10);
}

// ─── ZC ISR ──────────────────────────────────────────────────────────────
void zeroCrossISR() {
    // Debounce: o módulo MC-8A pode oscilar na borda e gerar 2-3 pulsos por
    // cruzamento. Ignoramos qualquer ZC que chegue menos de 5 ms após o
    // anterior (semiciclo de 50 Hz = 10 ms, então 5 ms é margem segura).
    static unsigned long lastZC = 0;
    unsigned long now = micros();
    if (now - lastZC < 4000UL) return;
    lastZC = now;
    g_zcCount++;   // monitor de taxa de ZC (lido e zerado no loop a cada 1 s)

    // GARANTIA #2: força DIM=LOW SEMPRE, independente do estado anterior.
    // Caso o COMPB do ciclo anterior não tenha executado a tempo, este reset
    // impede que o TRIAC fique disparado continuamente.
    digitalWrite(DIM_PIN, LOW);

    TCCR1B = 0;   // para timer (elimina qualquer disparo pendente)
    TCCR1A = 0;   // modo normal

    // GARANTIA #4: captura local — mudanças em g_fireTicks só valem no
    // próximo semiciclo, comportamento correto e sem race condition visível.
    uint16_t ft = g_fireTicks;

    TCNT1 = 0;
    OCR1A = ft;
    OCR1B = ft + (uint16_t)(PULSE_US * TICKS_PER_US);

    TIFR1  = (1 << OCF1A) | (1 << OCF1B);          // limpa flags pendentes
    TIMSK1 = (1 << OCIE1A) | (1 << OCIE1B);
    TCCR1B = (1 << CS11);                           // normal mode, prescaler 8
}

// ─── ISR A: instante de disparo ───────────────────────────────────────────
ISR(TIMER1_COMPA_vect) {
    // Só dispara se estiver dentro da janela válida
    if (OCR1A < (uint16_t)((HALF_CYCLE_US - SAFETY_MARGIN_US) * TICKS_PER_US)) {
        digitalWrite(DIM_PIN, HIGH);
    }
    TIMSK1 &= ~(1 << OCIE1A);
}

// ─── ISR B: fim do pulso ─────────────────────────────────────────────────
ISR(TIMER1_COMPB_vect) {
    digitalWrite(DIM_PIN, LOW);
    TIMSK1 = 0;
    TCCR1B = 0;
}

// ─── setup ────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    pinMode(ZC_PIN,  INPUT);
    pinMode(DIM_PIN, OUTPUT);
    digitalWrite(DIM_PIN, LOW);

    TCCR1A = 0; TCCR1B = 0; TIMSK1 = 0;
    attachInterrupt(digitalPinToInterrupt(ZC_PIN), zeroCrossISR, RISING);

    sensors.begin();
    if (sensors.getDeviceCount() == 0)
        Serial.println(F("WARN:DS18B20_NAO_ENCONTRADO"));
    sensors.setResolution(TEMP_RESOLUTION);
    sensors.setWaitForConversion(false);
    sensors.requestTemperatures();
    g_lastTempRequest = millis();

    Serial.println(F("READY"));
}

// ─── loop ─────────────────────────────────────────────────────────────────
void loop() {

    // ── DS18B20 assíncrono ──
    unsigned long now = millis();
    if (now - g_lastTempRequest >= TEMP_CONV_MS) {
        float t = sensors.getTempCByIndex(0);
        if (t > -100.0f) g_temperature = t;
        if (now - g_lastTempRequest >= TEMP_REQUEST_INTERVAL) {
            sensors.requestTemperatures();
            g_lastTempRequest = now;
        }
    }

    // ── Monitor de integridade (1 Hz) ──
    // Verifica taxa de ZC e qualidade do ADC. Emite WARN via serial se algo
    // estiver fora do esperado — detectável pelo Python mesmo sem parar o ensaio.
    if (millis() - g_lastZCCheck >= 1000UL) {
        g_lastZCCheck = millis();

        uint16_t cnt;
        noInterrupts(); cnt = g_zcCount; g_zcCount = 0; interrupts();

        // 50 Hz → 100 semiciclos/s. Aceita ±15 % de jitter.
        // 60 Hz → 120 semiciclos/s esperados. Aceita ±15%.
        if (cnt < 100 || cnt > 140) {
            Serial.print(F("WARN:ZC_RATE=")); Serial.println(cnt);
        }

        // ADC: se mais de 50 % das amostras foram rejeitadas, há spikes graves.
        if (g_sending && g_lastValidSamples < N_SAMPLES / 2) {
            Serial.print(F("WARN:ADC_SPIKE_PCT="));
            Serial.println(100U - (uint32_t)g_lastValidSamples * 100U / N_SAMPLES);
        }
    }

    // ── Comandos seriais (buffer estático — sem heap fragmentation) ──
    if (serialReadLine()) {
        if (strncmp(g_cmdBuf, "SET:", 4) == 0) {
            float pct = atof(g_cmdBuf + 4);
            uint32_t val = alpha_para_delay_us(potencia_para_alpha(pct));
            uint16_t newTicks = (uint16_t)(val * TICKS_PER_US);

            // GARANTIA #1: escrita atômica de 16 bits
            noInterrupts();
            g_fireTicks = newTicks;
            interrupts();

            Serial.print(F("ACK:")); Serial.println(val);

        } else if (strcmp(g_cmdBuf, "START") == 0) {
            g_sending  = true;
            g_lastSend = millis();
            Serial.println(F("TEST_STARTED"));

        } else if (strcmp(g_cmdBuf, "STOP") == 0) {
            g_sending = false;
            uint32_t val = alpha_para_delay_us(potencia_para_alpha(0.0f));
            noInterrupts();
            g_fireTicks = (uint16_t)(val * TICKS_PER_US);
            interrupts();
            Serial.println(F("TEST_STOPPED"));

        } else if (strcmp(g_cmdBuf, "CAL") == 0) {
            g_acsOffset = calibrateOffset();
            Serial.print(F("CAL_DONE:")); Serial.println(g_acsOffset);

        } else if (strncmp(g_cmdBuf, "SENS:", 5) == 0) {
            float s = atof(g_cmdBuf + 5);
            if (s > 10.0f && s < 500.0f) {
                g_acsSens = s;
                Serial.print(F("SENS_OK:")); Serial.println(g_acsSens, 2);
            } else {
                Serial.println(F("SENS_ERR:fora de faixa (10..500 mV/A)"));
            }
        }
    }

    // ── Envio de dados ──
    if (g_sending && (millis() - g_lastSend >= SEND_MS)) {
        g_lastSend = millis();

        float    irms = measureRMS();
        uint32_t delayUs;

        noInterrupts();
        delayUs = (uint32_t)(g_fireTicks / TICKS_PER_US);
        interrupts();

        // Calcula U_rms e P_rms a partir do ângulo de disparo
        // α = delay_us / HALF_CYCLE_US × π
        // U_rms(α) = 127 × √( (π−α + sin(2α)/2) / π )
        // P_rms = U_rms × I_rms
        float alpha = (float)delayUs / (float)HALF_CYCLE_US * PI;
        float pNorm = (PI - alpha + sinf(2.0f * alpha) / 2.0f) / PI;
        float urms  = 127.0f * sqrt(max(pNorm, 0.0f));
        float prms  = urms * irms;

        // DATA:<millis>,<delay_us>,<irms_A>,<temp_C>,<urms_V>,<prms_W>
        Serial.print(F("DATA:"));
        Serial.print(millis());        Serial.print(',');
        Serial.print(delayUs);         Serial.print(',');
        Serial.print(irms,   4);       Serial.print(',');
        Serial.print(g_temperature, 2); Serial.print(',');
        Serial.print(urms,   2);       Serial.print(',');
        Serial.println(prms, 3);
    }
}
