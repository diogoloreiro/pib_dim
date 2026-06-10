/*
 * controlador_pid.ino
 * Controlador PID/PI de temperatura — coeficientes via serial
 * Plataforma: Arduino MEGA 2560
 *
 * Pinos:
 *   ZC_PIN      = 3   → ZC do MC-8A (INT1)
 *   DIM_PIN     = 10  → Gatilho TRIAC
 *   ACS_PIN     = A8  → ACS712-5A (monitoramento)
 *   DS18B20_PIN = 26  → Sensor de temperatura
 *
 * Protocolo serial (115200 baud):
 *   KP:<val>   → ganho proporcional
 *   KI:<val>   → ganho integral  (Ki = Kp/Ti)
 *   KD:<val>   → ganho derivativo (Kd = Kp*Td)  — use 0 para PI
 *   SP:<val>   → setpoint de temperatura (°C)
 *   START      → inicia controle
 *   STOP       → para controle
 *   STATUS     → mostra parâmetros atuais
 *   CAL        → calibra ACS712 (sem carga)
 *
 * Saída (20 Hz enquanto controle ativo):
 *   DATA:<ms>,<T>,<SP>,<erro>,<saida_%>,<irms_A>
 */

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ─── Pinos ────────────────────────────────────────────────────────────────
#define ZC_PIN       3
#define DIM_PIN      10
#define ACS_PIN      A8
#define DS18B20_PIN  26

// ─── Dimmer ───────────────────────────────────────────────────────────────
#define HALF_CYCLE_US   8333UL
#define PULSE_US         150UL
#define SAFETY_US        400UL
#define MIN_DELAY_US     500UL
#define TICKS_PER_US       2UL

// ─── ACS712 ───────────────────────────────────────────────────────────────
#define ACS_SENS_mVpA  185.0f
#define N_SAMPLES        480    // 3 ciclos de 60 Hz

// ─── Limites de saída do controlador ────────────────────────────────────
#define U_MIN    15.0f   // % mínimo (TRIAC)
#define U_MAX   100.0f   // % máximo
// Coeficiente do filtro derivativo.
// N=10, Ts=1s → τ_filtro ≈ 0.1 s·rad; aumentar N = menos filtragem (mais ruído).
#define DERIV_N  10.0f

// ─── DS18B20 ─────────────────────────────────────────────────────────────
OneWire           oneWire(DS18B20_PIN);
DallasTemperature sensors(&oneWire);
float             g_temp        = -127.0f;
unsigned long     g_lastTempReq = 0;
#define TEMP_CONV_MS    400UL
#define TEMP_REQ_MS    1000UL

// ─── Estado do dimmer ─────────────────────────────────────────────────────
volatile uint16_t g_fireTicks =
    (uint16_t)((HALF_CYCLE_US - SAFETY_US) * TICKS_PER_US);

volatile uint16_t g_zcCount = 0;
uint16_t g_acsOffset = 512;

// ─── PID ──────────────────────────────────────────────────────────────────
// ATENÇÃO: Ki = Kp/Ti  e  Kd = Kp*Td  são válidos para Ts = 1 s (PID_MS=1000).
// Se PID_MS mudar, reescale: Ki_novo = Ki * (Ts_novo/1)  e  Kd_novo = Kd * (1/Ts_novo).
float g_Kp = 0.0f;
float g_Ki = 0.0f;
float g_Kd = 0.0f;
float g_SP = 30.0f;  // setpoint inicial (°C)

float g_integral = 0.0f;
float g_erro_ant = 0.0f;
float g_D_filt   = 0.0f;     // estado do filtro derivativo
float g_T_ant    = -127.0f;  // T medida no passo anterior (inicia inválida)
float g_saida    = 0.0f;     // saída do controlador (%)

bool  g_controlando = false;
unsigned long g_lastPID   = 0;
unsigned long g_lastSend  = 0;
#define PID_MS   1000UL   // período do PID (1 Hz = período DS18B20)
#define SEND_MS    50UL   // período de envio de dados (20 Hz)

// ─── Buffer serial ────────────────────────────────────────────────────────
#define CMD_BUF 32
char    g_cmdBuf[CMD_BUF];
uint8_t g_cmdIdx = 0;

bool serialReadLine() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (g_cmdIdx > 0) { g_cmdBuf[g_cmdIdx] = '\0'; g_cmdIdx = 0; return true; }
        } else if (g_cmdIdx < CMD_BUF - 1) {
            g_cmdBuf[g_cmdIdx++] = c;
        }
    }
    return false;
}

// ─── ZC ISR ───────────────────────────────────────────────────────────────
void zeroCrossISR() {
    static unsigned long lastZC = 0;
    unsigned long now = micros();
    if (now - lastZC < 4000UL) return;
    lastZC = now;
    g_zcCount++;

    digitalWrite(DIM_PIN, LOW);
    TCCR1B = 0; TCCR1A = 0;
    uint16_t ft = g_fireTicks;
    TCNT1 = 0;
    OCR1A = ft;
    OCR1B = ft + (uint16_t)(PULSE_US * TICKS_PER_US);
    TIFR1  = (1 << OCF1A) | (1 << OCF1B);
    TIMSK1 = (1 << OCIE1A) | (1 << OCIE1B);
    TCCR1B = (1 << CS11);
}

ISR(TIMER1_COMPA_vect) {
    if (OCR1A < (uint16_t)((HALF_CYCLE_US - SAFETY_US) * TICKS_PER_US))
        digitalWrite(DIM_PIN, HIGH);
    TIMSK1 &= ~(1 << OCIE1A);
}

ISR(TIMER1_COMPB_vect) {
    digitalWrite(DIM_PIN, LOW);
    TIMSK1 = 0; TCCR1B = 0;
}

// ─── Funções auxiliares ───────────────────────────────────────────────────
void setPct(float pct) {
    pct = constrain(pct, 0.0f, 100.0f);
    // LUT do dimmer: inverte P(α) = (π−α + sin(2α)/2)/π
    float p  = pct / 100.0f;
    float lo = 0.0f, hi = PI;
    for (int i = 0; i < 40; i++) {
        float mid = (lo + hi) / 2.0f;
        float pm  = (PI - mid + sinf(2.0f * mid) / 2.0f) / PI;
        if (pm > p) lo = mid; else hi = mid;
    }
    uint32_t d = (uint32_t)((lo + hi) / 2.0f / PI * HALF_CYCLE_US);
    d = constrain(d, MIN_DELAY_US, HALF_CYCLE_US - SAFETY_US);
    noInterrupts();
    g_fireTicks = (uint16_t)(d * TICKS_PER_US);
    interrupts();
}

float medirRMS() {
    double sum = 0.0; int valid = 0;
    for (int i = 0; i < N_SAMPLES; i++) {
        int dev = analogRead(ACS_PIN) - (int)g_acsOffset;
        if (abs(dev) > 900) continue;
        float I = dev * (5000.0f / 1024.0f) / ACS_SENS_mVpA;
        sum += (double)I * I; valid++;
    }
    return (valid < 10) ? 0.0f : (float)sqrt(sum / valid);
}

// ─── PID ──────────────────────────────────────────────────────────────────
// Forma posicional com três melhorias em relação à versão simples:
//
//  1. DERIVATIVO SOBRE A MEDIÇÃO (não sobre o erro):
//     D = -Kd * ΔT  em vez de  D = Kd * Δe
//     Evita o "derivative kick": quando SP muda abruptamente, Δe salta,
//     causando um pico enorme na saída. ΔT muda suavemente pelo sistema.
//
//  2. FILTRO PASSA-BAIXA NO DERIVATIVO (N = DERIV_N):
//     D(k) = (1−β)·D(k−1) − β·Kd·ΔT,   β = 1/(1+N)
//     O DS18B20 tem quantização de ±0,5 °C (11 bits); sem filtro esse ruído
//     é amplificado diretamente pelo termo derivativo. Com N=10 e Ts=1s,
//     a constante de tempo do filtro é ~10s, suficiente para o sistema térmico.
//     Para desativar o filtro: defina DERIV_N = 0 (acima).
//
//  3. COMPARAÇÃO FLOAT CORRETA NO ANTI-WINDUP:
//     fabsf(saida − saida_sat) > 1e-4f  em vez de  saida != saida_sat
float calcPID(float T_atual) {
    float erro = g_SP - T_atual;

    // Proporcional
    float P = g_Kp * erro;

    // Integral
    g_integral += g_Ki * erro;

    // Derivativo sobre medição com filtro passa-baixa
    // Só ativa quando Kd > 0 e há leitura anterior válida
    float D = 0.0f;
    if (g_Kd > 0.0f && g_T_ant > -100.0f) {
        const float beta = 1.0f / (1.0f + DERIV_N);
        g_D_filt = (1.0f - beta) * g_D_filt
                   - beta * g_Kd * (T_atual - g_T_ant);
        D = g_D_filt;
    }

    float saida     = P + g_integral + D;
    float saida_sat = constrain(saida, U_MIN, U_MAX);

    // Anti-windup: retira do integral o excesso saturado
    if (fabsf(saida - saida_sat) > 1e-4f) {
        g_integral -= (saida - saida_sat);
    }

    g_erro_ant = erro;
    g_T_ant    = T_atual;
    g_saida    = saida_sat;
    return saida_sat;
}

void resetPID() {
    g_integral = 0.0f;
    g_erro_ant = 0.0f;
    g_D_filt   = 0.0f;
    g_T_ant    = -127.0f;
    g_saida    = U_MIN;
}

// ─── Setup ────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    pinMode(DIM_PIN, OUTPUT);
    digitalWrite(DIM_PIN, LOW);
    pinMode(ZC_PIN, INPUT);
    TCCR1A = 0; TCCR1B = 0; TIMSK1 = 0;
    attachInterrupt(digitalPinToInterrupt(ZC_PIN), zeroCrossISR, RISING);

    sensors.begin();
    if (sensors.getDeviceCount() == 0)
        Serial.println(F("WARN:DS18B20_NAO_ENCONTRADO"));
    sensors.setResolution(11);
    sensors.setWaitForConversion(false);
    sensors.requestTemperatures();
    g_lastTempReq = millis();

    Serial.println(F("READY"));
    Serial.println(F("Comandos: KP: KI: KD: SP: START STOP STATUS CAL"));
}

// ─── Loop ─────────────────────────────────────────────────────────────────
void loop() {
    unsigned long now = millis();

    // ── DS18B20 assíncrono ──
    if (now - g_lastTempReq >= TEMP_CONV_MS) {
        float t = sensors.getTempCByIndex(0);
        if (t > -100.0f) g_temp = t;
        if (now - g_lastTempReq >= TEMP_REQ_MS) {
            sensors.requestTemperatures();
            g_lastTempReq = now;
        }
    }

    // ── PID (1 Hz, sincronizado com DS18B20) ──
    if (g_controlando && (now - g_lastPID >= PID_MS)) {
        g_lastPID = now;
        if (g_temp > -100.0f) {
            float u = calcPID(g_temp);
            setPct(u);
        }
    }

    // ── Comandos seriais ──
    if (serialReadLine()) {
        if (strncmp(g_cmdBuf, "KP:", 3) == 0) {
            g_Kp = atof(g_cmdBuf + 3);
            Serial.print(F("KP=")); Serial.println(g_Kp, 6);

        } else if (strncmp(g_cmdBuf, "KI:", 3) == 0) {
            g_Ki = atof(g_cmdBuf + 3);
            Serial.print(F("KI=")); Serial.println(g_Ki, 6);

        } else if (strncmp(g_cmdBuf, "KD:", 3) == 0) {
            g_Kd = atof(g_cmdBuf + 3);
            Serial.print(F("KD=")); Serial.println(g_Kd, 6);

        } else if (strncmp(g_cmdBuf, "SP:", 3) == 0) {
            float sp = atof(g_cmdBuf + 3);
            if (sp >= 0.0f && sp <= 150.0f) {
                g_SP = sp;
                Serial.print(F("SP=")); Serial.print(g_SP, 2);
                Serial.println(F(" °C"));
            } else {
                Serial.println(F("SP_ERR:fora de faixa (0..150 °C)"));
            }

        } else if (strcmp(g_cmdBuf, "START") == 0) {
            resetPID();
            g_saida = U_MIN;
            setPct(U_MIN);
            g_controlando = true;
            g_lastPID = millis();
            Serial.println(F("CONTROLE_INICIADO"));

        } else if (strcmp(g_cmdBuf, "STOP") == 0) {
            g_controlando = false;
            setPct(0);
            resetPID();
            Serial.println(F("CONTROLE_PARADO"));

        } else if (strcmp(g_cmdBuf, "STATUS") == 0) {
            Serial.print(F("Kp=")); Serial.print(g_Kp, 6);
            Serial.print(F("  Ki=")); Serial.print(g_Ki, 6);
            Serial.print(F("  Kd=")); Serial.println(g_Kd, 6);
            Serial.print(F("SP=")); Serial.print(g_SP, 2);
            Serial.print(F("°C  T=")); Serial.print(g_temp, 2);
            Serial.print(F("°C  u=")); Serial.print(g_saida, 1);
            Serial.println(F("%"));
            Serial.print(F("Controlando: "));
            Serial.println(g_controlando ? F("SIM") : F("NAO"));

        } else if (strcmp(g_cmdBuf, "CAL") == 0) {
            uint32_t s = 0;
            for (int i = 0; i < 1024; i++) s += analogRead(ACS_PIN);
            g_acsOffset = (uint16_t)(s >> 10);
            Serial.print(F("CAL_DONE:")); Serial.println(g_acsOffset);
        }
    }

    // ── Envio de dados (20 Hz) ──
    if (g_controlando && (now - g_lastSend >= SEND_MS)) {
        g_lastSend = now;
        float irms = medirRMS();

        // DATA:<ms>,<T>,<SP>,<erro>,<saida_%>,<irms_A>
        Serial.print(F("DATA:"));
        Serial.print(millis());       Serial.print(',');
        Serial.print(g_temp, 2);      Serial.print(',');
        Serial.print(g_SP, 2);        Serial.print(',');
        Serial.print(g_SP - g_temp, 3); Serial.print(',');
        Serial.print(g_saida, 2);     Serial.print(',');
        Serial.println(irms, 4);
    }

    // ── Monitor ZC (1 Hz) ──
    static unsigned long lastZC = 0;
    if (now - lastZC >= 1000) {
        lastZC = now;
        uint16_t cnt;
        noInterrupts(); cnt = g_zcCount; g_zcCount = 0; interrupts();
        if (cnt < 100 || cnt > 140)
            { Serial.print(F("WARN:ZC_RATE=")); Serial.println(cnt); }
    }
}
