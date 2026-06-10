/*
 * teste_acs_dimmer.ino
 * Valida ACS712 + Dimmer (ZC + TRIAC) sem DS18B20
 * Plataforma: Arduino MEGA 2560
 *
 * Pinos (ajuste se necessário):
 *   ZC_PIN  = 3   → sinal ZC do MC-8A (INT1)
 *   DIM_PIN = 10  → gatilho do TRIAC
 *   ACS_PIN = A8  → saída analógica do ACS712-5A
 *
 * Serial Monitor: 115200 baud
 *
 * Comandos:
 *   CAL        → calibra offset do ACS (sem carga)
 *   SET:<pct>  → define potência 0-100%
 *   INFO       → mostra estado atual
 */

#define ZC_PIN   3
#define DIM_PIN  10
#define ACS_PIN  A8

// ACS712-5A: 185 mV/A, Vref 5V, 10 bits
#define ACS_SENS_mVpA  143.4f
#define HALF_CYCLE_US  8333UL   // 60 Hz
#define MIN_DELAY_US    500UL
#define PULSE_US        150UL
#define TICKS_PER_US      2UL

// ─── Estado ──────────────────────────────────────────────────────────────────
volatile uint16_t g_fireTicks =
    (uint16_t)((HALF_CYCLE_US - 400UL) * TICKS_PER_US);  // desligado

volatile uint16_t g_zcCount = 0;
uint16_t g_acsOffset  = 512;
float    g_pct        = 0.0;

// ─── ZC ISR ──────────────────────────────────────────────────────────────────
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
    if (OCR1A < (uint16_t)((HALF_CYCLE_US - 400UL) * TICKS_PER_US))
        digitalWrite(DIM_PIN, HIGH);
    TIMSK1 &= ~(1 << OCIE1A);
}

ISR(TIMER1_COMPB_vect) {
    digitalWrite(DIM_PIN, LOW);
    TIMSK1 = 0; TCCR1B = 0;
}

// ─── ACS712 ──────────────────────────────────────────────────────────────────
float medirRMS() {
    double sum = 0.0;
    int valid = 0;
    for (int i = 0; i < 480; i++) {   // 480 × 104µs ≈ 3 ciclos de 60Hz
        int dev = analogRead(ACS_PIN) - (int)g_acsOffset;
        if (abs(dev) > 900) continue;  // rejeita spike do TRIAC
        float I = dev * (5000.0f / 1024.0f) / ACS_SENS_mVpA;
        sum += (double)I * I;
        valid++;
    }
    if (valid < 10) return 0.0f;
    return sqrt(sum / valid);
}

uint16_t calibrarOffset() {
    uint32_t s = 0;
    for (int i = 0; i < 1024; i++) s += analogRead(ACS_PIN);
    return (uint16_t)(s >> 10);
}

// ─── Conversão % → delay_us ──────────────────────────────────────────────────
// LUT simplificada: inverte P(α) = (π−α + sin(2α)/2)/π
uint32_t pctToDelay(float pct) {
    pct = constrain(pct, 0.0f, 100.0f);
    float p = pct / 100.0f;
    // busca binária na curva P(α)
    float lo = 0.0f, hi = PI;
    for (int i = 0; i < 40; i++) {
        float mid = (lo + hi) / 2.0f;
        float pm  = (PI - mid + sin(2.0f * mid) / 2.0f) / PI;
        if (pm > p) lo = mid; else hi = mid;
    }
    float alpha = (lo + hi) / 2.0f;
    uint32_t d = (uint32_t)(alpha / PI * HALF_CYCLE_US);
    return constrain(d, MIN_DELAY_US, HALF_CYCLE_US - 400UL);
}

// ─── Setup ───────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    pinMode(DIM_PIN, OUTPUT);
    digitalWrite(DIM_PIN, LOW);
    pinMode(ZC_PIN, INPUT);
    TCCR1A = 0; TCCR1B = 0; TIMSK1 = 0;
    attachInterrupt(digitalPinToInterrupt(ZC_PIN), zeroCrossISR, RISING);
    Serial.println(F("=== Teste ACS712 + Dimmer ==="));
    Serial.println(F("Comandos: CAL | SET:<pct> | INFO"));
    Serial.println(F("Aguardando..."));
}

// ─── Varredura automática ─────────────────────────────────────────────────────
// Níveis testados e tempo de espera por ponto
const float NIVEIS[]   = {0, 15, 30, 45, 60, 75, 90, 100};
const int   N_NIVEIS   = 8;
const int   ESPERA_S   = 7;    // segundos por ponto  (8 × 7 = 56 s ≈ 1 min)

void setPct(float pct) {
    g_pct = constrain(pct, 0.0f, 100.0f);
    uint32_t d = pctToDelay(g_pct);
    noInterrupts();
    g_fireTicks = (uint16_t)(d * TICKS_PER_US);
    interrupts();
}

void varreduraAutomatica() {
    Serial.println(F("\n=== VARREDURA AUTOMÁTICA ==="));
    Serial.println(F("Pot%   I_med(A)  I_min(A)  I_max(A)  ZC/s"));
    Serial.println(F("-----  --------  --------  --------  ----"));

    for (int n = 0; n < N_NIVEIS; n++) {
        setPct(NIVEIS[n]);
        delay(2000);   // estabiliza o filamento

        // Coleta amostras durante ESPERA_S segundos
        float soma = 0, minI = 999, maxI = 0;
        int   amostras = 0;
        unsigned long tFim = millis() + (unsigned long)(ESPERA_S - 2) * 1000;

        while (millis() < tFim) {
            float irms = medirRMS();
            soma  += irms;
            if (irms < minI) minI = irms;
            if (irms > maxI) maxI = irms;
            amostras++;
            delay(100);
        }

        uint16_t cnt;
        noInterrupts(); cnt = g_zcCount; g_zcCount = 0; interrupts();

        float med = soma / amostras;
        Serial.print(F("  "));
        if (NIVEIS[n] < 100) Serial.print(F(" "));
        Serial.print(NIVEIS[n], 1);
        Serial.print(F("   "));
        Serial.print(med,  4); Serial.print(F("    "));
        Serial.print(minI, 4); Serial.print(F("    "));
        Serial.print(maxI, 4); Serial.print(F("    "));
        Serial.println(cnt);
    }

    // Desliga ao terminar
    setPct(0);
    Serial.println(F("\n=== FIM DA VARREDURA ==="));
}

// ─── Loop ────────────────────────────────────────────────────────────────────
unsigned long lastPrint = 0;

void loop() {
    // ── Leitura de comandos ──
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();

        if (cmd == "CAL") {
            Serial.println(F("Calibrando... (retire a carga)"));
            g_acsOffset = calibrarOffset();
            Serial.print(F("CAL_DONE: offset = "));
            Serial.println(g_acsOffset);

        } else if (cmd == "RUN") {
            varreduraAutomatica();

        } else if (cmd.startsWith("SET:")) {
            g_pct = cmd.substring(4).toFloat();
            setPct(g_pct);
            Serial.print(F("SET: ")); Serial.print(g_pct, 1); Serial.println(F("%"));

        } else if (cmd == "INFO") {
            Serial.print(F("Potência: ")); Serial.print(g_pct, 1); Serial.println(F("%"));
            Serial.print(F("Offset  : ")); Serial.println(g_acsOffset);
        }
    }

    // ── Impressão periódica (1 Hz) ──
    unsigned long now = millis();
    if (now - lastPrint >= 1000) {
        lastPrint = now;
        float irms = medirRMS();
        uint16_t cnt;
        noInterrupts(); cnt = g_zcCount; g_zcCount = 0; interrupts();

        Serial.print(F("P=")); Serial.print(g_pct, 1);
        Serial.print(F("%  I=")); Serial.print(irms, 4);
        Serial.print(F("A  ZC/s=")); Serial.print(cnt);
        if (cnt == 0)        Serial.print(F("  ERRO: ZC ausente!"));
        else if (cnt < 100 || cnt > 140) Serial.print(F("  AVISO: ZC fora do range"));
        else                 Serial.print(F("  OK"));
        Serial.println();
    }
}
