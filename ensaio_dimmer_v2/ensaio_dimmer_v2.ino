/*
 * ensaio_dimmer_v2.ino
 * Ensaio sem controlador: Dimmer (ZC + DIM) + ACS712 + DS18B20
 * Plataforma: Arduino MEGA 2560
 *
 * Ligações:
 *   ZC_PIN      = 3   → sinal de cruzamento por zero (INT1)
 *   DIM_PIN     = 10  → gatilho do TRIAC (MOC3021 + BT137)
 *   ACS_PIN     = A8  → ACS712 (saída analógica)
 *   DS18B20_PIN = 26  → DS18B20 data  + 4,7 kΩ pull-up para 5 V
 *
 * ── Melhorias v1 (qualidade de medição) ──────────────────────────────────
 *  [M1] measureRMS() sincronizada com ZC → elimina erro de truncamento de fase
 *  [M2] Offset dinâmico por janela → compensa drift térmico do ACS712
 *  [M3] calibrateOffset() desliga dimmer antes de amostrar → offset puro
 *  [M4] Validação de temperatura com faixa física [0, 200°C]
 *  [M5] Filtro EMA na temperatura (α = 0.2); inicializa na 1ª leitura válida
 *
 * ── Melhorias v2 (linearidade do dimmer) ─────────────────────────────────
 *  [M6] LUT de correção do ângulo de disparo em PROGMEM
 *       Offset em µs somado ao delay calculado pelo NR.
 *       Ponto zero: 75% (erro medido = 0,0%).
 *  [M7] SET:<pct> → NR + LUT (corrige fórmula errada da versão anterior)
 *
 * ── Correções vs versão anterior ─────────────────────────────────────────
 *  [C1] ACS_SENS_mVpA corrigido: 143.4 → 185.0 mV/A (datasheet ACS712-5A)
 *       Ajustável em runtime via SENS:<mV/A>
 *  [C2] pctToDelayUs() usava aproximação sqrt errada; substituído por NR exato
 *  [C3] g_temperature inicializado em -127 (sentinel); EMA inicia na 1ª leitura
 *  [C4] sin()/sqrt() → sinf()/sqrtf() (float, mais rápido no AVR)
 *  [C5] Protocolo unificado: SET:<pct> (não delay_us direto)
 *
 * ── Garantias de timing ───────────────────────────────────────────────────
 *  1. Escrita atômica de 16 bits: noInterrupts()/interrupts() em g_fireTicks
 *  2. Reset forçado DIM=LOW sempre no ZC ISR
 *  3. Atraso mínimo MIN_DELAY_US para evitar OCR1A=0
 *  4. Captura local de g_fireTicks no ZC ISR
 *
 * Protocolo Serial (115200 baud, '\n'):
 *   Recebe:  SET:<pct>      → potência (0.0–100.0 %); converte via NR + LUT
 *            SENS:<mV/A>    → ajusta sensibilidade ACS712 (ex: SENS:173.2)
 *            START | STOP | CAL
 *   Envia:   DATA:<millis>,<delay_us>,<irms_A>,<temp_C>,<urms_V>,<prms_W>
 *            ACK:<delay_us> | SENS_OK:<val> | CAL_DONE:<offset> | READY | WARN:...
 */

#include <Arduino.h>
#include <math.h>       // libm: sinf, cosf, fabsf — float, mais rápido no AVR
#include <OneWire.h>
#include <DallasTemperature.h>

// ─── Pinos ────────────────────────────────────────────────────────────────
#define ZC_PIN       3
#define DIM_PIN      10
#define ACS_PIN      A8
#define DS18B20_PIN  26

// ─── ACS712 ───────────────────────────────────────────────────────────────
// [C1] Sensibilidade corrigida: 5A→185.0 | 20A→100.0 | 30A→66.0 (mV/A)
#define ACS_SENS_DEFAULT  185.0f
#define ADC_VREF_mV       5000.0f
#define ADC_BITS          1024
#define N_SAMPLES          480     // 480 × 104µs ≈ 49.9ms = 3 ciclos @ 60 Hz
#define ACS_SPIKE_LIMIT    900

float g_acsSens = ACS_SENS_DEFAULT;   // ajustável via SENS:<mV/A>

// ─── Dimmer — 60 Hz (Manaus) ──────────────────────────────────────────────
#define HALF_CYCLE_US    8333UL
#define PULSE_US           150UL
#define SAFETY_MARGIN_US   400UL
#define MIN_DELAY_US       500UL
#define TICKS_PER_US         2UL

#define ALPHA_MIN  0.18850f   // 500us  / 8333us × π
#define ALPHA_MAX  2.99252f   // 7933us / 8333us × π

// ─── DS18B20 ─────────────────────────────────────────────────────────────
OneWire           oneWire(DS18B20_PIN);
DallasTemperature sensors(&oneWire);
float             g_temperature     = -127.0f;  // [C3] sentinel até 1ª leitura
unsigned long     g_lastTempRequest = 0;
#define TEMP_RESOLUTION          11
#define TEMP_CONV_MS            400UL
#define TEMP_REQUEST_INTERVAL  1000UL
#define TEMP_MIN_VALID           0.0f
#define TEMP_MAX_VALID         200.0f
#define TEMP_FILTER_ALPHA        0.2f

// ─── [M6] LUT de correção do ângulo de disparo (PROGMEM) ─────────────────
// Gerada do ensaio_varredura_20260601_201853_completo.csv
// Offset em µs a SOMAR ao delay calculado pelo NR.
// Negativo = avança o gatilho | Positivo = atrasa o gatilho
// Ponto zero: 75% (erro medido = 0,0%)
static const int16_t  LUT_CORR_US[]  PROGMEM = {
  // 15%    20%    25%    30%    35%    40%    45%    50%    55%    60%
  -1062,  -961,  -892,  -821,  -756,  -677,  -594,  -521,  -430,  -336,
  // 65%    70%    75%    80%    85%    90%    95%   100%
   -230,  -115,     0,   140,   306,   536,   887,  1941
};
static const uint16_t LUT_PCT_X100[] PROGMEM = {
  1501, 2001, 2501, 3001, 3501, 4002, 4501, 5001,
  5502, 6001, 6501, 7001, 7501, 8001, 8501, 9001, 9501, 9986
};
#define LUT_SIZE 18

// Interpolação linear na LUT
int16_t lutCorrection(float pct) {
    uint16_t p = (uint16_t)(pct * 100.0f + 0.5f);
    if (p <= pgm_read_word(&LUT_PCT_X100[0]))
        return (int16_t)pgm_read_word(&LUT_CORR_US[0]);
    if (p >= pgm_read_word(&LUT_PCT_X100[LUT_SIZE - 1]))
        return (int16_t)pgm_read_word(&LUT_CORR_US[LUT_SIZE - 1]);
    for (uint8_t i = 0; i < LUT_SIZE - 1; i++) {
        uint16_t p0 = pgm_read_word(&LUT_PCT_X100[i]);
        uint16_t p1 = pgm_read_word(&LUT_PCT_X100[i + 1]);
        if (p >= p0 && p <= p1) {
            int16_t  c0  = (int16_t)pgm_read_word(&LUT_CORR_US[i]);
            int16_t  c1  = (int16_t)pgm_read_word(&LUT_CORR_US[i + 1]);
            int32_t  num = (int32_t)(c1 - c0) * (int32_t)(p - p0);
            return c0 + (int16_t)(num / (int32_t)(p1 - p0));
        }
    }
    return 0;
}

// ─── Conversão potência (%) → ângulo de disparo (rad) ────────────────────
// Newton-Raphson sobre P(α) = (π − α + sin(2α)/2) / π
// Ref: Michel Loureiro, jun/2026.
// f(α)  = P(α) − P_ref = 0
// f'(α) = (cos(2α) − 1) / π
// α₀    = π(1 − P_ref)
// [C2] Substitui a aproximação sqrt errada da versão anterior
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

// [M7] pct → delay_us via NR + correção LUT
uint32_t pctToDelayUs(float pct) {
    int32_t delay = (int32_t)(potencia_para_alpha(pct) / PI * (float)HALF_CYCLE_US)
                  + (int32_t)lutCorrection(pct);
    return (uint32_t)constrain(delay,
                               (long)MIN_DELAY_US,
                               (long)(HALF_CYCLE_US - SAFETY_MARGIN_US));
}

// ─── Estado global ────────────────────────────────────────────────────────
volatile uint16_t g_fireTicks =
    (uint16_t)((HALF_CYCLE_US - SAFETY_MARGIN_US) * TICKS_PER_US);

volatile uint16_t g_zcCount = 0;
volatile bool     g_zcPulse = false;  // [M1]

unsigned long g_lastZCCheck      = 0;
uint16_t      g_lastValidSamples = 0;
bool          g_sending          = false;
unsigned long g_lastSend         = 0;
#define       SEND_MS  50

#define CMD_BUF_LEN 32
char    g_cmdBuf[CMD_BUF_LEN];
uint8_t g_cmdIdx = 0;

// ─── Buffer serial ────────────────────────────────────────────────────────
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

// ─── [M1][M2] measureRMS — sincronizada com ZC + offset dinâmico ──────────
float measureRMS() {
    // [M1] Aguarda próximo ZC (timeout 30ms ≈ 2 semiciclos)
    g_zcPulse = false;
    unsigned long t0 = millis();
    while (!g_zcPulse) {
        if (millis() - t0 > 30UL) {
            Serial.println(F("WARN:ZC_TIMEOUT_RMS"));
            return 0.0f;
        }
    }

    // [M2] Coleta amostras e calcula offset dinâmico da janela
    static int raw_buf[N_SAMPLES];
    int32_t sumRaw = 0;
    for (uint16_t i = 0; i < N_SAMPLES; i++) {
        raw_buf[i]  = analogRead(ACS_PIN);
        sumRaw     += raw_buf[i];
    }
    int16_t dynOffset = (int16_t)(sumRaw / (int32_t)N_SAMPLES);

    double   sumSq = 0.0;
    uint16_t valid = 0;
    for (uint16_t i = 0; i < N_SAMPLES; i++) {
        int dev = raw_buf[i] - dynOffset;
        if (dev > ACS_SPIKE_LIMIT || dev < -ACS_SPIKE_LIMIT) continue;
        float V  = (float)dev * (ADC_VREF_mV / (float)ADC_BITS);
        float I  = V / g_acsSens;
        sumSq   += (double)I * I;
        valid++;
    }
    g_lastValidSamples = valid;
    if (valid < 10) return 0.0f;
    return sqrtf((float)(sumSq / valid));  // [C4]
}

// ─── [M3] calibrateOffset — dimmer desligado antes de amostrar ───────────
void calibrateOffset() {
    uint16_t savedTicks;
    noInterrupts();
    savedTicks  = g_fireTicks;
    g_fireTicks = (uint16_t)((HALF_CYCLE_US - SAFETY_MARGIN_US) * TICKS_PER_US);
    interrupts();

    delay(200);   // ≥ 12 semiciclos @ 60 Hz — TRIAC apaga

    uint32_t s = 0;
    for (uint16_t i = 0; i < 1024; i++) s += analogRead(ACS_PIN);
    uint16_t offset = (uint16_t)(s >> 10);

    noInterrupts();
    g_fireTicks = savedTicks;
    interrupts();

    Serial.print(F("CAL_DONE:")); Serial.println(offset);
}

// ─── ZC ISR ──────────────────────────────────────────────────────────────
void zeroCrossISR() {
    static unsigned long lastZC = 0;
    unsigned long now = micros();
    if (now - lastZC < 4000UL) return;
    lastZC = now;

    g_zcCount++;
    g_zcPulse = true;           // [M1]
    digitalWrite(DIM_PIN, LOW); // GARANTIA #2
    TCCR1B = 0;
    TCCR1A = 0;

    uint16_t ft = g_fireTicks;  // GARANTIA #4
    TCNT1  = 0;
    OCR1A  = ft;
    OCR1B  = ft + (uint16_t)(PULSE_US * TICKS_PER_US);
    TIFR1  = (1 << OCF1A) | (1 << OCF1B);
    TIMSK1 = (1 << OCIE1A) | (1 << OCIE1B);
    TCCR1B = (1 << CS11);
}

ISR(TIMER1_COMPA_vect) {
    if (OCR1A < (uint16_t)((HALF_CYCLE_US - SAFETY_MARGIN_US) * TICKS_PER_US))
        digitalWrite(DIM_PIN, HIGH);
    TIMSK1 &= ~(1 << OCIE1A);
}

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
    unsigned long now = millis();

    // ── [M4][M5] Temperatura com validação e filtro EMA ───────────────────
    if (now - g_lastTempRequest >= TEMP_CONV_MS) {
        float t = sensors.getTempCByIndex(0);
        if (t >= TEMP_MIN_VALID && t < TEMP_MAX_VALID) {
            // [C3] Inicializa EMA na primeira leitura válida
            if (g_temperature < -100.0f)
                g_temperature = t;
            else
                g_temperature = g_temperature * (1.0f - TEMP_FILTER_ALPHA)
                              + t * TEMP_FILTER_ALPHA;
        } else {
            Serial.print(F("WARN:TEMP_INVALIDA="));
            Serial.println(t, 1);
        }
        if (now - g_lastTempRequest >= TEMP_REQUEST_INTERVAL) {
            sensors.requestTemperatures();
            g_lastTempRequest = now;
        }
    }

    // ── Monitor de taxa de ZC (1 Hz) ──────────────────────────────────────
    if (millis() - g_lastZCCheck >= 1000UL) {
        g_lastZCCheck = millis();
        uint16_t cnt;
        noInterrupts(); cnt = g_zcCount; g_zcCount = 0; interrupts();
        if (cnt < 100 || cnt > 140)
            { Serial.print(F("WARN:ZC_RATE=")); Serial.println(cnt); }
        if (g_sending && g_lastValidSamples < N_SAMPLES / 2) {
            Serial.print(F("WARN:ADC_SPIKE_PCT="));
            Serial.println(100U - (uint32_t)g_lastValidSamples * 100U / N_SAMPLES);
        }
    }

    // ── Comandos seriais ──────────────────────────────────────────────────
    if (serialReadLine()) {

        if (strncmp(g_cmdBuf, "SET:", 4) == 0) {
            // [C5] Recebe pct, converte via NR + LUT
            float pct = atof(g_cmdBuf + 4);
            pct = constrain(pct, 0.0f, 100.0f);
            uint32_t val = pctToDelayUs(pct);
            noInterrupts();
            g_fireTicks = (uint16_t)(val * TICKS_PER_US);
            interrupts();
            Serial.print(F("ACK:")); Serial.println(val);

        } else if (strncmp(g_cmdBuf, "SENS:", 5) == 0) {
            float s = atof(g_cmdBuf + 5);
            if (s > 10.0f && s < 500.0f) {
                g_acsSens = s;
                Serial.print(F("SENS_OK:")); Serial.println(g_acsSens, 2);
            } else {
                Serial.println(F("WARN:SENS_INVALIDA"));
            }

        } else if (strcmp(g_cmdBuf, "START") == 0) {
            g_sending  = true;
            g_lastSend = millis();
            Serial.println(F("TEST_STARTED"));

        } else if (strcmp(g_cmdBuf, "STOP") == 0) {
            g_sending = false;
            uint32_t val = pctToDelayUs(0.0f);
            noInterrupts();
            g_fireTicks = (uint16_t)(val * TICKS_PER_US);
            interrupts();
            Serial.println(F("TEST_STOPPED"));

        } else if (strcmp(g_cmdBuf, "CAL") == 0) {
            calibrateOffset();   // [M3] já envia CAL_DONE
        }
    }

    // ── Envio de dados ────────────────────────────────────────────────────
    if (g_sending && (millis() - g_lastSend >= SEND_MS)) {
        g_lastSend = millis();

        float    irms = measureRMS();
        uint32_t delayUs;
        noInterrupts(); delayUs = g_fireTicks / TICKS_PER_US; interrupts();

        // [C4] sinf/sqrtf — float, mais rápido no AVR
        float alpha = (float)delayUs / (float)HALF_CYCLE_US * PI;
        float pNorm = (PI - alpha + sinf(2.0f * alpha) / 2.0f) / PI;
        float urms  = 127.0f * sqrtf(max(pNorm, 0.0f));
        float prms  = urms * irms;

        Serial.print(F("DATA:"));
        Serial.print(millis());          Serial.print(',');
        Serial.print(delayUs);           Serial.print(',');
        Serial.print(irms,   4);         Serial.print(',');
        Serial.print(g_temperature, 2);  Serial.print(',');
        Serial.print(urms,   2);         Serial.print(',');
        Serial.println(prms, 3);
    }
}
