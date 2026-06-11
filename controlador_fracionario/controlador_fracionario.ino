/*
 * controlador_fracionario.ino
 * Controlador PI^λ D^μ (fracionário) de temperatura
 * Aproximação de Grünwald-Letnikov, Ts = 1 s, janela GL_N amostras
 *
 * Plataforma: Arduino MEGA 2560
 *
 * Pinos:
 *   ZC_PIN      = 3   → ZC do MC-8A (INT1)
 *   DIM_PIN     = 10  → Gatilho TRIAC
 *   ACS_PIN     = A8  → ACS712-5A (monitoramento)
 *   DS18B20_PIN = 26  → Sensor de temperatura
 *
 * ── Aproximação GL ────────────────────────────────────────────────────────
 *
 *   D^α f(t) ≈ (1/h^α) Σ_{j=0}^{N−1} w_j^(α) · f(t − j·h)
 *
 *   Pesos:  w_0 = 1,  w_j = w_{j−1} · (j − 1 − α) / j
 *
 *   Para Ts = 1 s → h^α = 1, sem fator de escala.
 *   Se mudar PID_MS, escale os ganhos: Ki_novo = Ki · (Ts_novo)^λ
 *   e Kd_novo = Kd / (Ts_novo)^μ.
 *
 *   λ ∈ (0,1]: 1.0 = integral clássico; < 1 = memória mais curta
 *   μ ∈ (0,1]: 1.0 = derivada clássica; < 1 = derivada suavizada
 *
 *   Controlador:
 *     u = Kp·e  +  Ki·D^{−λ}e  −  Kd·D^{μ}T
 *   (derivativo sobre a medição T, não sobre o erro — evita kick no setpoint)
 *
 *   Anti-windup: ao saturar, zera a entrada corrente no histórico de erro,
 *   congelando a componente integral fracionária a partir do próximo passo.
 *
 * ── Memória SRAM ──────────────────────────────────────────────────────────
 *
 *   GL_N = 64:  4 arrays × 64 floats × 4 bytes = 1 KB  (MEGA tem 8 KB)
 *
 * ── Protocolo serial (115200 baud, '\n') ─────────────────────────────────
 *
 *   Recebe:
 *     KP:<val>  → ganho proporcional
 *     KI:<val>  → ganho integral fracionário
 *     KD:<val>  → ganho derivativo fracionário
 *     LA:<val>  → ordem λ ∈ (0,1]  (ex: LA:0.8)
 *     MU:<val>  → ordem μ ∈ (0,1]  (ex: MU:0.5)
 *     SP:<val>  → setpoint (°C, 0–150)
 *     START     → inicia controle
 *     STOP      → para controle (saída → 0)
 *     STATUS    → parâmetros atuais
 *     RESET     → zera histórico GL sem parar controle
 *     CAL       → calibra offset do ACS712 (sem carga)
 *
 *   Envia (20 Hz enquanto controlando):
 *     DATA:<ms>,<T_C>,<SP_C>,<erro_C>,<saida_%>,<irms_A>
 */

#include <Arduino.h>
#include <math.h>
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
#define N_SAMPLES        480    // 3 ciclos de 60 Hz ≈ 50 ms

// ─── Limites de saída ────────────────────────────────────────────────────
#define U_MIN   15.0f
#define U_MAX  100.0f

// ─── GL: tamanho da janela de memória ────────────────────────────────────
// 64 amostras × Ts=1s → 64 s de memória fracionária
#define GL_N  64

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

// ─── Parâmetros do controlador ────────────────────────────────────────────
float g_Kp     = 0.0f;
float g_Ki     = 0.0f;
float g_Kd     = 0.0f;
float g_lambda = 1.0f;   // ordem do integrador (λ=1 → integral clássico)
float g_mu     = 1.0f;   // ordem do derivador  (μ=1 → derivada clássica)
float g_SP     = 30.0f;
float g_saida  = 0.0f;

// ─── GL: pesos pré-calculados e buffers circulares ────────────────────────
float   g_wI[GL_N];           // pesos GL para D^{-λ}e  (integrador)
float   g_wD[GL_N];           // pesos GL para D^{μ}T   (derivador sobre T)
float   g_err_buf[GL_N];      // histórico de erro para o integrador
float   g_temp_buf[GL_N];     // histórico de T para o derivador
uint8_t g_hist_idx   = 0;     // posição atual no buffer circular
uint8_t g_hist_count = 0;     // amostras válidas acumuladas (até GL_N)

// ─── Buffer serial ────────────────────────────────────────────────────────
#define CMD_BUF 32
char    g_cmdBuf[CMD_BUF];
uint8_t g_cmdIdx = 0;

// ─── Temporização ─────────────────────────────────────────────────────────
bool          g_controlando = false;
unsigned long g_lastPID     = 0;
unsigned long g_lastSend    = 0;
#define PID_MS   1000UL    // período do controlador (sincronizado com DS18B20)
#define SEND_MS    50UL    // período de envio (20 Hz)

// ─── Utilitários ─────────────────────────────────────────────────────────
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

// ─── GL: pré-cálculo dos pesos ────────────────────────────────────────────
// w_0 = 1,  w_j = w_{j-1} * (j - 1 - alpha) / j
// Integrador: alpha = -lambda  → pesos decrecem lentamente para zero
// Derivador:  alpha =  mu      → pesos alternam sinal e decaem (≈ diferença finita)
void computeWeights(float *w, float alpha, uint8_t N) {
    w[0] = 1.0f;
    for (uint8_t j = 1; j < N; j++)
        w[j] = w[j - 1] * ((j - 1.0f - alpha) / (float)j);
}

void resetController() {
    memset(g_err_buf,  0, sizeof(g_err_buf));
    memset(g_temp_buf, 0, sizeof(g_temp_buf));
    g_hist_idx   = 0;
    g_hist_count = 0;
    g_saida      = U_MIN;
}

// ─── Controlador PIλDμ ────────────────────────────────────────────────────
float calcFOPID(float T_atual) {
    float erro = g_SP - T_atual;

    // Avança buffer circular e registra a nova amostra
    g_hist_idx = (g_hist_idx + 1) % GL_N;
    if (g_hist_count < GL_N) g_hist_count++;
    g_err_buf[g_hist_idx]  = erro;
    g_temp_buf[g_hist_idx] = T_atual;

    // Proporcional
    float P = g_Kp * erro;

    // Integrador fracionário D^{-λ}e  (Ts=1s → h^λ=1)
    float I_acc = 0.0f;
    uint8_t nI = (g_hist_count < GL_N) ? g_hist_count : (uint8_t)GL_N;
    for (uint8_t j = 0; j < nI; j++) {
        uint8_t idx = (g_hist_idx - j + GL_N) % GL_N;
        I_acc += g_wI[j] * g_err_buf[idx];
    }

    // Derivador fracionário D^{μ}T sobre a medição (evita derivative kick)
    // Só entra quando há ao menos 2 amostras e Kd > 0
    float D = 0.0f;
    if (g_Kd > 0.0f && g_hist_count >= 2) {
        float D_acc = 0.0f;
        uint8_t nD = (g_hist_count < GL_N) ? g_hist_count : (uint8_t)GL_N;
        for (uint8_t j = 0; j < nD; j++) {
            uint8_t idx = (g_hist_idx - j + GL_N) % GL_N;
            D_acc += g_wD[j] * g_temp_buf[idx];
        }
        D = -g_Kd * D_acc;
    }

    float saida_raw = P + g_Ki * I_acc + D;
    float saida_sat = constrain(saida_raw, U_MIN, U_MAX);

    // Anti-windup: zera o erro recém-inserido se a saída saturou.
    // Na próxima iteração, a contribuição deste passo no somatório GL é nula,
    // congelando efetivamente o integrador fracionário enquanto houver saturação.
    if (fabsf(saida_raw - saida_sat) > 0.1f)
        g_err_buf[g_hist_idx] = 0.0f;

    g_saida = saida_sat;
    return saida_sat;
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

    // Pesos GL para ordens padrão (λ=1, μ=1 → PID clássico como caso base)
    computeWeights(g_wI, -g_lambda, GL_N);
    computeWeights(g_wD,  g_mu,     GL_N);
    resetController();

    Serial.println(F("READY"));
    Serial.println(F("PIlDm GL: KP: KI: KD: LA: MU: SP: START STOP STATUS RESET CAL"));
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

    // ── Controlador (1 Hz, sincronizado com DS18B20) ──
    if (g_controlando && (now - g_lastPID >= PID_MS)) {
        g_lastPID = now;
        if (g_temp > -100.0f) {
            float u = calcFOPID(g_temp);
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

        } else if (strncmp(g_cmdBuf, "LA:", 3) == 0) {
            float la = atof(g_cmdBuf + 3);
            if (la > 0.0f && la <= 1.0f) {
                g_lambda = la;
                computeWeights(g_wI, -g_lambda, GL_N);
                resetController();
                Serial.print(F("LA=")); Serial.println(g_lambda, 4);
                Serial.println(F("INFO:pesos_I recomputados, historico zerado"));
            } else {
                Serial.println(F("LA_ERR:fora de faixa (0,1]"));
            }

        } else if (strncmp(g_cmdBuf, "MU:", 3) == 0) {
            float mu = atof(g_cmdBuf + 3);
            if (mu > 0.0f && mu <= 1.0f) {
                g_mu = mu;
                computeWeights(g_wD, g_mu, GL_N);
                resetController();
                Serial.print(F("MU=")); Serial.println(g_mu, 4);
                Serial.println(F("INFO:pesos_D recomputados, historico zerado"));
            } else {
                Serial.println(F("MU_ERR:fora de faixa (0,1]"));
            }

        } else if (strncmp(g_cmdBuf, "SP:", 3) == 0) {
            float sp = atof(g_cmdBuf + 3);
            if (sp >= 0.0f && sp <= 150.0f) {
                g_SP = sp;
                Serial.print(F("SP=")); Serial.print(g_SP, 2); Serial.println(F(" C"));
            } else {
                Serial.println(F("SP_ERR:fora de faixa (0..150 C)"));
            }

        } else if (strcmp(g_cmdBuf, "START") == 0) {
            resetController();
            setPct(U_MIN);
            g_controlando = true;
            g_lastPID = millis();
            Serial.println(F("CONTROLE_INICIADO"));

        } else if (strcmp(g_cmdBuf, "STOP") == 0) {
            g_controlando = false;
            setPct(0);
            resetController();
            Serial.println(F("CONTROLE_PARADO"));

        } else if (strcmp(g_cmdBuf, "RESET") == 0) {
            resetController();
            Serial.println(F("HISTORICO_ZERADO"));

        } else if (strcmp(g_cmdBuf, "STATUS") == 0) {
            Serial.print(F("Kp=")); Serial.print(g_Kp, 4);
            Serial.print(F("  Ki=")); Serial.print(g_Ki, 4);
            Serial.print(F("  Kd=")); Serial.println(g_Kd, 4);
            Serial.print(F("lambda=")); Serial.print(g_lambda, 4);
            Serial.print(F("  mu=")); Serial.println(g_mu, 4);
            Serial.print(F("SP=")); Serial.print(g_SP, 2);
            Serial.print(F("C  T=")); Serial.print(g_temp, 2);
            Serial.print(F("C  u=")); Serial.print(g_saida, 1);
            Serial.println(F("%"));
            Serial.print(F("GL_N=")); Serial.print(GL_N);
            Serial.print(F("  amostras_acum=")); Serial.println(g_hist_count);
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
        Serial.print(millis());             Serial.print(',');
        Serial.print(g_temp, 2);            Serial.print(',');
        Serial.print(g_SP, 2);              Serial.print(',');
        Serial.print(g_SP - g_temp, 3);     Serial.print(',');
        Serial.print(g_saida, 2);           Serial.print(',');
        Serial.println(irms, 4);
    }

    // ── Monitor ZC (1 Hz) ──
    static unsigned long lastZC = 0;
    if (now - lastZC >= 1000UL) {
        lastZC = now;
        uint16_t cnt;
        noInterrupts(); cnt = g_zcCount; g_zcCount = 0; interrupts();
        if (cnt < 100 || cnt > 140) {
            Serial.print(F("WARN:ZC_RATE=")); Serial.println(cnt);
        }
    }
}
