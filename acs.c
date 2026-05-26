/**
 * ╔══════════════════════════════════════════════════════════════════════════╗
 * ║  ACS — Altitude Control System por Freio Aerodinâmico (Emulado)        ║
 * ║  Plataforma: BitDogLab / Raspberry Pi Pico W (RP2040)                  ║
 * ║                                                                          ║
 * ║  Core 0: interface — WS2812B, LED RGB, buzzer, botão de arme            ║
 * ║  Core 1: malha de controle PD em tempo real — ADC + PWM servo           ║
 * ╚══════════════════════════════════════════════════════════════════════════╝
 */

#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "ws2812.pio.h"

// ═══════════════════════════════════════════════════════════════════════════
// PINAGEM — BitDogLab
// ═══════════════════════════════════════════════════════════════════════════
#define PIN_WS2812      7   // Matriz 5×5 WS2812B
#define PIN_BTN_ARME    5   // Botão A — arme/desarme (interrupção)
#define PIN_JOY_Y       27  // ADC1 — eixo Y do joystick (altitude emulada)
#define PIN_SERVO       28  // PWM — servo motor (flap de freio)  ← GPIO livre
#define PIN_BUZZER      21  // PWM — buzzer passivo
#define PIN_LED_R       13  // LED RGB — canal vermelho
#define PIN_LED_G       11  // LED RGB — canal verde
#define PIN_LED_B       12  // LED RGB — canal azul

// ═══════════════════════════════════════════════════════════════════════════
// PARÂMETROS DO SISTEMA
// ═══════════════════════════════════════════════════════════════════════════
#define NUM_LEDS            25
#define ADC_CENTER          2048        // centro do range ADC (12 bits)
#define ADC_MAX_DEVIATION   2048.0f     // desvio máximo normalizado

#define SAMPLE_PERIOD_MS    20          // período da malha de controle
#define FILTER_SIZE         4           // janela da média móvel (anti-ruído D)

// Ganhos do controlador PD (ajuste fino em bancada)
#define KP_DEFAULT          0.8f
#define KD_DEFAULT          0.15f

// Servo: 50 Hz — pulso 1000–2000 μs
// clkdiv=125 → 1 MHz → 1 count = 1 μs — wrap=19999 → período 20 ms
#define SERVO_PWM_CLKDIV    125.0f
#define SERVO_PWM_WRAP      19999
#define SERVO_MIN_US        1000        // freio fechado (0°)
#define SERVO_MAX_US        2000        // freio totalmente aberto (90°)

// Buzzer: frequência proporcional ao erro
#define BUZZER_CLKDIV       10.0f       // clk efetivo = 12.5 MHz
#define BUZZER_FREQ_MIN     250         // Hz — erro no limiar
#define BUZZER_FREQ_MAX     1000        // Hz — erro máximo
#define ERROR_BUZZER_THRESH 0.4f        // limiar para ativar buzzer

// ═══════════════════════════════════════════════════════════════════════════
// ESTADO COMPARTILHADO — Core 0 ↔ Core 1
// Acesso via volatile; dados são somente de display (leitura tolerante a tearing)
// ═══════════════════════════════════════════════════════════════════════════
static volatile float g_error_norm  = 0.0f;  // erro normalizado  [-1.0, +1.0]
static volatile float g_control_out = 0.0f;  // ação de controle  [ 0.0, +1.0]
static volatile bool  g_armed       = false;

// ═══════════════════════════════════════════════════════════════════════════
// WS2812B
// ═══════════════════════════════════════════════════════════════════════════
static PIO     ws_pio = pio0;
static uint    ws_sm  = 0;
static uint32_t led_buf[NUM_LEDS];

// Formato GRB — exigido pelo WS2812B
static inline uint32_t grb(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)g << 16) | ((uint32_t)r << 8) | (uint32_t)b;
}

static void ws_send(void) {
    for (int i = 0; i < NUM_LEDS; i++)
        pio_sm_put_blocking(ws_pio, ws_sm, led_buf[i] << 8u);
}

static void ws_clear(void) {
    for (int i = 0; i < NUM_LEDS; i++) led_buf[i] = 0;
}

// Mapeamento serpentina da BitDogLab
// Linhas pares: esq → dir | Linhas ímpares: dir → esq
static inline int led_idx(int row, int col) {
    return (row % 2 == 0) ? row * 5 + col : row * 5 + (4 - col);
}

// ═══════════════════════════════════════════════════════════════════════════
// SERVO
// ═══════════════════════════════════════════════════════════════════════════
static uint servo_slice, servo_chan;

static void servo_init(void) {
    gpio_set_function(PIN_SERVO, GPIO_FUNC_PWM);
    servo_slice = pwm_gpio_to_slice_num(PIN_SERVO);
    servo_chan  = pwm_gpio_to_channel(PIN_SERVO);

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, SERVO_PWM_CLKDIV);   // 1 MHz
    pwm_config_set_wrap(&cfg, SERVO_PWM_WRAP);         // 20 ms
    pwm_init(servo_slice, &cfg, true);

    pwm_set_chan_level(servo_slice, servo_chan, SERVO_MIN_US); // fechado
}

// ctrl ∈ [0.0, 1.0] → pulso ∈ [1000, 2000] μs
static void servo_set(float ctrl) {
    if (ctrl < 0.0f) ctrl = 0.0f;
    if (ctrl > 1.0f) ctrl = 1.0f;
    uint16_t us = SERVO_MIN_US + (uint16_t)(ctrl * (SERVO_MAX_US - SERVO_MIN_US));
    pwm_set_chan_level(servo_slice, servo_chan, us);
}

// ═══════════════════════════════════════════════════════════════════════════
// BUZZER
// ═══════════════════════════════════════════════════════════════════════════
static uint buz_slice, buz_chan;

static void buzzer_init(void) {
    gpio_set_function(PIN_BUZZER, GPIO_FUNC_PWM);
    buz_slice = pwm_gpio_to_slice_num(PIN_BUZZER);
    buz_chan  = pwm_gpio_to_channel(PIN_BUZZER);
    pwm_set_enabled(buz_slice, false);
}

static void buzzer_tone(uint32_t freq_hz) {
    if (freq_hz == 0) { pwm_set_enabled(buz_slice, false); return; }

    // clk_efetivo = 125 MHz / BUZZER_CLKDIV = 12.5 MHz
    float clk_efetivo = clock_get_hz(clk_sys) / BUZZER_CLKDIV;
    uint32_t wrap = (uint32_t)(clk_efetivo / freq_hz) - 1;

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, BUZZER_CLKDIV);
    pwm_config_set_wrap(&cfg, wrap);
    pwm_init(buz_slice, &cfg, true);
    pwm_set_chan_level(buz_slice, buz_chan, wrap / 2); // 50% duty cycle
}

// ═══════════════════════════════════════════════════════════════════════════
// LED RGB
// ═══════════════════════════════════════════════════════════════════════════
typedef enum { COLOR_OFF, COLOR_GREEN, COLOR_YELLOW, COLOR_RED } RgbColor;

static void led_rgb_init(void) {
    const uint pins[] = {PIN_LED_R, PIN_LED_G, PIN_LED_B};
    for (int i = 0; i < 3; i++) {
        gpio_init(pins[i]);
        gpio_set_dir(pins[i], GPIO_OUT);
        gpio_put(pins[i], 0);
    }
}

static void led_rgb_set(RgbColor c) {
    gpio_put(PIN_LED_R, c == COLOR_RED    || c == COLOR_YELLOW);
    gpio_put(PIN_LED_G, c == COLOR_GREEN  || c == COLOR_YELLOW);
    gpio_put(PIN_LED_B, 0);
}

// ═══════════════════════════════════════════════════════════════════════════
// BOTÃO DE ARME — interrupção em GPIO_IRQ_EDGE_FALL
// ═══════════════════════════════════════════════════════════════════════════
static volatile uint32_t last_btn_ms = 0;

static void btn_irq_handler(uint gpio, uint32_t events) {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (now - last_btn_ms < 250) return; // debounce por software
    last_btn_ms = now;
    g_armed = !g_armed;
}

// ═══════════════════════════════════════════════════════════════════════════
// VISUALIZAÇÃO NA MATRIZ 5×5
//
//  Linhas 0–3: cursor de erro — coluna desloca esq↔dir conforme e_norm
//              Verde < 15% | Amarelo 15–40% | Vermelho > 40%
//  Linha 4:    barra de abertura do servo (output do controlador)
//              Amarelo — indica quanto o freio está aberto
// ═══════════════════════════════════════════════════════════════════════════
static void matrix_update(float e_norm, float ctrl, bool armed) {
    ws_clear();

    if (!armed) {
        // Desarmado: LED central pulsando em azul fraco
        led_buf[led_idx(2, 2)] = grb(0, 0, 15);
        ws_send();
        return;
    }

    // --- Cursor de erro (linhas 0–3) ---
    // Coluna central (2) = setpoint; cursor desloca ±2 colunas
    float abs_e = fabsf(e_norm);

    uint8_t r, g, b;
    if (abs_e < 0.15f) {
        r = 0;  g = 50; b = 0;   // verde
    } else if (abs_e < 0.40f) {
        r = 60; g = 35; b = 0;   // amarelo
    } else {
        r = 80; g = 0;  b = 0;   // vermelho
    }

    // Coluna 2 sempre acesa em verde fraco (referência do setpoint)
    for (int row = 0; row < 4; row++)
        led_buf[led_idx(row, 2)] = grb(0, 12, 0);

    // Cursor: mapeia e_norm [-1,+1] → coluna [0,4]
    int col = (int)(2.0f + e_norm * 2.0f + 0.5f);
    if (col < 0) col = 0;
    if (col > 4) col = 4;
    for (int row = 0; row < 4; row++)
        led_buf[led_idx(row, col)] = grb(r, g, b);

    // --- Barra de abertura do servo (linha 4) ---
    // ctrl ∈ [0,1] → 0 a 4 LEDs acesos da esquerda
    int barras = (int)(ctrl * 4.0f + 0.5f);
    for (int c = 0; c < barras; c++)
        led_buf[led_idx(4, c)] = grb(60, 40, 0); // amarelo

    ws_send();
}

// ═══════════════════════════════════════════════════════════════════════════
// CORE 1 — Malha de controle PD em tempo real
// ═══════════════════════════════════════════════════════════════════════════

// Filtro de média móvel — amortece ruído para o termo derivativo
static float filt_buf[FILTER_SIZE];
static int   filt_idx = 0;

static float moving_avg(float v) {
    filt_buf[filt_idx] = v;
    filt_idx = (filt_idx + 1) % FILTER_SIZE;
    float s = 0.0f;
    for (int i = 0; i < FILTER_SIZE; i++) s += filt_buf[i];
    return s / FILTER_SIZE;
}

static void core1_entry(void) {
    // Inicializa ADC e servo dentro do Core 1
    adc_init();
    adc_gpio_init(PIN_JOY_Y);
    servo_init();

    float e_prev = 0.0f;
    const float kp = KP_DEFAULT;
    const float kd = KD_DEFAULT;
    const float T  = SAMPLE_PERIOD_MS / 1000.0f; // período em segundos

    // Timing drift-free: acumula o instante absoluto do próximo ciclo
    absolute_time_t next = get_absolute_time();

    while (true) {
        next = delayed_by_ms(next, SAMPLE_PERIOD_MS);
        sleep_until(next); // aguarda precisamente o próximo ciclo

        bool armed = g_armed; // leitura atômica (bool é 1 byte)

        if (!armed) {
            // Sistema desarmado: fecha o freio e zera o estado
            servo_set(0.0f);
            e_prev = 0.0f;
            for (int i = 0; i < FILTER_SIZE; i++) filt_buf[i] = 0.0f;
            g_error_norm  = 0.0f;
            g_control_out = 0.0f;
            continue;
        }

        // ── Leitura do ADC (joystick Y, canal ADC1) ──────────────────────
        adc_select_input(1);
        uint16_t raw = adc_read();

        // ── Normalização e filtragem ──────────────────────────────────────
        float e_raw  = (float)raw - ADC_CENTER;
        float e_norm = moving_avg(e_raw / ADC_MAX_DEVIATION); // [-1.0, +1.0]

        // ── Controlador PD ────────────────────────────────────────────────
        // u = Kp·e[n] + Kd·(e[n] − e[n−1]) / T
        float u = kp * e_norm + kd * (e_norm - e_prev) / T;
        e_prev = e_norm;

        // ── Saturação unidirecional: só frenagem, sem aceleração ──────────
        if (u < 0.0f) u = 0.0f;
        if (u > 1.0f) u = 1.0f;

        // ── Atuação ───────────────────────────────────────────────────────
        servo_set(u);

        // ── Publica estado para Core 0 ────────────────────────────────────
        g_error_norm  = e_norm;
        g_control_out = u;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// CORE 0 — Setup e loop de display
// ═══════════════════════════════════════════════════════════════════════════
int main(void) {
    stdio_init_all();

    // ── WS2812B ────────────────────────────────────────────────────────────
    uint ws_offset = pio_add_program(ws_pio, &ws2812_program);
    ws2812_program_init(ws_pio, ws_sm, ws_offset, PIN_WS2812, 800000, false);
    ws_clear();
    ws_send();

    // ── LED RGB ─────────────────────────────────────────────────────────────
    led_rgb_init();
    led_rgb_set(COLOR_RED); // começa desarmado

    // ── Buzzer ──────────────────────────────────────────────────────────────
    buzzer_init();

    // ── Botão de arme — interrupção ─────────────────────────────────────────
    gpio_init(PIN_BTN_ARME);
    gpio_set_dir(PIN_BTN_ARME, GPIO_IN);
    gpio_pull_up(PIN_BTN_ARME);
    gpio_set_irq_enabled_with_callback(PIN_BTN_ARME,
        GPIO_IRQ_EDGE_FALL, true, btn_irq_handler);

    // ── Inicia Core 1 ────────────────────────────────────────────────────────
    multicore_launch_core1(core1_entry);

    // ── Loop de display (Core 0) ─────────────────────────────────────────────
    while (true) {
        // Lê estado publicado pelo Core 1
        float err  = g_error_norm;
        float ctrl = g_control_out;
        bool  arm  = g_armed;

        // LED RGB — indica estado geral do sistema
        if (!arm) {
            led_rgb_set(COLOR_RED);           // desarmado
        } else if (fabsf(err) < 0.15f) {
            led_rgb_set(COLOR_GREEN);          // armado — dentro da zona OK
        } else if (fabsf(err) < 0.40f) {
            led_rgb_set(COLOR_YELLOW);         // armado — desvio moderado
        } else {
            led_rgb_set(COLOR_RED);            // armado — desvio crítico
        }

        // Buzzer — ativa proporcional ao erro acima do limiar
        if (arm && fabsf(err) >= ERROR_BUZZER_THRESH) {
            float t = (fabsf(err) - ERROR_BUZZER_THRESH) / (1.0f - ERROR_BUZZER_THRESH);
            uint32_t freq = BUZZER_FREQ_MIN + (uint32_t)(t * (BUZZER_FREQ_MAX - BUZZER_FREQ_MIN));
            buzzer_tone(freq);
        } else {
            buzzer_tone(0);
        }

        // Matriz de LEDs
        matrix_update(err, ctrl, arm);

        // Display atualiza a 20 Hz (50 ms)
        sleep_ms(50);
    }
}
