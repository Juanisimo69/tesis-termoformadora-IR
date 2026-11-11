#pragma once

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ═══════════════════════════════════════
// DisplayManager
// Centraliza todo lo relacionado con el
// OLED: inicialización, pantallas y menús.
// ═══════════════════════════════════════

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64

// Lista de materiales disponibles
// (usada tanto en menús como en lógica)
static const int TOTAL_ANALOG_OPTIONS = 4;
static const int TOTAL_MAIN_OPTIONS   = 2;

class DisplayManager {
public:
    // Constructor: configura el objeto
    // SSD1306 con tamaño y pin de reset
    DisplayManager();

    // Inicializa el OLED, detiene el
    // programa si no lo encuentra
    void begin();

    // ── Pantallas ──────────────────────
    void drawSplash();
    void drawMainMenu(int selectedOption);
    void drawAnalogMenu(int selectedOption);
    void drawManualTemp(int temp);
    void drawManualTime(int seconds);
    void drawWifiScreen();

    // Pantallas modo PWM Directo
    void drawPwmPct(int pct);
    void drawPwmDirectScreen(
        float         temp,
        float         pwmPct,
        unsigned long remainingSecs
    );

    // Pantalla de proceso activo:
    // recibe los valores en vivo
    void drawProcessScreen(
        const char* material,
        float       currentTemp,
        float       setpoint,
        float       outputPct,
        unsigned long remainingSecs
    );

private:
    Adafruit_SSD1306 _display;
};