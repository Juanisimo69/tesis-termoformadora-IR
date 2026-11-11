#pragma once

// ═══════════════════════════════════════
// PWMManager
// Configura y controla el canal PWM
// del ESP32 para el elemento calefactor.
// ═══════════════════════════════════════

#define PWM_CHANNEL    0 
#define PWM_PIN        25
#define PWM_FREQUENCY  20
#define PWM_RESOLUTION 10
#define PWM_MAX_DUTY   716

class PWMManager {
public:
    // Inicializa el canal PWM
    void begin();

    // Escribe un duty cycle en crudo (0..1023)
    void writeDuty(int duty);

    // Escribe un porcentaje (0.0 .. 100.0)
    // y lo convierte a duty respetando PWM_MAX_DUTY
    void writePct(float pct);

    // Igual que writePct pero SIN el cap de PWM_MAX_DUTY.
    // Usar solo en modo PWM Directo donde el usuario
    // elige el porcentaje de forma explícita.
    void writePctDirect(float pct);

    // Apaga la salida (duty = 0)
    void off();
};