#pragma once

#include "max6675.h"

// ═══════════════════════════════════════
// TemperatureSensor
// Encapsula la lectura del MAX6675
// y la calibración lineal.
// ═══════════════════════════════════════

#define THERMO_CLK 18
#define THERMO_DO  19
#define THERMO_CS   5

class TemperatureSensor {
public:
    // Crea el objeto MAX6675 internamente
    TemperatureSensor();

    // Inicializa (no requiere nada especial,
    // pero se llama en setup() para consistencia)
    void begin();

    // Devuelve la temperatura calibrada en °C
    float readCelsius();

private:
    MAX6675 _tc;

    // Aplica la calibración lineal
    float calibrate(float raw);
};