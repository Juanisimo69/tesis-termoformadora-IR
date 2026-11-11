#include "TemperatureSensor.h"

// El constructor inicializa el MAX6675
// con los pines definidos en el .h
TemperatureSensor::TemperatureSensor()
    : _tc(THERMO_CLK, THERMO_CS, THERMO_DO)
{}

void TemperatureSensor::begin() {
    // Sin inicialización especial requerida
    // por la librería MAX6675.
    // Se deja aquí para mantener la
    // convención begin() en todos los módulos.
}

float TemperatureSensor::readCelsius() {
    float raw = _tc.readCelsius();
    return calibrate(raw);
}

float TemperatureSensor::calibrate(float raw) {
    // Calibración lineal obtenida empíricamente:
    // temperatura_real = 0.83396 * raw + 9.5883
    return 0.83396f * raw + 9.5883f;
}