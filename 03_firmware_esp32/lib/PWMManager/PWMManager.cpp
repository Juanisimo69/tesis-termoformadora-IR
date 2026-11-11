#include "PWMManager.h"
#include <Arduino.h>

void PWMManager::begin() {
    ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(PWM_PIN, PWM_CHANNEL);
    ledcWrite(PWM_CHANNEL, 0);
}

void PWMManager::writeDuty(int duty) {
    duty = constrain(duty, 0, PWM_MAX_DUTY);
    ledcWrite(PWM_CHANNEL, duty);
}

void PWMManager::writePct(float pct) {
    int duty = (int)(pct / 100.0f * 1023.0f);
    writeDuty(duty);   // pasa por el cap PWM_MAX_DUTY
}

void PWMManager::writePctDirect(float pct) {
    // Sin cap: 100% → duty 1023 (ciclo de trabajo completo).
    // Solo se usa desde el modo PWM Directo.
    int duty = constrain((int)(pct / 100.0f * 1023.0f), 0, 1023);
    ledcWrite(PWM_CHANNEL, duty);
}

void PWMManager::off() {
    ledcWrite(PWM_CHANNEL, 0);
}