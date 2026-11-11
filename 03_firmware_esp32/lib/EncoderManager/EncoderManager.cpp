#include "EncoderManager.h"

void EncoderManager::begin() {
    pinMode(ENC_A,   INPUT_PULLUP);
    pinMode(ENC_B,   INPUT_PULLUP);
    pinMode(ENC_KEY, INPUT_PULLUP);

    _lastAState = digitalRead(ENC_A);
}

void EncoderManager::onRotate(EncoderCallback cb) {
    _rotateCb = cb;
}

void EncoderManager::onPress(ButtonCallback cb) {
    _pressCb = cb;
}

void EncoderManager::update() {
    _readEncoder();
    _readButton();
}

// ───────────────────────────────────────
// Lectura del encoder
// ───────────────────────────────────────
void EncoderManager::_readEncoder() {
    int currentA = digitalRead(ENC_A);

    if (_lastAState == LOW && currentA == HIGH) {
        if (millis() - _lastEncTime > ENC_DELAY) {

            int direction = (digitalRead(ENC_B) == HIGH) ? 1 : -1;

            _dirHistory[0] = _dirHistory[1];
            _dirHistory[1] = direction;

            // Solo dispara cuando se confirman
            // dos pulsos consecutivos en el
            // mismo sentido (antirebote)
            if (_dirHistory[0] == 1 && _dirHistory[1] == 1) {
                if (_rotateCb) _rotateCb(1);
                _resetHistory();
            }
            if (_dirHistory[0] == -1 && _dirHistory[1] == -1) {
                if (_rotateCb) _rotateCb(-1);
                _resetHistory();
            }

            _lastEncTime = millis();
        }
    }

    _lastAState = currentA;
}

// ───────────────────────────────────────
// Lectura del botón con debounce
// ───────────────────────────────────────
void EncoderManager::_readButton() {
    bool reading = digitalRead(ENC_KEY);

    if (reading != _lastReading) {
        _lastBtnTime = millis();
    }

    if ((millis() - _lastBtnTime) > BTN_DELAY) {
        if (reading != _stableState) {
            _stableState = reading;

            // Flanco de bajada = pulsación
            if (_stableState == LOW) {
                if (_pressCb) _pressCb();
            }
        }
    }

    _lastReading = reading;
}

void EncoderManager::_resetHistory() {
    _dirHistory[0] = 0;
    _dirHistory[1] = 0;
}