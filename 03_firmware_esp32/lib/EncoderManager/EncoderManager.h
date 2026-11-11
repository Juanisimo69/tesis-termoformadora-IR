#pragma once

#include <Arduino.h>
#include <functional>

// ═══════════════════════════════════════
// EncoderManager
// Lee el encoder rotativo y el botón.
// Notifica al main.cpp mediante callbacks
// en lugar de acceder directamente al
// estado de los menús.
// ═══════════════════════════════════════

#define ENC_A   14
#define ENC_B   13
#define ENC_KEY 27

// Tipos de callback para desacoplar
// el encoder de la lógica de menús
using EncoderCallback = std::function<void(int dir)>;
using ButtonCallback  = std::function<void()>;

class EncoderManager {
public:
    void begin();

    // Registra las funciones que se llaman
    // cuando hay giro o pulsación
    void onRotate(EncoderCallback cb);
    void onPress(ButtonCallback cb);

    // Debe llamarse en cada iteración del loop()
    void update();

private:
    // Callbacks guardados
    EncoderCallback _rotateCb;
    ButtonCallback  _pressCb;

    // Estado interno del encoder
    int  _lastAState   = HIGH;
    int  _dirHistory[2] = {0, 0};
    unsigned long _lastEncTime = 0;
    static const int ENC_DELAY = 2;

    // Estado interno del botón
    bool _lastReading     = HIGH;
    bool _stableState     = HIGH;
    unsigned long _lastBtnTime = 0;
    static const int BTN_DELAY = 50;

    void _readEncoder();
    void _readButton();
    void _resetHistory();
};