#include "DisplayManager.h"
#include <Wire.h>

// ───────────────────────────────────────
// Constructor: tamaño + reset pin (-1)
// ───────────────────────────────────────
// No se puede inicializar Adafruit_SSD1306
// en el header porque necesita parámetros,
// así que se usa lista de inicialización
// en el .cpp a través de un constructor
// delegado implícito.
//
// Definimos el constructor aquí para tener
// acceso a las constantes de tamaño.

// Truco estándar en Arduino: el objeto
// display se construye con sus parámetros
// cuando se crea el DisplayManager.
// Como Adafruit_SSD1306 no tiene
// constructor vacío, declaramos _display
// en el .h y lo inicializamos aquí.

// Para que compile sin constructor
// explícito en el .h, usamos un hack
// habitual en firmware: redefinir el
// objeto en el .cpp vía placement.
// Lo más limpio en C++ moderno para
// Arduino es pasar los parámetros
// al constructor en el inicializador.
//
// Solución: añadir constructor explícito
// en el .cpp con lista de inicialización:


static const char* const MATERIAL_OPTIONS[] = {
    "PET", "PS", "MANUAL", "PWM"
};
static const char* const MAIN_OPTIONS[] = {
    "ANALOGO", "WIFI"
};

DisplayManager::DisplayManager()
    // Wire, reset = -1 (no reset externo)
    : _display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1)
{}

void DisplayManager::begin() {
    // Inicializar I2C explícitamente a 400 kHz
    // antes de que cualquier otra librería (SPI, WiFi)
    // haya podido alterar el estado del bus.
    Wire.begin();
    Wire.setClock(400000);

    // Reintentar hasta OLED_INIT_RETRIES veces con
    // reset del bus entre intentos, en lugar de
    // bloquear el sistema con while(1).
    const int     OLED_INIT_RETRIES = 5;
    const uint32_t OLED_RETRY_DELAY = 50; // ms
    bool ok = false;

    for (int attempt = 0; attempt < OLED_INIT_RETRIES && !ok; attempt++) {
        if (attempt > 0) {
            // Reset suave del bus I2C para recuperarse
            // de un estado bloqueado (SDA/SCL stuck low)
            Wire.end();
            delay(OLED_RETRY_DELAY);
            Wire.begin();
            Wire.setClock(400000);
        }
        ok = _display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    }

    if (!ok) {
        // OLED no responde tras todos los intentos.
        // En lugar de colgar, reportar por Serial
        // y continuar — el sistema sigue funcionando
        // (control, sensor, WiFi) aunque sin pantalla.
        Serial.println("[DISPLAY] ERROR: OLED SSD1306 no responde.");
        return;
    }

    _display.clearDisplay();
    _display.setTextColor(WHITE);
    Serial.println("[DISPLAY] OLED OK.");
}

// ───────────────────────────────────────
// Splash
// ───────────────────────────────────────
void DisplayManager::drawSplash() {
    _display.clearDisplay();

    _display.drawRect(34, 8, 60, 20, WHITE);
    _display.drawLine(34, 8,  94, 28, WHITE);
    _display.drawLine(94, 8,  34, 28, WHITE);

    _display.setTextSize(2);
    _display.setCursor(10, 38);
    _display.println("HeatForm");

    _display.display();
}

// ───────────────────────────────────────
// Menú principal
// ───────────────────────────────────────
void DisplayManager::drawMainMenu(int sel) {
    _display.clearDisplay();
    _display.setTextSize(1);

    _display.setCursor(30, 0);
    _display.println("HeatForm");
    _display.drawLine(0, 10, 128, 10, WHITE);

    for (int i = 0; i < TOTAL_MAIN_OPTIONS; i++) {
        if (i == sel) {
            _display.fillRect(10, 18 + i * 14, 108, 12, WHITE);
            _display.setTextColor(BLACK);
        } else {
            _display.setTextColor(WHITE);
        }
        _display.setCursor(20, 20 + i * 14);
        _display.println(MAIN_OPTIONS[i]);
    }

    _display.display();
}

// ───────────────────────────────────────
// Menú análogo (materiales)
// ───────────────────────────────────────
void DisplayManager::drawAnalogMenu(int sel) {
    _display.clearDisplay();
    _display.setTextSize(1);

    _display.setCursor(35, 0);
    _display.println("ANALOGO");
    _display.drawLine(0, 10, 128, 10, WHITE);

    for (int i = 0; i < TOTAL_ANALOG_OPTIONS; i++) {
        if (i == sel) {
            _display.fillRect(10, 14 + i * 10, 108, 9, WHITE);
            _display.setTextColor(BLACK);
        } else {
            _display.setTextColor(WHITE);
        }
        _display.setCursor(20, 15 + i * 10);
        _display.println(MATERIAL_OPTIONS[i]);
    }

    _display.display();
}

// ───────────────────────────────────────
// Ajuste manual de temperatura
// ───────────────────────────────────────
void DisplayManager::drawManualTemp(int temp) {
    _display.clearDisplay();
    _display.setTextColor(WHITE);
    _display.setTextSize(1);

    _display.setCursor(20, 0);
    _display.println("TEMP OBJ");
    _display.drawLine(0, 10, 128, 10, WHITE);

    // Número centrado
    _display.setTextSize(3);
    _display.setCursor(18, 18);
    _display.print(temp);
    _display.print("C");

    // Barra de progreso
    // Marco exterior
    _display.drawRect(4, 50, 120, 10, WHITE);

    // Relleno proporcional entre 30 y 250
    int fill = map(temp, 30, 250, 0, 116);
    _display.fillRect(6, 52, fill, 6, WHITE);
    _display.display();
}

// ───────────────────────────────────────
// Ajuste manual de tiempo
// ───────────────────────────────────────
void DisplayManager::drawManualTime(int seconds) {
    _display.clearDisplay();
    _display.setTextColor(WHITE);
    _display.setTextSize(1);

    _display.setCursor(30, 0);
    _display.println("TIEMPO");
    _display.drawLine(0, 10, 128, 10, WHITE);

    // Número centrado
    _display.setTextSize(3);
    _display.setCursor(18, 18);
    _display.print(seconds);
    _display.print("s");

    // Barra de progreso
    // Marco exterior
    _display.drawRect(4, 50, 120, 10, WHITE);

    // Relleno proporcional entre 10 y 420
    int fill = map(seconds, 10, 420, 0, 116);
    _display.fillRect(6, 52, fill, 6, WHITE);

    _display.display();
}

// ───────────────────────────────────────
// Pantalla WiFi
// ───────────────────────────────────────
void DisplayManager::drawWifiScreen() {
    _display.clearDisplay();
    _display.setTextSize(1);
    _display.setTextColor(WHITE);
    _display.setCursor(20, 0);
    _display.println("WiFi Activo");
    _display.drawLine(0, 10, 128, 10, WHITE);
    _display.setCursor(0, 16);
    _display.println("SSID: HeatForm");
    _display.setCursor(0, 28);
    _display.println("Pass: 12345678");
    _display.setCursor(0, 40);
    _display.println("IP: 192.168.4.1");
    _display.setCursor(0, 54);
    _display.println("[BTN] para salir");
    _display.display();
    // delay() eliminado — nunca bloquear el bus I2C
}

// ───────────────────────────────────────
// Pantalla de proceso activo
// ───────────────────────────────────────
void DisplayManager::drawProcessScreen(
    const char*   material,
    float         currentTemp,
    float         setpoint,
    float         outputPct,
    unsigned long remainingSecs
) {
    _display.clearDisplay();
    _display.setTextSize(1);

    // Fila superior: material y PWM
    _display.setCursor(0, 0);
    _display.print(material);

    _display.setCursor(80, 0);
    _display.print("PWM ");
    _display.print((int)outputPct);
    _display.print("%");

    _display.drawLine(0, 10, 128, 10, WHITE);

    // Temperatura actual
    _display.setCursor(0, 18);
    _display.print("TMP ");
    _display.print(currentTemp, 1);
    _display.print("C");

    // Setpoint
    _display.setCursor(0, 32);
    _display.print("OBJ ");
    _display.print(setpoint, 0);
    _display.print("C");

    // Tiempo restante (grande)
    _display.setTextSize(2);
    _display.setCursor(20, 48);
    _display.print(remainingSecs);
    _display.print("s");

    _display.display();
}

// ───────────────────────────────────────
// PWM Directo — selección de porcentaje
// ───────────────────────────────────────
void DisplayManager::drawPwmPct(int pct) {
    _display.clearDisplay();
    _display.setTextColor(WHITE);
    _display.setTextSize(1);

    _display.setCursor(18, 0);
    _display.println("PWM DIRECTO");
    _display.drawLine(0, 10, 128, 10, WHITE);

    // Porcentaje centrado grande
    _display.setTextSize(3);
    _display.setCursor(18, 18);
    _display.print(pct);
    _display.print("%");

    // Barra de progreso 0-100%
    _display.drawRect(4, 50, 120, 10, WHITE);
    int fill = map(pct, 0, 100, 0, 116);
    if (fill > 0) _display.fillRect(6, 52, fill, 6, WHITE);

    _display.display();
}

// ───────────────────────────────────────
// PWM Directo — pantalla de proceso activo
// ───────────────────────────────────────
void DisplayManager::drawPwmDirectScreen(
    float         temp,
    float         pwmPct,
    unsigned long remainingSecs
) {
    _display.clearDisplay();
    _display.setTextSize(1);

    // Fila superior: modo + valor PWM fijo
    _display.setCursor(0, 0);
    _display.print("PWM DIR");
    _display.setCursor(72, 0);
    _display.print("PWM:");
    _display.print((int)pwmPct);
    _display.print("%");

    _display.drawLine(0, 10, 128, 10, WHITE);

    // Temperatura medida
    _display.setCursor(0, 14);
    _display.print("TEMP: ");
    _display.print(temp, 1);
    _display.print("C");

    // Barra de temperatura (0-300°C)
    _display.drawRect(0, 26, 128, 6, WHITE);
    int tempFill = (int)map((long)temp, 0, 300, 0, 124);
    tempFill = constrain(tempFill, 0, 124);
    if (tempFill > 0) _display.fillRect(2, 28, tempFill, 2, WHITE);

    // Tiempo restante (grande)
    _display.setTextSize(2);
    _display.setCursor(4, 38);
    _display.print(remainingSecs);
    _display.print("s restante");

    _display.display();
}