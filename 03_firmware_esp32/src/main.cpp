/*
 * HeatForm
 * main.cpp — punto de entrada del firmware
 * + Sensor IR MLX90614 y comandos por serial (P<n>, S) para registro de datos
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>

// ── Módulos propios ─────────────────────
#include <DisplayManager.h>
#include <EncoderManager.h>
#include <TemperatureSensor.h>
#include <PWMManager.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>

// ═══════════════════════════════════════
// WIFI — Access Point
// ═══════════════════════════════════════
const char* AP_SSID     = "HeatForm";
const char* AP_PASSWORD = "12345678";

IPAddress ap_ip(192, 168, 4, 1);
IPAddress ap_gateway(192, 168, 4, 1);
IPAddress ap_subnet(255, 255, 255, 0);

AsyncWebServer server(80);
bool wifiActive = false;   // WiFi apagado por defecto

// ═══════════════════════════════════════
// Utilidades
// ═══════════════════════════════════════
static const char* const MATERIAL_OPTIONS[] = {
    "PET", "PS", "MANUAL", "PWM"
};

// ═══════════════════════════════════════
// OBJETOS GLOBALES
// ═══════════════════════════════════════
DisplayManager    display;
EncoderManager    encoder;
TemperatureSensor sensor;
PWMManager        pwm;

// ── Sensor IR MLX90614 (I2C: SDA=21, SCL=22, mismo bus de la OLED) ──
Adafruit_MLX90614 mlx;
bool  mlxOk  = false;
float irObj  = NAN;     // temperatura de la lamina (objeto)
float irAmb  = NAN;     // temperatura del chip (ambiente)
const float T_MAX_SEGURIDAD = 200.0f;   // corta el PWM si la termocupla pasa esto
const unsigned long TS_LOG_MS = 500;    // periodo de registro en modo PWM directo (>=250 ms por el MAX6675)

// ── Control PI sobre la lamina (sensor IR) ──
// Disenado en MATLAB con G_ir = 277.7/(176.7 s + 1) e^(-26 s), margen de fase 75 grados.
// Simulacion: sin sobrepico, asentamiento ~294 s, PWM max ~56 %.
// PID v2 (6-nov): con PI (0.639 / 0.00362) a 100 °C tardaba >7 min en el tramo final.
// PID simulado con 3 modelos: asentamiento 210-270 s, sobrepico <= 1 %.
float Kp_ir = 1.0f;          // % por °C
float Ki_ir = 0.0072f;       // % por (°C·s)
float Kd_ir = 10.0f;         // % por (°C/s)   (Kd = 0 -> PI)
float Tf_ir = 8.0f;          // s, filtro de la derivada
float d_ir  = 0.0f;          // derivada filtrada
float ir_prev = NAN;
bool  irMode        = false; // true = control PI con el IR
float irSetpoint    = 100.0f;
float irFilt        = NAN;   // IR filtrado (mediana de 3)
float irBuf[3]      = {NAN, NAN, NAN};
int   irBufN        = 0;
float integral_ir   = 0.0f;
float error_ir_prev = 0.0f;
const float IR_MARGEN_HUECO = 25.0f;   // si la lamina pasa SP + esto -> posible hueco: se apaga

// ═══════════════════════════════════════
// PIDF — parámetros
// ═══════════════════════════════════════
float Kp  = 0.78f;
float Ki  = 0.008f;
float Kd  = 5.0f;
float Tf  = 4.2899f;
float Ts  = 2.0f;
float Kff = 0.12f;

float OUT_MIN = 0.0f;
float OUT_MAX = 70.0f;

// ── Estado interno del PIDF ─────────────
float integral   = 0.0f;
float error_prev = 0.0f;
float derivative = 0.0f;
float d_prev     = 0.0f;

// ═══════════════════════════════════════
// ESTADO DEL PROCESO
// ═══════════════════════════════════════
float    setpoint        = 145.0f;
float    output_pct      = 0.0f;
float    currentTemp     = 25.0f;
bool     running         = false;
String   currentMaterial = "PET";

unsigned long startMs           = 0;
unsigned long lastSampleMs      = 0;
unsigned long processDuration   = 120;
unsigned long lastDisplayUpdate = 0;

// ═══════════════════════════════════════
// COMANDO WEB PENDIENTE
// El handler de AsyncWebServer corre en otra tarea.
// Se usa un flag + parámetros: la tarea web los
// escribe y el loop() los consume de forma segura.
// ═══════════════════════════════════════
struct WebCmd {
    volatile bool pending  = false;
    bool          isPwm    = false;
    float         temp     = 145.0f;
    unsigned long duration = 120;
    float         pwmPct   = 50.0f;
    bool          isIr     = false;
    float         kp       = NAN;
    float         ki       = NAN;
    float         kd       = NAN;
} pendingCmd;

String lastAlarm = "";   // ultimo motivo de parada por seguridad (se muestra en la web)

// ═══════════════════════════════════════
// ESTADO DE MENÚS
// ═══════════════════════════════════════
enum ScreenState {
    SPLASH,
    MAIN_MENU,
    ANALOG_MENU,
    PROCESS_SCREEN,
    MANUAL_TEMP,
    MANUAL_TIME,
    PWM_PCT,        // Selección % PWM para modo Directo
    WIFI_SCREEN
};

ScreenState currentScreen     = SPLASH;
int         currentMainOption = 0;
int         currentAnalogOpt  = 0;

int  manualTemp     = 145;
int  manualTime     = 120;
int  manualPwm      = 50;   // % para modo PWM Directo
bool pwmDirectMode  = false; // true = PWM fijo, false = PIDF

// ═══════════════════════════════════════
// PIDF — implementación
// ═══════════════════════════════════════
float calcularFeedforward(float error) {
    if (error < 5.0f) return 0.0f;
    float base   = Kff * (setpoint - 25.0f);
    float factor = constrain(error / 40.0f, 0.0f, 1.0f);
    return base * factor;
}

float calcularPIDF(float temp_actual) {
    float error = setpoint - temp_actual;

    float integral_nueva =
        integral + Ki * (Ts / 2.0f) * (error + error_prev);
    integral_nueva = constrain(integral_nueva, OUT_MIN, OUT_MAX);

    float deriv_med = -(temp_actual - (setpoint - error_prev)) / Ts;
    float alpha     = Tf / (Tf + Ts);
    derivative      = alpha * d_prev + (1.0f - alpha) * deriv_med;

    float D  = Kd * derivative;
    float ff = calcularFeedforward(error);

    float output_tent = Kp * error + integral_nueva + D + ff;

    bool sat_alto = (output_tent >= OUT_MAX) && (error > 0.0f);
    bool sat_bajo = (output_tent <= OUT_MIN) && (error < 0.0f);

    if (!sat_alto && !sat_bajo) integral = integral_nueva;

    error_prev = error;
    d_prev     = derivative;

    float output = Kp * error + integral + D + ff;
    return constrain(output, OUT_MIN, OUT_MAX);
}

// ═══════════════════════════════════════
// PROCESO
// ═══════════════════════════════════════
void startProcess() {
    irMode     = false;
    running    = true;
    integral   = 0;
    error_prev = 0;
    derivative = 0;
    d_prev     = 0;

    startMs      = millis();
    lastSampleMs = millis() - (unsigned long)(Ts * 1000);

    currentScreen = PROCESS_SCREEN;
    display.drawProcessScreen(
        currentMaterial.c_str(), currentTemp, setpoint, 0, processDuration
    );
}

// ── Modo PWM Directo ─────────────────────────
// Aplica un PWM fijo sin PIDF y registra temp.
void startPwmDirectProcess() {
    irMode        = false;
    running       = true;
    output_pct    = (float)manualPwm;

    startMs      = millis();
    lastSampleMs = millis() - (unsigned long)(Ts * 1000);

    pwm.writePctDirect(output_pct); // Sin cap: 100% → duty 1023

    currentScreen = PROCESS_SCREEN;
    display.drawPwmDirectScreen(currentTemp, output_pct, processDuration);

    Serial.printf("[PWM DIR] Inicio: PWM=%.0f%% Tiempo=%lus\n",
                  output_pct, processDuration);
}

void stopProcess() {
    running    = false;
    output_pct = 0;
    pwm.off();
}

// ── Mediana de las 3 ultimas lecturas del IR (quita los bajones aislados) ──
float filtrarIR(float x) {
    if (isnan(x)) return irFilt;
    irBuf[irBufN % 3] = x; irBufN++;
    if (irBufN < 3) return x;
    float a = irBuf[0], b = irBuf[1], c = irBuf[2];
    return max(min(a, b), min(max(a, b), c));
}

// ── PI con anti-windup (misma forma que calcularPIDF) ──
float calcularPI_IR(float temp_lamina) {
    float error = irSetpoint - temp_lamina;
    float integral_nueva = integral_ir + Ki_ir * (Ts / 2.0f) * (error + error_ir_prev);
    integral_nueva = constrain(integral_nueva, OUT_MIN, OUT_MAX);
    // Derivada sobre la medicion (no salta al cambiar la referencia), filtrada
    float dm = isnan(ir_prev) ? 0.0f : -(temp_lamina - ir_prev) / Ts;
    float alpha = Tf_ir / (Tf_ir + Ts);
    d_ir = alpha * d_ir + (1.0f - alpha) * dm;
    ir_prev = temp_lamina;
    float D = Kd_ir * d_ir;
    float tent = Kp_ir * error + integral_nueva + D;
    bool sat_alto = (tent >= OUT_MAX) && (error > 0.0f);
    bool sat_bajo = (tent <= OUT_MIN) && (error < 0.0f);
    if (!sat_alto && !sat_bajo) integral_ir = integral_nueva;
    error_ir_prev = error;
    return constrain(Kp_ir * error + integral_ir + D, OUT_MIN, OUT_MAX);
}

void startIrProcess(float sp, unsigned long dur = 3600) {
    sp              = constrain(sp, 30.0f, 180.0f);
    lastAlarm       = "";
    irSetpoint      = sp;
    irMode          = true;
    pwmDirectMode   = false;
    currentMaterial = "IR";
    setpoint        = sp;
    integral_ir     = 0.0f;
    error_ir_prev   = 0.0f;
    d_ir            = 0.0f;
    ir_prev         = NAN;
    irBufN          = 0;
    processDuration = dur;
    running         = true;
    startMs         = millis();
    lastSampleMs    = millis() - (unsigned long)(Ts * 1000);
    if (currentScreen != WIFI_SCREEN) currentScreen = PROCESS_SCREEN;
    Serial.printf("#CMD IR SP=%.1f Kp=%.3f Ki=%.5f Kd=%.2f\n", irSetpoint, Kp_ir, Ki_ir, Kd_ir);
}

// ── Comandos por serial (desde MATLAB o el monitor serial) ──
//   P<n>  -> PWM directo al n %   (ej: P40, P70, P0)
//   R<t>  -> control PID de la lamina a t °C (ej: R100)
//   K<kp>,<ki>,<kd> -> cambia ganancias (ej: K1.0,0.0072,10  o  K0.639,0.00362,0 para PI)
//   S     -> parar
void handleSerial() {
    if (!Serial.available()) return;
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() == 0) return;

    if (cmd[0] == 'K' || cmd[0] == 'k') {
        float kp, ki, kd;
        if (sscanf(cmd.c_str() + 1, "%f,%f,%f", &kp, &ki, &kd) == 3 && kp > 0 && ki >= 0 && kd >= 0) {
            Kp_ir = kp; Ki_ir = ki; Kd_ir = kd;
            Serial.printf("#CMD K Kp=%.4f Ki=%.5f Kd=%.3f\n", Kp_ir, Ki_ir, Kd_ir);
        } else {
            Serial.println("#ERROR: formato K<kp>,<ki>,<kd>");
        }
    } else if (cmd[0] == 'R' || cmd[0] == 'r') {
        float sp = constrain(cmd.substring(1).toFloat(), 30.0f, 180.0f);
        if (!mlxOk) { Serial.println("#ERROR: sensor IR no responde, no se inicia el control"); return; }
        startIrProcess(sp);
    } else if (cmd[0] == 'P' || cmd[0] == 'p') {
        irMode = false;
        int pct = constrain(cmd.substring(1).toInt(), 0, 100);
        manualPwm       = pct;
        pwmDirectMode   = true;
        currentMaterial = "PWM";
        if (!running) {
            processDuration = 3600;        // 1 h; MATLAB decide cuando parar
            startPwmDirectProcess();
        } else {
            output_pct = (float)pct;
            pwm.writePctDirect(output_pct);
        }
        Serial.printf("#CMD PWM=%d\n", pct);
    } else if (cmd[0] == 'S' || cmd[0] == 's') {
        stopProcess();
        Serial.println("#CMD STOP");
    }
}

// ── Comando recibido desde la página web ────────────
// Se llama desde loop() en el contexto correcto.
// No llama a display si el WiFi está activo.
void processPendingWebCmd() {
    irMode = false;
    if (pendingCmd.isIr) {
        if (!isnan(pendingCmd.kp) && pendingCmd.kp > 0) Kp_ir = pendingCmd.kp;
        if (!isnan(pendingCmd.ki) && pendingCmd.ki >= 0) Ki_ir = pendingCmd.ki;
        if (!isnan(pendingCmd.kd) && pendingCmd.kd >= 0) Kd_ir = pendingCmd.kd;
        if (!mlxOk) { Serial.println("#ERROR: sensor IR no responde"); lastAlarm = "Sensor IR no responde"; return; }
        startIrProcess(pendingCmd.temp, pendingCmd.duration);
        return;
    }
    if (pendingCmd.isPwm) {
        // ── Modo PWM Directo ──
        currentMaterial = "PWM";
        manualPwm       = constrain((int)pendingCmd.pwmPct, 0, 100);
        processDuration = pendingCmd.duration;
        pwmDirectMode   = true;

        running      = true;
        output_pct   = (float)manualPwm;
        startMs      = millis();
        lastSampleMs = millis() - (unsigned long)(Ts * 1000);

        pwm.writePctDirect(output_pct);

        if (currentScreen != WIFI_SCREEN) {
            currentScreen = PROCESS_SCREEN;
            display.drawPwmDirectScreen(currentTemp, output_pct, processDuration);
        }
        Serial.printf("[WEB] PWM Directo: %.0f%% por %lus\n",
                      output_pct, pendingCmd.duration);
    } else {
        // ── Modo PIDF ──
        currentMaterial = "WEB";
        setpoint        = pendingCmd.temp;
        processDuration = pendingCmd.duration;
        pwmDirectMode   = false;

        integral   = 0;
        error_prev = 0;
        derivative = 0;
        d_prev     = 0;
        running    = true;
        startMs      = millis();
        lastSampleMs = millis() - (unsigned long)(Ts * 1000);

        if (currentScreen != WIFI_SCREEN) {
            currentScreen = PROCESS_SCREEN;
            display.drawProcessScreen(
                currentMaterial.c_str(), currentTemp, setpoint, 0, processDuration
            );
        }
        Serial.printf("[WEB] PIDF: SP=%.1f por %lus\n",
                      setpoint, pendingCmd.duration);
    }
}

// ═══════════════════════════════════════
// WIFI — arranque y apagado bajo demanda
// ═══════════════════════════════════════
void setupRoutes() {
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
        req->send(SPIFFS, "/index.html", "text/html");
    });

    server.on("/chart.js", HTTP_GET, [](AsyncWebServerRequest* req) {
        req->send(SPIFFS, "/chart.js", "application/javascript");
    });

    server.on("/data", HTTP_GET, [](AsyncWebServerRequest* req) {
        long timer = running
            ? max(0L, (long)(processDuration - (millis() - startMs) / 1000))
            : 0;
        auto num = [](float v, int d) { return isnan(v) ? String("null") : String(v, d); };
        float tCtrl = irMode ? irFilt : currentTemp;            // variable controlada
        String modo = irMode ? "IR" : (pwmDirectMode ? "PWM" : "PIDF");
        float integ = irMode ? integral_ir : integral;

        String json = "{";
        json += "\"temp\":"       + num(tCtrl, 1)                     + ",";
        json += "\"tc\":"         + num(currentTemp, 1)               + ",";
        json += "\"ir\":"         + num(irFilt, 1)                    + ",";
        json += "\"ir_amb\":"     + num(irAmb, 1)                     + ",";
        json += "\"target\":"     + String(setpoint, 1)               + ",";
        json += "\"pwm\":"        + String(output_pct, 1)             + ",";
        json += "\"error\":"      + num(setpoint - tCtrl, 1)          + ",";
        json += "\"integral\":"   + String(integ, 2)                  + ",";
        json += "\"derivative\":" + String(irMode ? Kd_ir * d_ir : derivative, 1) + ",";
        json += "\"heater\":"     + String(output_pct, 1)             + ",";
        json += "\"fan\":0,\"vacuum\":false,";
        json += "\"timer\":"      + String(timer)                     + ",";
        json += "\"state\":\""    + String(running ? "EN PROCESO" : "DETENIDO") + "\",";
        json += "\"mode\":\""     + modo                              + "\",";
        json += "\"kp\":"         + String(Kp_ir, 4)                  + ",";
        json += "\"ki\":"         + String(Ki_ir, 5)                  + ",";
        json += "\"kd\":"         + String(Kd_ir, 3)                  + ",";
        json += "\"mlx\":"        + String(mlxOk ? "true" : "false")  + ",";
        json += "\"alarm\":\""    + lastAlarm                         + "\",";
        json += "\"material\":\"" + currentMaterial                   + "\"";
        json += "}";
        req->send(200, "application/json", json);
    });

    server.on("/set", HTTP_POST, [](AsyncWebServerRequest* req) {
        if (req->hasParam("cmd", true)) {
            String cmd = req->getParam("cmd", true)->value();
            if (cmd == "stop") stopProcess();
        }
        req->send(200, "application/json", "{\"ok\":true}");
    });

    // POST /start — inicia ciclo desde la web
    // Parámetros (form-urlencoded):
    //   mode : "PWM" | "PI" | "PID" | "PIDF_FF"
    //   temp : temperatura objetivo (°C, para modos PIDF)
    //   time : duración en segundos
    //   pwm  : porcentaje 0-100 (solo modo PWM)
    server.on("/start", HTTP_POST, [](AsyncWebServerRequest* req) {
        String mode = req->hasParam("mode", true)
                        ? req->getParam("mode", true)->value() : "PIDF_FF";
        float  temp = req->hasParam("temp", true)
                        ? req->getParam("temp", true)->value().toFloat() : 145.0f;
        int    t    = req->hasParam("time", true)
                        ? req->getParam("time", true)->value().toInt()   : 120;
        float  wpct = req->hasParam("pwm",  true)
                        ? req->getParam("pwm",  true)->value().toFloat() : 50.0f;

        pendingCmd.isPwm    = (mode == "PWM");
        pendingCmd.isIr     = (mode == "IR");
        pendingCmd.kp = req->hasParam("kp", true) ? req->getParam("kp", true)->value().toFloat() : NAN;
        pendingCmd.ki = req->hasParam("ki", true) ? req->getParam("ki", true)->value().toFloat() : NAN;
        pendingCmd.kd = req->hasParam("kd", true) ? req->getParam("kd", true)->value().toFloat() : NAN;
        pendingCmd.temp     = constrain(temp, 0.0f, 300.0f);
        pendingCmd.duration = constrain(t, 1, 3600);
        pendingCmd.pwmPct   = constrain(wpct, 0.0f, 100.0f);
        pendingCmd.pending  = true;  // el loop() lo ejecuta

        req->send(200, "application/json", "{\"ok\":true}");
    });

    server.onNotFound([](AsyncWebServerRequest* req) {
        req->send(404, "text/plain", "No encontrado");
    });
}

void startWifi() {
    if (wifiActive) return;

    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(ap_ip, ap_gateway, ap_subnet);
    WiFi.softAP(AP_SSID, AP_PASSWORD);

    setupRoutes();
    server.begin();

    wifiActive = true;

    Serial.print("AP activo | SSID: ");
    Serial.print(AP_SSID);
    Serial.print(" | IP: ");
    Serial.println(WiFi.softAPIP());
}

void stopWifi() {
    if (!wifiActive) return;

    server.end();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);

    wifiActive = false;

    Serial.println("WiFi apagado");
}

// ═══════════════════════════════════════
// SELECCIÓN DE MATERIAL
// ═══════════════════════════════════════
void applyMaterial(const char* name) {
    currentMaterial = name;

    if      (strcmp(name, "PET") == 0) { setpoint = 145; processDuration = 120; }
    else if (strcmp(name, "PS")  == 0) { setpoint = 135; processDuration = 100; }
    else if (strcmp(name, "PLA") == 0) { setpoint = 160; processDuration = 110; }
    else if (strcmp(name, "ABS") == 0) { setpoint = 180; processDuration = 140; }
}

// ═══════════════════════════════════════
// CALLBACKS DEL ENCODER
// ═══════════════════════════════════════
void onEncoderRotate(int dir) {
    switch (currentScreen) {

        case MAIN_MENU:
            currentMainOption = (currentMainOption + dir + TOTAL_MAIN_OPTIONS)
                                % TOTAL_MAIN_OPTIONS;
            display.drawMainMenu(currentMainOption);
            break;

        case ANALOG_MENU:
            currentAnalogOpt = (currentAnalogOpt + dir + TOTAL_ANALOG_OPTIONS)
                               % TOTAL_ANALOG_OPTIONS;
            display.drawAnalogMenu(currentAnalogOpt);
            break;

        case MANUAL_TEMP:
            manualTemp = constrain(manualTemp + dir * 5, 30, 250);
            display.drawManualTemp(manualTemp);
            break;

        case MANUAL_TIME:
            manualTime = constrain(manualTime + dir * 10, 10, 420);
            display.drawManualTime(manualTime);
            break;

        case PWM_PCT:
            manualPwm = constrain(manualPwm + dir * 5, 0, 100);
            display.drawPwmPct(manualPwm);
            break;

        case WIFI_SCREEN:
            // Sin navegación mientras WiFi está activo
            break;

        default:
            break;
    }
}

void onButtonPress() {
    switch (currentScreen) {

        case MAIN_MENU:
            if (currentMainOption == 0) {
                // → Menú analógico
                currentScreen = ANALOG_MENU;
                display.drawAnalogMenu(currentAnalogOpt);
            } else {
                // → Pantalla WiFi: arranca AP y muestra info
                startWifi();
                currentScreen = WIFI_SCREEN;
                display.drawWifiScreen();
            }
            break;

        case ANALOG_MENU: {
            const char* selected = MATERIAL_OPTIONS[currentAnalogOpt];
            if (strcmp(selected, "MANUAL") == 0) {
                pwmDirectMode = false;
                currentScreen = MANUAL_TEMP;
                display.drawManualTemp(manualTemp);
            } else if (strcmp(selected, "PWM") == 0) {
                // → Modo PWM Directo: elegir % primero
                pwmDirectMode = true;
                currentScreen = PWM_PCT;
                display.drawPwmPct(manualPwm);
            } else {
                pwmDirectMode = false;
                applyMaterial(selected);
                startProcess();
            }
            break;
        }

        case MANUAL_TEMP:
            currentScreen = MANUAL_TIME;
            display.drawManualTime(manualTime);
            break;

        case PWM_PCT:
            // Confirmar % → seleccionar tiempo
            currentScreen = MANUAL_TIME;
            display.drawManualTime(manualTime);
            break;

        case MANUAL_TIME:
            processDuration = manualTime;
            if (pwmDirectMode) {
                currentMaterial = "PWM";
                startPwmDirectProcess();
            } else {
                currentMaterial = "MANUAL";
                setpoint        = manualTemp;
                startProcess();
            }
            break;

        case PROCESS_SCREEN:
            stopProcess();
            currentScreen = ANALOG_MENU;
            display.drawAnalogMenu(currentAnalogOpt);
            break;

        case WIFI_SCREEN:
            // Botón en pantalla WiFi → apaga AP y vuelve al menú
            stopWifi();
            currentScreen = MAIN_MENU;
            display.drawMainMenu(currentMainOption);
            break;

        default:
            break;
    }
}

// ═══════════════════════════════════════
// SETUP
// ═══════════════════════════════════════
void setup() {
    Serial.begin(115200);
    Serial.println("[HeatForm] Iniciando...");

    // ── 1. Apagar WiFi PRIMERO ──────────────────────────
    // El stack WiFi puede generar transitorios en la
    // alimentación y ruido en el bus al activarse/apagarse.
    // Silenciarlo antes de cualquier periférico I2C/SPI
    // elimina interferencias durante el boot.
    WiFi.mode(WIFI_OFF);
    delay(100); // Esperar a que los rieles se estabilicen

    // ── 2. PWM apagado (salida a 0 antes de arrancar) ──
    pwm.begin();

    // ── 3. OLED — inicializar con bus I2C estable ───────
    display.begin();
    display.drawSplash();

    // ── 3b. Sensor IR (mismo bus I2C que la OLED) ──
    // El MLX90614 (SMBus) admite maximo 100 kHz; DisplayManager deja el bus en 400 kHz.
    Wire.setClock(100000);
    mlxOk = mlx.begin();
    if (mlxOk) {
        Serial.printf("#MLX OK  emisividad=%.3f  obj=%.2f  amb=%.2f\n",
                      mlx.readEmissivity(), mlx.readObjectTempC(),
                      mlx.readAmbientTempC());
    } else {
        Serial.println("#MLX NO RESPONDE (revisar soldadura / 0x5A)");
    }

    // ── 4. Sensor SPI ──────────────────────────────────
    // Se inicializa después del OLED para no contaminar
    // el bus I2C con actividad SPI concurrente durante begin()
    sensor.begin();

    // ── 5. SPIFFS ──────────────────────────────────────
    if (!SPIFFS.begin(true)) {
        Serial.println("[HeatForm] Error al montar SPIFFS");
    }

    // ── 6. Encoder (último: evita callbacks durante init) ──
    encoder.begin();
    encoder.onRotate(onEncoderRotate);
    encoder.onPress(onButtonPress);

    // Splash + menú inicial
    delay(2000);
    currentScreen = MAIN_MENU;
    display.drawMainMenu(currentMainOption);
}

// ═══════════════════════════════════════
// LOOP
// ═══════════════════════════════════════
void loop() {
    handleSerial();
    encoder.update();

    // ── Procesar comando web pendiente ───────────────────────
    // Se comprueba antes del return de WiFi para que
    // el ciclo arranque aunque el OLED muestre WiFi.
    if (pendingCmd.pending) {
        pendingCmd.pending = false;
        processPendingWebCmd();
    }

    // En pantalla WiFi solo continuamos si hay proceso
    // activo (iniciado desde la web). Sin proceso, sale.
    if (currentScreen == WIFI_SCREEN && !running) return;

    if (!running) return;

    unsigned long now     = millis();
    unsigned long elapsed = (now - startMs) / 1000;

    // Fin del proceso
    if (elapsed >= processDuration) {
        stopProcess();
        // Solo cambiar pantalla OLED si no está en modo WiFi
        if (currentScreen != WIFI_SCREEN) {
            currentScreen = ANALOG_MENU;
            display.drawAnalogMenu(currentAnalogOpt);
        }
        return;
    }

    // Muestreo cada Ts segundos — termocupla + IR
    unsigned long periodo = pwmDirectMode ? TS_LOG_MS : (unsigned long)(Ts * 1000);
    if (now - lastSampleMs >= periodo) {
        lastSampleMs = now;

        currentTemp = sensor.readCelsius();
        if (mlxOk) {
            irObj = mlx.readObjectTempC();
            irAmb = mlx.readAmbientTempC();
            irFilt = filtrarIR(irObj);
        }

        if (pwmDirectMode) {
            output_pct = (float)manualPwm;
            pwm.writePctDirect(output_pct);
        } else if (irMode) {
            if (isnan(irFilt)) { output_pct = 0.0f; }
            else               { output_pct = calcularPI_IR(irFilt); }   // ya limitado a 0..OUT_MAX
            pwm.writePctDirect(output_pct);   // misma escala que en la identificacion (PWM directo)
        } else {
            output_pct = calcularPIDF(currentTemp);
            pwm.writePct(output_pct);
        }

        // Seguridad: sobretemperatura o termocupla sin lectura (3 nan seguidos)
        static int nanSeguidos = 0;
        nanSeguidos = isnan(currentTemp) ? nanSeguidos + 1 : 0;
        if (currentTemp > T_MAX_SEGURIDAD) {
            stopProcess();
            Serial.println("#SEGURIDAD: termocupla > T_MAX, PWM apagado");
            lastAlarm = "Termocupla > 200 C";
        } else if (irMode && !isnan(irFilt) && irFilt > irSetpoint + IR_MARGEN_HUECO) {
            stopProcess();
            Serial.println("#SEGURIDAD: lamina muy por encima de la referencia (posible hueco), PWM apagado");
            lastAlarm = "Lamina muy por encima de la referencia (posible hueco)";
        } else if (nanSeguidos >= 3) {
            stopProcess();
            nanSeguidos = 0;
            Serial.println("#SEGURIDAD: termocupla sin lectura (nan), PWM apagado");
            lastAlarm = "Termocupla sin lectura";
        }

        // Linea de datos para MATLAB: DATA,ms,pwm,termocupla,ir_obj,ir_amb
        Serial.printf("DATA,%lu,%.1f,%.2f,%.2f,%.2f\n",
                      now, output_pct, currentTemp, irObj, irAmb);
    }

    // Refresca OLED cada 500 ms
    // No actualizar si el OLED está mostrando la pantalla WiFi
    if (currentScreen != WIFI_SCREEN && now - lastDisplayUpdate > 500) {
        lastDisplayUpdate = now;

        unsigned long remaining = processDuration - elapsed;

        if (pwmDirectMode) {
            display.drawPwmDirectScreen(currentTemp, output_pct, remaining);
        } else {
            display.drawProcessScreen(
                currentMaterial.c_str(),
                irMode ? irFilt : currentTemp,
                setpoint,
                output_pct,
                remaining
            );
        }
    }
}