# Control térmico e instrumentación IR de una termoformadora de pequeña escala

Proyecto de grado – Ingeniería Electrónica, Universidad de los Andes  
**Autor:** Juan Sebastián Roncancio Ascencio · **Asesor:** Johann Faccelo Osma Cruz  
Grupo de investigación Biomicrosystems (BIOMS)

Optimización del sistema de control e instrumentación térmica de la termoformadora **HeatForm**
(C. D. Meza Poveda, 2026) para aplicaciones de electrónica en molde (IME).
La máquina original controla la temperatura de la **resistencia** con una termocupla; este proyecto
mide la **superficie de la lámina** con un sensor infrarrojo MLX90614 y la controla directamente (PI / PID).

## Estructura

| Carpeta | Contenido |
|---|---|
| `03_firmware_esp32/` | Firmware ESP32 (PlatformIO). `src/main.cpp`: control PI/PID sobre la lámina con el IR, protecciones, protocolo serial. `respaldo/`: versiones anteriores. |
| `04_matlab/ensayos/` | `ensayo_PID.m` (ensayo de control PID) y `termoformadoir.m` (termoformado guiado). |
| `04_matlab/ensayo_PI/` | `ensayo_PI.m` (mismo ensayo con el PI, para comparar). |
| `04_matlab/calibracion/` | `ensayo_calibracion.m`: IR vs termocupla de contacto (RMSE, emisividad, corrección). |
| `04_matlab/identificacion_y_pid_25oct/` | Identificación del modelo de la planta y diseño del controlador. |
| `04_matlab/identificacion_inicial/` | Primeras identificaciones (System Identification). |
| `05_datos_ensayos/` | Datos de todos los ensayos: `ENSAYOS PID/`, `ENSAYOS PI/`, `PID VS PI/`, `IDENTIFICACION 23oct/`, `resumen_ensayos.csv`, bitácora en `LEEME_ENSAYOS.txt`. |
| `06_soporte_mdf/` | Diseño del soporte del sensor IR (STEP y planos DXF para corte en MDF). |

Nombres de los ensayos: `PS_85C_E1_06nov_1740` = lámina PS, referencia 85 °C, repetición 1, 6-nov, 17:40.

## Hardware

- ESP32 (esp32dev) · termocupla tipo K + MAX6675 (CLK 18, DO 19, CS 5) · MLX90614 / GY-906 en I²C (SDA 21, SCL 22, 100 kHz)
- OLED SSD1306 (I²C 0x3C) · encoder · calefactor de cuarzo por PWM (pin 25, máximo 70 %)

## Protocolo serial (115200 baud)

| Comando | Acción |
|---|---|
| `R<t>` | Control de la lámina con el IR a `t` °C (30–180) |
| `K<kp>,<ki>,<kd>` | Ganancias del controlador |
| `P<n>` | PWM directo `n` % |
| `S` | Detener |

Salida cada 2 s: `DATA,ms,pwm,T_termocupla,T_ir_obj,T_ir_amb`. Líneas con `#` = estado o alarmas.

## Controladores (salida en % de PWM, Ts = 2 s)

| | Kp | Ki | Kd | Origen |
|---|---|---|---|---|
| PI | 0.639 | 0.00362 | 0 | Diseño en MATLAB (modelo de un polo) |
| PID | 1.3 | 0.010 | 10 (filtro Tf = 8 s) | Re-sintonizado con modelo de dos modos (6-nov-2026) |

## Créditos

El firmware parte del proyecto **HeatForm** de Cristihan David Meza Poveda (Universidad de los Andes, 2026),
distribuido con licencia MIT (ver `03_firmware_esp32/LICENSE_HeatForm_Meza`). Las librerías `lib/` y la interfaz web
original son de ese proyecto; las modificaciones de control, sensado IR, protecciones y los scripts y datos
de este repositorio son de J. S. Roncancio.
