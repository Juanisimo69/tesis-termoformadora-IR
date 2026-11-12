"""
CALIBRACION DEL SENSOR IR CONTRA TERMOCUPLA DE CONTACTO (version Python, sin MATLAB)

Uso:   python ensayo_calibracion.py COM5
       (cambia COM5 por tu puerto; si no sabes cual es:  python -m serial.tools.list_ports)

Requisitos (una sola vez):  pip install pyserial matplotlib

- El control lo hace el ESP32 con el IR (PID). La lamina pasa por escalones de temperatura.
- Al final de cada escalon suena un pitido y la consola te pide escribir lo que marca el multimetro
  y el termometro laser (punto del IR, cinta, centro). Enter vacio = no lo mediste.
- Ctrl+C en cualquier momento: apaga el cuarzo y guarda todo.
- Guarda en 05_datos_ensayos/CALIBRACION IR/<MATERIAL>_calibracion_<fecha>/
"""
import sys, os, time, csv, math, datetime
import serial

MATERIAL  = "PS"                      # PS = lamina BLANCA | PET = lamina TRANSPARENTE
ESCALONES = [45, 60, 75, 85] if MATERIAL == "PS" else [40, 50, 60, 65]
T_PASO    = 300                       # s por escalon
T_AVISO   = 30                        # s antes de terminar el escalon se piden las lecturas
ENFRIAR   = 120                       # s registrando con el cuarzo apagado
GANANCIAS = (1.3, 0.010, 10)          # PID

def beep():
    try:
        import winsound; winsound.Beep(1200, 300); winsound.Beep(1200, 300)
    except Exception:
        print("\a", end="", flush=True)

def leer_num(txt):
    v = input(txt).strip().replace(",", ".")
    try: return float(v)
    except ValueError: return float("nan")

def main():
    if len(sys.argv) < 2:
        print("Uso: python ensayo_calibracion.py COM5"); return
    puerto = sys.argv[1]
    raiz = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    meses = ["ene","feb","mar","abr","may","jun","jul","ago","sep","oct","nov","dic"]
    ahora = datetime.datetime.now()
    nombre = f"{MATERIAL}_calibracion_{ahora.day:02d}{meses[ahora.month-1]}_{ahora.hour:02d}{ahora.minute:02d}"
    carpeta = os.path.join(raiz, "05_datos_ensayos", "CALIBRACION IR", nombre)
    os.makedirs(carpeta, exist_ok=True)
    base = os.path.join(carpeta, nombre)
    log = open(os.path.join(carpeta, "consola.txt"), "w", encoding="utf-8")
    def p(*a):
        s = " ".join(str(x) for x in a); print(s); log.write(s + "\n"); log.flush()

    s = serial.Serial(puerto, 115200, timeout=1); time.sleep(2); s.reset_input_buffer()
    f = open(base + ".csv", "w", newline="", encoding="utf-8"); w = csv.writer(f)
    w.writerow(["t_pc_s","t_esp_ms","pwm_pct","T_termocupla","T_ir_obj","T_ir_amb","SP"])
    D, LECT = [], []
    def enviar(cmd): s.write((cmd + "\n").encode())

    enviar("K%g,%g,%g" % GANANCIAS); time.sleep(0.3)
    t0 = time.time(); paso = 0; sp = float("nan"); apagado = False
    anotado = [False]*len(ESCALONES); t_ult = 0.0; total = len(ESCALONES)*T_PASO
    p(f"Inicio {ahora:%Y-%m-%d %H:%M} | {MATERIAL} | escalones {ESCALONES} °C | {T_PASO} s cada uno")
    p("Ctrl+C para detener (apaga el cuarzo y guarda).")
    try:
        while time.time() - t0 < total + ENFRIAR:
            tt = time.time() - t0
            nuevo = int(tt // T_PASO) + 1
            if nuevo <= len(ESCALONES) and nuevo != paso:
                paso = nuevo; sp = ESCALONES[paso-1]; enviar("R%g" % sp); t_ult = tt
                p(f"[{tt:6.0f} s] Escalon {paso}: referencia {sp} °C")
            elif nuevo > len(ESCALONES) and not apagado:
                enviar("P0"); apagado = True; sp = float("nan")
                p(f"[{tt:6.0f} s] Escalones terminados, cuarzo apagado, enfriando"
                  "  <- AHORA: mira si el laser cae de golpe (reflejo) o despacio")
            if (paso >= 1 and paso <= len(ESCALONES) and not anotado[paso-1]
                    and tt >= paso*T_PASO - T_AVISO and len(D) > 5):
                anotado[paso-1] = True; beep()
                rec = [d for d in D if d[0] > tt - 30]
                ir = sorted(d[4] for d in rec)[len(rec)//2]; amb = sum(d[5] for d in rec)/len(rec)
                tcc = sum(d[3] for d in rec)/len(rec)
                p(f"\n>>> ESCALON {sp} °C  (IR marca {ir:.1f} °C).  Escribe y Enter (vacio = no medido):")
                m  = leer_num("  1) MULTIMETRO (termocupla pegada) °C: ")
                lp = leer_num("  2) LASER en el punto del IR      °C: ")
                lc = leer_num("  3) LASER sobre la cinta          °C: ")
                lz = leer_num("  4) LASER en el centro            °C: ")
                LECT.append([tt, sp, m, lp, lc, lz, ir, amb, tcc])
                p(f"[{tt:6.0f} s] ANOTADO SP={sp} multimetro={m} laser_punto={lp} laser_cinta={lc} laser_centro={lz} | IR={ir:.1f}\n")
                s.reset_input_buffer(); t_ult = time.time() - t0; continue
            if tt - t_ult > 15:
                enviar("S"); p("No llegan datos hace 15 s. Cuarzo apagado."); break
            L = s.readline().decode(errors="ignore").strip()
            if L.startswith("DATA,"):
                v = L[5:].split(",")
                if len(v) == 5:
                    try: v = [float(x) for x in v]
                    except ValueError: continue
                    fila = [time.time()-t0, v[0], v[1], v[2], v[3], v[4], sp]
                    D.append(fila); w.writerow([f"{x:.3f}" if isinstance(x, float) else x for x in fila]); t_ult = fila[0]
                    if len(D) % 5 == 0:
                        print(f"t={fila[0]:5.0f} s  SP={sp:>5}  PWM={v[1]:4.1f}  TC={v[2]:6.1f}  IR={v[3]:6.1f}")
            elif L.startswith("#"):
                p(L)
                if "SEGURIDAD" in L or "ERROR" in L: p("El ESP32 detuvo el proceso."); break
    except KeyboardInterrupt:
        p("\nDETENIDO por el usuario.")
    finally:
        enviar("S"); time.sleep(0.2); s.close(); f.close()

    # ---- resultados ----
    with open(base + "_resultados.csv", "w", newline="", encoding="utf-8") as fr:
        wr = csv.writer(fr)
        wr.writerow(["t_s","SP_C","T_contacto_C","Laser_punto_C","Laser_cinta_C","Laser_centro_C",
                     "T_IR_C","T_amb_sensor_C","T_termocupla_camara_C","Error_IR_C","Emisividad_aprox"])
        errs, epss = [], []
        for r in LECT:
            e = r[6] - r[2] if not math.isnan(r[2]) else float("nan")
            K = lambda T: (T + 273.15)**4
            eps = (K(r[6]) - K(r[7])) / (K(r[2]) - K(r[7])) if not math.isnan(r[2]) and abs(r[2]-r[7]) > 3 else float("nan")
            if not math.isnan(e): errs.append(e)
            if not math.isnan(eps): epss.append(eps)
            wr.writerow([round(x, 3) if isinstance(x, float) else x for x in r + [e, eps]])
    p("\nRESULTADOS (IR - contacto por escalon):")
    for r in LECT:
        p(f"  SP {r[1]:>4}: contacto {r[2]:6.1f}  IR {r[6]:6.1f}  laser punto {r[3]:6.1f}  cinta {r[4]:6.1f}  centro {r[5]:6.1f}")
    if errs:
        rmse = math.sqrt(sum(e*e for e in errs)/len(errs))
        p(f"RMSE IR vs contacto: {rmse:.2f} °C | sesgo medio {sum(errs)/len(errs):+.2f} °C"
          + (f" | emisividad aprox {sum(epss)/len(epss):.3f}" if epss else ""))
    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        t = [d[0] for d in D]
        fig, ax = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True)
        ax[0].plot(t, [d[4] for d in D], color=(0.1,0.35,0.95), label="IR (lámina)")
        ax[0].plot(t, [d[3] for d in D], color=(0.85,0.1,0.1), label="Termocupla cámara")
        ax[0].step(t, [d[6] for d in D], "--", color=(0,0.75,0), where="post", label="Referencia")
        ax[0].plot([r[0] for r in LECT], [r[2] for r in LECT], "ks", mfc="y", ms=8, label="Multímetro")
        ax[0].set_ylabel("T (°C)"); ax[0].legend(loc="lower right"); ax[0].grid(alpha=.4); ax[0].set_title(nombre)
        ax[1].step(t, [d[2] for d in D], color=(1,0.5,0), where="post"); ax[1].set_ylabel("PWM (%)"); ax[1].set_xlabel("t (s)"); ax[1].grid(alpha=.4)
        fig.tight_layout(); fig.savefig(base + ".png", dpi=150)
    except Exception as ex:
        p("(sin grafica:", ex, ")")
    p(f"Listo: todo guardado en {carpeta}")
    log.close()

if __name__ == "__main__":
    main()
