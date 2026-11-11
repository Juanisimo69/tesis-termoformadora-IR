%% IDENTIFICACION Y DISENO DE CONTROL - Termoformadora (sensor IR + termocupla)
% se usaron los datos de corrida1_30_50.csv (escalon 0->30 %, se corta antes de perforarse la lamina)
% se usaron los datos de corrida_final.csv  (escalera 20 -> 30 -> 0 %)
clear; clc; close all;

Ts        = 2;          % s  (confirmado: reloj ESP32 = reloj PC)
T_CORTE1  = 715;        % s  corrida 1: despues de esto la lamina se perforo
N_PRE     = 5;          % muestras de reposo agregadas antes del escalon
VENTANA   = 5;          % filtro de mediana (quita los bajones del IR)
dirFig    = "figuras";  if ~exist(dirFig,"dir"), mkdir(dirFig); end

%% 1) CARGAR Y PREPARAR DATOS
[d1tc, d1ir, info1] = prepararCorrida("corrida1_30_50.csv", T_CORTE1, Ts, N_PRE, VENTANA);
[d2tc, d2ir, info2] = prepararCorrida("corrida_final.csv",  inf,      Ts, N_PRE, VENTANA);
fprintf("Corrida 1: %d muestras | T0 termocupla %.1f °C, IR %.1f °C\n", info1.N, info1.y0tc, info1.y0ir);
fprintf("Corrida 2: %d muestras | T0 termocupla %.1f °C, IR %.1f °C\n", info2.N, info2.y0tc, info2.y0ir);

figure('Name','Datos'); 
subplot(2,1,1); plot(info1.t,info1.tc,'r',info1.t,info1.ir,'b'); grid on; title('Corrida 1 (0 \rightarrow 30 %)'); ylabel('°C'); legend('Termocupla','IR lamina','Location','best')
subplot(2,1,2); plot(info2.t,info2.tc,'r',info2.t,info2.ir,'b'); grid on; title('Corrida final (20 \rightarrow 30 \rightarrow 0 %)'); ylabel('°C'); xlabel('t (s)')
exportgraphics(gcf, fullfile(dirFig,"fig_datos_corridas.png"),'Resolution',200);

%% 2) IDENTIFICACION (con la corrida 1) Y VALIDACION CRUZADA (con la corrida final)
opt  = tfestOptions('InitialCondition','zero');
copt = compareOptions('InitialCondition','z');
estructuras = {'1 polo + retardo',1,0; '2 polos + retardo',2,0; '2 polos + 1 cero',2,1};
salidas = {'Termocupla',d1tc,d2tc; 'IR (lamina)',d1ir,d2ir};
modelos = struct(); T = table();
for s = 1:2
    for e = 1:size(estructuras,1)
        np = estructuras{e,2}; nz = estructuras{e,3};
        if nz==0, m = tfest(salidas{s,2}, np, nz, NaN, opt);   % estima el retardo
        else,     m = tfest(salidas{s,2}, np, nz, opt); end
        [~,fid] = compare(salidas{s,2}, m, copt);
        [~,fva] = compare(salidas{s,3}, m, copt);
        K = dcgain(m);
        T = [T; table(string(salidas{s,1}), string(estructuras{e,1}), fid, fva, K, m.IODelay, ...
             'VariableNames',{'Salida','Modelo','Ajuste_id','Ajuste_val','K_C_por_duty','Retardo_s'})]; %#ok<AGROW>
        modelos.(sprintf('s%d_e%d',s,e)) = m;
    end
end
disp(T)
writetable(T, "tabla_modelos.csv");

% Mejor modelo de cada salida = mayor ajuste en VALIDACION
[~,i1] = max(T.Ajuste_val(1:3));
i2 = 1;   % IR: 1 polo + retardo (el de 2 polos + 1 cero tiene un cero en el semiplano derecho, no fisico)
G_tc = modelos.(sprintf('s1_e%d',i1));  G_ir = modelos.(sprintf('s2_e%d',i2));
fprintf("\nModelo elegido termocupla: %s\n", T.Modelo(i1)); G_tc
fprintf("Modelo elegido IR (lamina): %s\n", T.Modelo(3+i2)); G_ir

figure('Name','Validacion termocupla');
compare(d2tc, modelos.s1_e1, modelos.s1_e2, modelos.s1_e3, copt); title('Termocupla: modelos de la corrida 1 validados con la corrida final')
exportgraphics(gcf, fullfile(dirFig,"fig_validacion_termocupla.png"),'Resolution',200);
figure('Name','Validacion IR');
compare(d2ir, modelos.s2_e1, modelos.s2_e2, modelos.s2_e3, copt); title('IR (lamina): modelos de la corrida 1 validados con la corrida final')
exportgraphics(gcf, fullfile(dirFig,"fig_validacion_ir.png"),'Resolution',200);

figure('Name','Escalon'); step(0.3*G_tc, 0.3*G_ir, 1500); grid on
legend('Termocupla','IR (lamina)','Location','southeast'); title('Respuesta a un escalon de 30 % (modelos elegidos)')
exportgraphics(gcf, fullfile(dirFig,"fig_escalon_modelos.png"),'Resolution',200);

%% 3) DISENO DE CONTROL Y SIMULACION (con saturacion 0-70 % y anti-windup, igual que el firmware)
T_AMB   = info2.y0ir;     % °C, temperatura inicial de la lamina
SP_LAM  = 100;            % °C, referencia de la lamina (ajusta al material)
U_MAX   = 0.70;           % salida maxima (OUT_MAX del firmware)
T_SIM   = 1500;           % s
TC_LIM  = 200;            % °C, limite de seguridad de la termocupla

Gp_tc = pade(tf(G_tc),2);  Gp_ir = pade(tf(G_ir),2);     % aproximacion del retardo para disenar

% --- Opcion A: lazo simple PI sobre el IR (lamina) ---
C_A = pidtune(Gp_ir,'PI',pidtuneOptions('PhaseMargin',75));

% --- Opcion B: cascada (esclavo = termocupla, maestro = IR) ---
C_in  = pidtune(Gp_tc,'PI',pidtuneOptions('PhaseMargin',60));   % esclavo con su sintonia natural
T_in  = feedback(C_in*Gp_tc,1);
wb_in = bandwidth(T_in);
fprintf("Esclavo estable: %d | ancho de banda %.4f rad/s\n", isstable(T_in), wb_in);
P_out = minreal(ss(Gp_ir*C_in/(1 + C_in*Gp_tc)));                % de referencia de termocupla a lamina
C_out = pidtune(P_out,'PI',wb_in/3,pidtuneOptions('PhaseMargin',65));  % maestro 3 veces mas lento que el esclavo
fprintf("Esclavo: Kp=%.5f Ki=%.6f | Maestro: Kp=%.4f Ki=%.6f\n", C_in.Kp, C_in.Ki, C_out.Kp, C_out.Ki);
if C_out.Kp <= 0 || C_out.Ki <= 0
    warning("El maestro salio con ganancias no positivas: la cascada no es viable con estos modelos.");
end

simA = simularLazoSimple(G_ir, G_tc, C_A, SP_LAM-T_AMB, U_MAX, Ts, T_SIM);
simB = simularCascada(G_ir, G_tc, C_in, C_out, SP_LAM-T_AMB, U_MAX, Ts, T_SIM, info2.y0tc);

figure('Name','Control');
subplot(3,1,1); plot(simA.t, simA.ir+T_AMB, simB.t, simB.ir+T_AMB, 'LineWidth',1.2); yline(SP_LAM,'k--'); grid on
ylabel('Lamina (°C)'); legend('A: PI sobre IR','B: cascada','Referencia','Location','southeast'); title('Simulacion de control')
subplot(3,1,2); plot(simA.t, simA.tc+info2.y0tc, simB.t, simB.tc+info2.y0tc,'LineWidth',1.2); yline(TC_LIM,'r:'); grid on; ylabel('Termocupla (°C)')
subplot(3,1,3); stairs(simA.t,100*simA.u); hold on; stairs(simB.t,100*simB.u); grid on; ylabel('PWM (%)'); xlabel('t (s)')
exportgraphics(gcf, fullfile(dirFig,"fig_simulacion_control.png"),'Resolution',200);

R = [metricas("A: PI sobre IR", simA, SP_LAM-T_AMB); metricas("B: cascada", simB, SP_LAM-T_AMB)];
disp(R); writetable(R,"tabla_control.csv");

%% 4) GANANCIAS PARA EL FIRMWARE (salida en %, Ts = 2 s, misma forma que calcularPIDF)
fprintf("\n=== Ganancias para el firmware (salida en %%) ===\n");
fprintf("Opcion A (PI sobre IR):        Kp = %.3f  Ki = %.5f\n", 100*C_A.Kp, 100*C_A.Ki);
fprintf("Opcion B esclavo (termocupla): Kp = %.3f  Ki = %.5f\n", 100*C_in.Kp, 100*C_in.Ki);
fprintf("Opcion B maestro (IR):         Kp = %.3f  Ki = %.5f   (salida en °C de referencia de termocupla)\n", C_out.Kp, C_out.Ki);
save("resultados_identificacion_control.mat","G_tc","G_ir","modelos","T","C_A","C_in","C_out","R","simA","simB");
fprintf("\nListo. Figuras en la carpeta '%s'. Tablas: tabla_modelos.csv, tabla_control.csv\n", dirFig);

%% ===================== FUNCIONES =====================
function [dtc, dir_, info] = prepararCorrida(archivo, tCorte, Ts, nPre, ventana)
    c  = readtable(archivo);
    t  = (c.t_esp_ms - c.t_esp_ms(1))/1000;
    k  = t <= tCorte;  c = c(k,:);  t = t(k);
    tg = (0:Ts:t(end))';
    u  = interp1(t, c.pwm_pct/100, tg, 'previous');
    tc = interp1(t, movmedian(c.T_termocupla, ventana), tg, 'linear');
    ir = interp1(t, movmedian(c.T_ir_obj,     ventana), tg, 'linear');
    y0tc = mean(tc(1:5)); y0ir = mean(ir(1:5));
    U  = [zeros(nPre,1); u];
    dtc  = iddata([zeros(nPre,1); tc-y0tc], U, Ts, 'OutputName','Termocupla','InputName','PWM');
    dir_ = iddata([zeros(nPre,1); ir-y0ir], U, Ts, 'OutputName','IR','InputName','PWM');
    info = struct('t',tg,'tc',tc,'ir',ir,'u',u,'y0tc',y0tc,'y0ir',y0ir,'N',numel(tg));
end

function s = simularLazoSimple(Gir, Gtc, C, r, umax, Ts, Tsim)
    [Air,Bir,Cir,~] = ssdata(ss(absorbDelay(c2d(tf(Gir),Ts))));
    [Atc,Btc,Ctc,~] = ssdata(ss(absorbDelay(c2d(tf(Gtc),Ts))));
    N = round(Tsim/Ts); xi = zeros(size(Air,1),1); xt = zeros(size(Atc,1),1); I = 0;
    s.t = (0:N-1)'*Ts; s.ir = zeros(N,1); s.tc = s.ir; s.u = s.ir;
    for k = 1:N
        yir = Cir*xi; ytc = Ctc*xt;
        e = r - yir;
        [u, I] = piAntiWindup(C.Kp, C.Ki, e, I, Ts, 0, umax);
        xi = Air*xi + Bir*u; xt = Atc*xt + Btc*u;
        s.ir(k) = yir; s.tc(k) = ytc; s.u(k) = u;
    end
end

function s = simularCascada(Gir, Gtc, Cin, Cout, r, umax, Ts, Tsim, tc0)
    [Air,Bir,Cir,~] = ssdata(ss(absorbDelay(c2d(tf(Gir),Ts))));
    [Atc,Btc,Ctc,~] = ssdata(ss(absorbDelay(c2d(tf(Gtc),Ts))));
    N = round(Tsim/Ts); xi = zeros(size(Air,1),1); xt = zeros(size(Atc,1),1); Ii = 0; Io = 0;
    rtcMax = 200 - tc0;                                 % el maestro nunca pide mas de 200 °C a la termocupla
    s.t = (0:N-1)'*Ts; s.ir = zeros(N,1); s.tc = s.ir; s.u = s.ir;
    for k = 1:N
        yir = Cir*xi; ytc = Ctc*xt;
        [rtc, Io] = piAntiWindup(Cout.Kp, Cout.Ki, r - yir, Io, Ts, 0, rtcMax);
        [u,   Ii] = piAntiWindup(Cin.Kp,  Cin.Ki,  rtc - ytc, Ii, Ts, 0, umax);
        xi = Air*xi + Bir*u; xt = Atc*xt + Btc*u;
        s.ir(k) = yir; s.tc(k) = ytc; s.u(k) = u;
    end
end

function [u, I] = piAntiWindup(Kp, Ki, e, I, Ts, umin, umax)
    Inew = I + Ki*Ts*e;
    ut   = Kp*e + Inew;
    if ~((ut > umax && e > 0) || (ut < umin && e < 0)), I = Inew; end   % clamping, como el firmware
    u = min(max(Kp*e + I, umin), umax);
end

function R = metricas(nombre, s, r)
    y = s.ir; final = mean(y(end-20:end));
    sobre = max(0, (max(y)-r)/r*100);
    k = find(abs(y - r) > 0.02*r, 1, 'last');
    if isempty(k), ts = 0; else, ts = s.t(min(k+1,numel(s.t))); end
    k10 = find(y >= 0.1*r,1); k90 = find(y >= 0.9*r,1);
    tr = NaN; if ~isempty(k10) && ~isempty(k90), tr = s.t(k90)-s.t(k10); end
    R = table(string(nombre), r-final, sobre, tr, ts, 100*max(s.u), ...
        'VariableNames',{'Controlador','Error_final_C','Sobrepico_pct','T_subida_s','T_asent_2pct_s','PWM_max_pct'});
end
