%% PRUEBA EN ESCALERA (CORREGIDO 20-30) - registro cada 0.5 s: PWM 20% -> 30% -> 0%, registra termocupla + IR
% Version corregida tras perforarse la lamina con 30/50 %. Potencias mas bajas
% y corte por temperatura de la lamina (IR) para no volver a perforarla.
% MATLAB manda el PWM al ESP32 y guarda todo. No tocar la maquina durante la prueba.
clear; clc;

PUERTO   = "COM14";         % <-- cambia si es otro
NOMBRE   = "corregido2030"; % nombre de los archivos de salida
% Escalera: [PWM (%)  duracion (s)]
ESCALERA = [20  600;
            30  600;
             0  600];      % enfriamiento
IR_MAX   = 115;            % °C en la lamina (IR). Si se supera -> PWM 0 y sigue registrando
                           % Corrida 1: con 30 % la lamina llego a ~99 °C y aguanto; con 50 % se perforo hacia ~136 °C.

s = serialport(PUERTO,115200);
configureTerminator(s,"LF"); s.Timeout = 5;
pause(2); flush(s);

archivo = NOMBRE + string(datetime("now","Format","yyyyMMddHHmmss"));
fid = fopen(archivo + ".csv",'w');
fprintf(fid,"t_pc_s,t_esp_ms,pwm_pct,T_termocupla,T_ir_obj,T_ir_amb\n");

D = zeros(0,6);
figure('Name',NOMBRE); 
subplot(2,1,1); h1=plot(nan,nan,'r'); hold on; h2=plot(nan,nan,'b'); grid on
ylabel('T (°C)'); legend('Termocupla','IR (lamina)','Location','northwest')
subplot(2,1,2); h3=stairs(nan,nan,'k'); grid on; ylabel('PWM (%)'); xlabel('t (s)')

t0 = tic; etapa = 0; t_fin_etapa = 0; cortado = false; t_ult_dato = 0;
fprintf("Inicio %s\n", datetime('now'));
while true
    tp = toc(t0);
    % cambio de etapa
    if tp >= t_fin_etapa
        etapa = etapa + 1;
        if etapa > size(ESCALERA,1), break; end
        if cortado, writeline(s,"P0"); else, writeline(s, "P" + ESCALERA(etapa,1)); end
        t_fin_etapa = t_fin_etapa + ESCALERA(etapa,2);
        fprintf("[%6.0f s] PWM = %d %%\n", tp, ESCALERA(etapa,1));
    end
    % Si no llegan datos en 15 s, se apaga todo y se avisa
    if toc(t0) - t_ult_dato > 15
        writeline(s,"S");
        warning("No llegan lineas DATA del ESP32 hace 15 s. PWM apagado. Revisa el firmware/puerto.");
        break
    end
    % lectura
    try, L = readline(s); catch, continue; end
    if isempty(L), continue; end
    L = strtrim(L);
    if startsWith(L,"DATA,")
        v = str2double(split(extractAfter(L,"DATA,"),","))';
        if numel(v)==5
            fila = [toc(t0) v];
            D(end+1,:) = fila; %#ok<SAGROW>
            t_ult_dato = fila(1);
            if mod(size(D,1),20)==0, save(archivo + ".mat","D","ESCALERA"); end   % respaldo continuo
            fprintf(fid,"%.3f,%d,%.1f,%.2f,%.2f,%.2f\n",fila);
            % Corte por temperatura de la lamina
            if ~cortado && fila(5) > IR_MAX
                writeline(s,"P0"); cortado = true;
                ESCALERA(etapa:end,1) = 0;            % el resto de la prueba queda en 0 %
                fprintf("[%6.0f s] IR = %.1f °C > IR_MAX (%g °C): PWM -> 0 %%\n",fila(1),fila(5),IR_MAX);
            end
            if mod(size(D,1),5)==0
                set(h1,'XData',D(:,1),'YData',D(:,4)); set(h2,'XData',D(:,1),'YData',D(:,5));
                set(h3,'XData',D(:,1),'YData',D(:,3)); drawnow limitrate
                fprintf("t=%5.0f s  n=%4d  PWM=%3.0f  TC=%6.1f  IR=%6.1f\n",fila(1),size(D,1),fila(3),fila(4),fila(5));
            end
        end
    elseif startsWith(L,"#")
        fprintf("%s\n",L);           % mensajes del ESP32 (comandos, seguridad)
        if contains(L,"SEGURIDAD"), warning("El ESP32 apago el PWM por seguridad. Se detiene la prueba."); break; end
    end
end
writeline(s,"S"); fclose(fid);
datos = array2table(D,'VariableNames',{'t_pc_s','t_esp_ms','pwm_pct','T_termocupla','T_ir_obj','T_ir_amb'});
save(archivo + ".mat","datos","D","ESCALERA");
savefig(gcf, archivo + ".fig");
clear s
fprintf("Listo: %s.mat / .csv  (%d muestras)\n", archivo, height(datos));
% Chequeo de Ts real
dt = diff(datos.t_esp_ms)/1000;
fprintf("Ts real (reloj ESP32): media %.3f s, min %.3f, max %.3f\n",mean(dt),min(dt),max(dt));
fprintf("Termocupla sin lectura (nan): %d muestras\n", sum(isnan(datos.T_termocupla)));