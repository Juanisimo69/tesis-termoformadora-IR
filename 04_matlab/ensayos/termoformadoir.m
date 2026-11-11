%% PRUEBA DE TERMOFORMADO CON CONTROL PI SOBRE LA LAMINA (sensor IR)
% Fases: 1) calentar con PI hasta SP  2) mantener SP durante SOAK s (dentro de +-BANDA)
%        3) AVISO: aplicar vacio / formar  4) apagar al presionar "YA FORME" (o al pasar VENTANA s)
%        5) registrar enfriamiento y guardar.
% Firmware: main.cpp con modo IR (comando R<temp>). Cierra el monitor serial antes de correr.
clear; clc;
% Los datos se guardan siempre en la carpeta 05_datos_ensayos (al lado de 04_matlab)
CARPETA_DATOS = fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), '05_datos_ensayos');
if ~isfolder(CARPETA_DATOS), mkdir(CARPETA_DATOS); end
cd(CARPETA_DATOS);

PUERTO   = "COM14";
MATERIAL = "PS";         % PS = lamina BLANCA   |   PET = lamina TRANSPARENTE
ENSAYO   = 1;
SP       = 100;          % °C de la lamina (IR). Punto de partida: PS ~100, PET ~110 (ajustar)
BANDA    = 3;            % °C: se considera "en temperatura" si |IR - SP| <= BANDA
SOAK     = 30;           % s en temperatura antes de avisar
VENTANA  = 60;           % s maximos con calor despues del aviso (si no presionas el boton)
T_MAX_CAL= 900;          % s maximos de calentamiento (si no llega, se aborta)
ENFRIAR  = 180;          % s registrando despues de apagar

nombre = "TF_" + MATERIAL + "_SP" + SP + "_E" + ENSAYO + "_" + string(datetime("now","Format","yyyyMMddHHmmss"));
s = serialport(PUERTO,115200); configureTerminator(s,"LF"); s.Timeout = 5; pause(2); flush(s);
fid = fopen(nombre + ".csv",'w'); fprintf(fid,"t_pc_s,t_esp_ms,pwm_pct,T_termocupla,T_ir_obj,T_ir_amb\n");

% ---- figura con boton ----
fig = figure('Name',nombre,'NumberTitle','off');
ax1 = subplot(2,1,1); h1=plot(nan,nan,'r'); hold on; h2=plot(nan,nan,'b'); yline(SP,'k--');
yline(SP+BANDA,'k:'); yline(SP-BANDA,'k:'); grid on; ylabel('T (°C)'); legend('Termocupla','IR (lamina)','Location','southeast')
ttl = title(ax1, "Calentando...",'FontSize',14);
subplot(2,1,2); h3=stairs(nan,nan,'k'); grid on; ylabel('PWM (%)'); xlabel('t (s)')
setappdata(fig,'formado',false);
uicontrol(fig,'Style','pushbutton','String','YA FORME','FontSize',14,'FontWeight','bold', ...
    'Units','normalized','Position',[0.80 0.005 0.18 0.06],'Callback',@(~,~) setappdata(fig,'formado',true));

% ---- proceso ----
D = zeros(0,6); fase = "CALENTANDO"; t_en_temp = NaN; t_aviso = NaN; t_apagado = NaN; t_ult = 0; ult_bip = -inf;
writeline(s, "R" + SP); t0 = tic;
fprintf("Inicio %s | %s | SP = %g °C\n", datetime('now'), MATERIAL, SP);
while true
    tp = toc(t0);
    if toc(t0) - t_ult > 15, writeline(s,"S"); warning("No llegan datos hace 15 s. PWM apagado."); break; end
    try, L = strtrim(readline(s)); catch, continue; end
    if startsWith(L,"#")
        fprintf("%s\n",L);
        if contains(L,["SEGURIDAD","ERROR"]), warning("El ESP32 detuvo el proceso."); fase = "ABORTADO"; break; end
        continue
    end
    if ~startsWith(L,"DATA,"), continue; end
    v = str2double(split(extractAfter(L,"DATA,"),","))'; if numel(v)~=5, continue; end
    fila = [tp v]; D(end+1,:) = fila; t_ult = tp; ir = fila(5); %#ok<SAGROW>
    fprintf(fid,"%.3f,%d,%.1f,%.2f,%.2f,%.2f\n",fila);
    if mod(size(D,1),20)==0, save(nombre + ".mat","D"); end

    switch fase
        case "CALENTANDO"
            if abs(ir - SP) <= BANDA
                if isnan(t_en_temp), t_en_temp = tp; end
                if tp - t_en_temp >= SOAK
                    fase = "FORMAR"; t_aviso = tp;
                    fprintf("\n[%6.0f s] >>> LISTO: APLICA EL VACIO Y FORMA LA LAMINA <<<\n\n", tp);
                    set(ttl,'String',"¡APLICA EL VACIO / FORMA AHORA!",'Color',[0.8 0 0]);
                end
            else
                t_en_temp = NaN;
            end
            if tp > T_MAX_CAL
                fase = "ABORTADO"; writeline(s,"P0"); t_apagado = tp;
                warning("No se alcanzo la temperatura en %d s. Se apaga.", T_MAX_CAL);
            end
        case "FORMAR"
            if tp - ult_bip > 3, beep; ult_bip = tp; end
            if (isvalid(fig) && getappdata(fig,'formado')) || tp - t_aviso > VENTANA
                writeline(s,"P0"); t_apagado = tp; fase = "ENFRIANDO";
                if isvalid(fig) && getappdata(fig,'formado'), motivo = "boton"; else, motivo = "tiempo maximo"; end
                fprintf("[%6.0f s] Calor apagado (%s). Registrando enfriamiento...\n", tp, motivo);
                set(ttl,'String',"Enfriando...",'Color',[0 0 0]);
            end
        case {"ENFRIANDO","ABORTADO"}
            if tp - t_apagado > ENFRIAR, break; end
    end
    if mod(size(D,1),3)==0 && isvalid(fig)
        set(h1,'XData',D(:,1),'YData',D(:,4)); set(h2,'XData',D(:,1),'YData',D(:,5)); set(h3,'XData',D(:,1),'YData',D(:,3));
        drawnow limitrate
        fprintf("t=%5.0f s  %-10s PWM=%4.1f  TC=%6.1f  IR=%6.1f\n",tp,fase,fila(3),fila(4),fila(5));
    end
end
writeline(s,"S"); fclose(fid); clear s

% ---- resumen ----
datos = array2table(D,'VariableNames',{'t_pc_s','t_esp_ms','pwm_pct','T_termocupla','T_ir_obj','T_ir_amb'});
y = movmedian(datos.T_ir_obj,3); t = datos.t_pc_s;
k_aviso = find(t >= t_aviso, 1); if isempty(k_aviso), ir_form = NaN; else, ir_form = y(k_aviso); end
R = table(MATERIAL, SP, ENSAYO, t_en_temp, t_aviso, t_apagado, max(y(t <= min(t_apagado, t(end)))) - SP, ...
    ir_form, max(datos.pwm_pct), max(datos.T_termocupla), fase, ...
    'VariableNames',{'Material','SP_C','Ensayo','t_llega_SP_s','t_aviso_s','t_apagado_s','Sobrepaso_C', ...
                     'IR_al_formar_C','PWM_max_pct','TC_max_C','Estado_final'});
disp(R)
save(nombre + ".mat","datos","D","R","SP","MATERIAL","ENSAYO");
if isvalid(fig), savefig(fig, nombre + ".fig"); end
if isfile("resumen_termoformado.csv"), writetable(R,"resumen_termoformado.csv",'WriteMode','append'); else, writetable(R,"resumen_termoformado.csv"); end
fprintf("Listo: %s (.csv .mat .fig) y fila en resumen_termoformado.csv\n", nombre);
