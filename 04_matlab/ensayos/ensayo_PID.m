%% ENSAYO DE CONTROL PID SOBRE LA LAMINA (sensor IR)
% Manda las ganancias (K) y la referencia (R) al ESP32, registra todo, calcula metricas
% y guarda cada ensayo en su propia carpeta dentro de 05_datos_ensayos:
%     05_datos_ensayos\ENSAYOS PID\PS_85C_E1_06nov_1740\   (o ENSAYOS PI)
%     05_datos_ensayos\PID VS PI\PS_85C_E1_PIvsPID.png     (si ya existe el ensayo del otro controlador)
% Antes de correr: maquina fria (termocupla ~30 °C), lamina nueva, monitor serial CERRADO.
% Correr con el boton Run (no con F9).
clear; clc;
CARPETA_DATOS = fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), '05_datos_ensayos');
if ~isfolder(CARPETA_DATOS), mkdir(CARPETA_DATOS); end
cd(CARPETA_DATOS);

PUERTO   = "COM14";
MATERIAL = "PET";         % PS = lamina BLANCA   |   PET = lamina TRANSPARENTE
ENSAYO   = 3;            % numero de repeticion (1, 2, 3...)
SP       = 60;           % °C. Ensayo de control ~10 °C por DEBAJO de Tg:  PS -> 85   |   PET -> 60
DURACION = 600;          % s de control
ENFRIAR  = 180;          % s registrando despues de apagar
GANANCIAS = [1.3 0.010 10];   % [Kp Ki Kd]  PID (sintonizado con el modelo de 2 modos, 6-nov)
                              % PID anterior: [1.0 0.0072 10]

if GANANCIAS(3) > 0, CTRL = "PID"; OTRO = "PI"; else, CTRL = "PI"; OTRO = "PID"; end
SUB = "ENSAYOS " + CTRL;  SUB_OTRO = "ENSAYOS " + OTRO;  COMP = "PID VS PI";
meses = ["ene","feb","mar","abr","may","jun","jul","ago","sep","oct","nov","dic"]; ahora = datetime('now');
corto  = MATERIAL + "_" + SP + "C_E" + ENSAYO;                                   % ej: PS_85C_E1
nombre = corto + "_" + sprintf("%02d%s_%02d%02d", day(ahora), meses(month(ahora)), hour(ahora), minute(ahora));  % ej: PS_85C_E1_06nov_1740
carpeta = fullfile(SUB, nombre); mkdir(carpeta); base = fullfile(carpeta, nombre);
diary(char(fullfile(carpeta, "consola.txt")));

s = serialport(PUERTO,115200); configureTerminator(s,"LF"); s.Timeout = 5; pause(2); flush(s);
fid = fopen(base + ".csv",'w'); fprintf(fid,"t_pc_s,t_esp_ms,pwm_pct,T_termocupla,T_ir_obj,T_ir_amb\n");

C_TC = [0.85 0.1 0.1]; C_IR = [0.1 0.35 0.95]; C_SP = [0 0.75 0]; C_PWM = [1 0.5 0];
D = zeros(0,6);
fig = figure('Name',nombre); setappdata(fig,'parar',false);
uicontrol(fig,'Style','pushbutton','String','DETENER (guarda todo)','FontWeight','bold', ...
    'BackgroundColor',[1 .6 .6],'Units','normalized','Position',[.80 .94 .19 .05], ...
    'Callback',@(~,~) setappdata(fig,'parar',true));
subplot(2,1,1); h1=plot(nan,nan,'Color',C_TC,'LineWidth',1.2); hold on; h2=plot(nan,nan,'Color',C_IR,'LineWidth',1.2);
yline(SP,'--','Color',C_SP,'LineWidth',1.5); grid on
ylabel('T (°C)'); legend('Termocupla','IR (lamina)','Referencia','Location','southeast'); title(nombre,'Interpreter','none')
subplot(2,1,2); h3=stairs(nan,nan,'Color',C_PWM,'LineWidth',1.3); grid on; ylabel('PWM (%)'); xlabel('t (s)')

writeline(s, sprintf("K%g,%g,%g", GANANCIAS)); pause(0.3);
writeline(s, "R" + SP); t0 = tic; apagado = false; t_ult = 0;
fprintf("Inicio %s | %s | %s | SP = %g °C | Kp=%g Ki=%g Kd=%g\n", datetime('now'), MATERIAL, CTRL, SP, GANANCIAS);
while toc(t0) < DURACION + ENFRIAR
    if isvalid(fig) && getappdata(fig,'parar')
        writeline(s,"S"); fprintf("[%6.0f s] DETENIDO por el usuario. Cuarzo apagado, guardando...\n", toc(t0)); break
    end
    if ~apagado && toc(t0) >= DURACION
        writeline(s,"P0"); apagado = true; fprintf("[%6.0f s] Control terminado, registrando enfriamiento\n", toc(t0));
    end
    if toc(t0) - t_ult > 15, writeline(s,"S"); warning("No llegan datos hace 15 s. PWM apagado."); break; end
    try, L = strtrim(readline(s)); catch, continue; end
    if startsWith(L,"DATA,")
        v = str2double(split(extractAfter(L,"DATA,"),","))';
        if numel(v)==5
            fila = [toc(t0) v]; D(end+1,:) = fila; t_ult = fila(1); %#ok<SAGROW>
            fprintf(fid,"%.3f,%d,%.1f,%.2f,%.2f,%.2f\n",fila);
            if mod(size(D,1),20)==0, save(base + ".mat","D"); end
            if mod(size(D,1),5)==0 && isvalid(fig)
                set(h1,'XData',D(:,1),'YData',D(:,4)); set(h2,'XData',D(:,1),'YData',D(:,5)); set(h3,'XData',D(:,1),'YData',D(:,3)); drawnow limitrate
                fprintf("t=%5.0f s  PWM=%4.1f  TC=%6.1f  IR=%6.1f\n",fila(1),fila(3),fila(4),fila(5));
            end
        end
    elseif startsWith(L,"#")
        fprintf("%s\n",L);
        if contains(L,["SEGURIDAD","ERROR"]), warning("El ESP32 detuvo el proceso."); break; end
    end
end
writeline(s,"S"); fclose(fid); clear s
datos = array2table(D,'VariableNames',{'t_pc_s','t_esp_ms','pwm_pct','T_termocupla','T_ir_obj','T_ir_amb'});

% ---- Metricas (solo el tramo con control) ----
k   = datos.t_pc_s <= min(DURACION, datos.t_pc_s(end));  t = datos.t_pc_s(k);  y = movmedian(datos.T_ir_obj(k),3);
y0  = y(1); dy = SP - y0;
k10 = find(y >= y0 + 0.1*dy,1); k90 = find(y >= y0 + 0.9*dy,1);
fuera = find(abs(y - SP) > 0.02*dy, 1, 'last');
if isempty(k10) || isempty(k90), t_sub = NaN; else, t_sub = t(k90)-t(k10); end
if isempty(fuera), t_est = 0; else, t_est = t(min(fuera+1,numel(t))); end
M = table(MATERIAL, CTRL, SP, ENSAYO, GANANCIAS(1), GANANCIAS(2), GANANCIAS(3), ...
    mean(SP - y(t >= t(end)-120)), max(0,(max(y)-SP)/dy*100), t_sub, t_est, ...
    max(datos.pwm_pct(k)), max(datos.T_termocupla(k)), string(SUB + "\" + nombre), ...
    'VariableNames',{'Material','Control','SP_C','Ensayo','Kp','Ki','Kd','Error_final_C','Sobrepico_pct', ...
                     'T_subida_s','T_asent_2pct_s','PWM_max_pct','TC_max_C','Carpeta'});
disp(M)
save(base + ".mat","datos","D","M","SP","MATERIAL","ENSAYO","GANANCIAS");
if isvalid(fig)
    savefig(fig, base + ".fig");
    try, theme(fig,"light"); catch, end            % imagen con fondo blanco para la tesis
    exportgraphics(fig, base + ".png", 'Resolution', 200);
end
if isfile("resumen_ensayos.csv"), writetable(M,"resumen_ensayos.csv",'WriteMode','append'); else, writetable(M,"resumen_ensayos.csv"); end

% ---- Comparacion automatica con el otro controlador (mismo material, SP y numero de ensayo) ----
ref = dir(fullfile(CARPETA_DATOS, SUB_OTRO, "**", char(corto + "_*.csv")));
if isempty(ref), ref = dir(fullfile(CARPETA_DATOS, SUB_OTRO, "**", char(MATERIAL + "_" + SP + "C_E*.csv"))); end   % si no hay del mismo E, el mas reciente
if ~isempty(ref)
    [~,j] = max([ref.datenum]); P = readtable(fullfile(ref(j).folder, ref(j).name));
    if CTRL == "PID", A = P; B = datos; nA = "PI"; nB = "PID"; else, A = datos; B = P; nA = "PI"; nB = "PID"; end
    if ~isfolder(COMP), mkdir(COMP); end; bcomp = fullfile(COMP, corto + "_PIvsPID");
    f2 = figure('Name',"PI vs PID - " + MATERIAL + " SP" + SP);
    subplot(2,1,1); plot(A.t_pc_s, A.T_ir_obj,'Color',[.55 .55 .55],'LineWidth',1.3); hold on
    plot(B.t_pc_s, B.T_ir_obj,'Color',C_IR,'LineWidth',1.3); yline(SP,'--','Color',C_SP,'LineWidth',1.5); grid on
    ylabel('T lamina IR (°C)'); legend(nA, nB, 'Referencia','Location','southeast'); title("PI vs PID - " + MATERIAL + " SP " + SP + " °C")
    subplot(2,1,2); stairs(A.t_pc_s, A.pwm_pct,'Color',[.55 .55 .55],'LineWidth',1.2); hold on
    stairs(B.t_pc_s, B.pwm_pct,'Color',C_PWM,'LineWidth',1.2); grid on; ylabel('PWM (%)'); xlabel('t (s)'); legend(nA, nB)
    savefig(f2, bcomp + ".fig");
    try, theme(f2,"light"); catch, end
    exportgraphics(f2, bcomp + ".png", 'Resolution', 200);
    fprintf("Comparacion guardada en 05_datos_ensayos\\%s (contra %s)\n", bcomp, ref(j).name);
end
fprintf("Listo: todo guardado en 05_datos_ensayos\\%s  y fila agregada a resumen_ensayos.csv\n", carpeta);
diary off
