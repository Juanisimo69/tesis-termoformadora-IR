%% CALIBRACION DEL SENSOR IR CONTRA TERMOCUPLA DE CONTACTO
% El control sigue siendo con el IR (PID). La lamina se lleva por varios escalones de temperatura
% y en cada uno se compara la lectura del IR con una referencia de contacto.
%
% MODO = "multimetro": termocupla tipo K del multimetro (UT33) pegada con Kapton a la lamina,
%                      a ~1 cm del punto del IR. La termocupla ORIGINAL de la camara se queda conectada.
%                      Al final de cada escalon suena un pitido y sale una ventana para escribir
%                      lo que marca el multimetro y, si quieres, el termometro laser (punto, cinta, centro).
% MODO = "bornera":    termocupla tipo K conectada en la bornera en lugar de la original (registro automatico).
%
% Guarda todo en 05_datos_ensayos\CALIBRACION IR\MATERIAL_calibracion_fecha\
% Antes de correr: monitor serial CERRADO. Correr con el boton Run.
clear; clc;
CARPETA_DATOS = fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), '05_datos_ensayos');
cd(CARPETA_DATOS);

PUERTO   = "COM14";
MODO     = "multimetro";         % "multimetro" o "bornera"
MATERIAL = "PET";                 % PS = lamina BLANCA | PET = lamina TRANSPARENTE
if MATERIAL == "PS", ESCALONES = [45 60 75 85]; else, ESCALONES = [40 50 60 65]; end   % siempre por debajo de Tg
T_PASO   = 300;                  % s en cada escalon
T_AVISO  = 30;                   % s antes de terminar el escalon sale la ventana para anotar
ENFRIAR  = 120;                  % s registrando al final con el cuarzo apagado
GANANCIAS = [1.3 0.010 10];      % PID
cruda = @(tc) (tc - 9.5883) / 0.83396;   % deshace la calibracion de Cristian (solo para MODO bornera)

meses = ["ene","feb","mar","abr","may","jun","jul","ago","sep","oct","nov","dic"]; ahora = datetime('now');
nombre  = MATERIAL + "_calibracion_" + sprintf("%02d%s_%02d%02d", day(ahora), meses(month(ahora)), hour(ahora), minute(ahora));
carpeta = fullfile("CALIBRACION IR", nombre); mkdir(carpeta); base = fullfile(carpeta, nombre);
diary(char(fullfile(carpeta, "consola.txt")));

s = serialport(PUERTO,115200); configureTerminator(s,"LF"); s.Timeout = 5; pause(2); flush(s);
fid = fopen(base + ".csv",'w'); fprintf(fid,"t_pc_s,t_esp_ms,pwm_pct,T_termocupla,T_ir_obj,T_ir_amb,SP\n");

D = zeros(0,7); LECT = zeros(0,9);   % t, SP, multimetro, laser_punto, laser_cinta, laser_centro, IR_prom, IR_amb, TC_camara
fig = figure('Name',nombre); setappdata(fig,'parar',false);
uicontrol(fig,'Style','pushbutton','String','DETENER (guarda todo)','FontWeight','bold', ...
    'BackgroundColor',[1 .6 .6],'Units','normalized','Position',[.80 .94 .19 .05], ...
    'Callback',@(~,~) setappdata(fig,'parar',true));
subplot(2,1,1); h2=plot(nan,nan,'b','LineWidth',1.2); hold on; h1=plot(nan,nan,'r','LineWidth',1.0); h4=stairs(nan,nan,'--','Color',[0 .75 0]);
h5=plot(nan,nan,'ks','MarkerFaceColor','y','MarkerSize',8);
grid on; ylabel('T (°C)'); legend('IR','Termocupla','Referencia','Multimetro','Location','southeast'); title(nombre,'Interpreter','none')
subplot(2,1,2); h3=stairs(nan,nan,'Color',[1 .5 0],'LineWidth',1.2); grid on; ylabel('PWM (%)'); xlabel('t (s)');

writeline(s, sprintf("K%g,%g,%g", GANANCIAS)); pause(0.3);
t0 = tic; t_ult = 0; paso = 0; SPact = NaN; apagado = false; anotado = false(1,numel(ESCALONES));
T_TOTAL = numel(ESCALONES)*T_PASO;
fprintf("Inicio %s | %s | modo %s | escalones %s °C | %d s cada uno\n", datetime('now'), MATERIAL, MODO, mat2str(ESCALONES), T_PASO);
while toc(t0) < T_TOTAL + ENFRIAR
    tt = toc(t0);
    if isvalid(fig) && getappdata(fig,'parar'), writeline(s,"S"); fprintf("[%6.0f s] DETENIDO por el usuario\n", tt); break; end
    nuevo = floor(tt / T_PASO) + 1;
    if nuevo <= numel(ESCALONES) && nuevo ~= paso
        paso = nuevo; SPact = ESCALONES(paso); writeline(s, "R" + SPact); t_ult = tt;
        fprintf("[%6.0f s] Escalon %d: referencia %g °C\n", tt, paso, SPact);
    elseif nuevo > numel(ESCALONES) && ~apagado
        writeline(s,"P0"); apagado = true; SPact = NaN; fprintf("[%6.0f s] Escalones terminados, enfriando\n", tt);
    end
    % ---- ventana para anotar el multimetro al final de cada escalon ----
    if MODO == "multimetro" && paso >= 1 && paso <= numel(ESCALONES) && ~anotado(paso) && tt >= paso*T_PASO - T_AVISO && size(D,1) > 5
        anotado(paso) = true; beep; pause(0.2); beep;
        k = D(:,1) > tt - 30; ir_prom = median(D(k,5)); ir_amb = mean(D(k,6)); tc_b = mean(D(k,4));
        r = inputdlg({sprintf('Escalon %g °C (IR marca %.1f °C).  1) MULTIMETRO (termocupla pegada) (°C):', SPact, ir_prom), ...
                      '2) LASER en el punto del IR, al lado de la cinta (°C)', ...
                      '3) LASER sobre la cinta Kapton (°C)', ...
                      '4) LASER en el CENTRO de la lamina (°C)'}, ...
                      'Anotar lecturas (deja vacio lo que no midas)', [1 75], {'','','',''});
        if isempty(r), r = {'','','',''}; end
        v3 = str2double(strrep(r, ',', '.'));
        LECT(end+1,:) = [tt SPact v3(:)' ir_prom ir_amb tc_b]; %#ok<SAGROW>
        if isvalid(fig), set(h5,'XData',LECT(:,1),'YData',LECT(:,3)); end
        fprintf("[%6.0f s] ANOTADO  SP=%g  multimetro=%.1f  laser_punto=%.1f  laser_cinta=%.1f  laser_centro=%.1f  |  IR=%.1f\n", tt, SPact, v3, ir_prom);
        flush(s); t_ult = toc(t0); continue
    end
    if tt - t_ult > 15, writeline(s,"S"); warning("No llegan datos hace 15 s. PWM apagado."); break; end
    try, L = strtrim(readline(s)); catch, continue; end
    if startsWith(L,"DATA,")
        v = str2double(split(extractAfter(L,"DATA,"),","))';
        if numel(v)==5
            tcv = v(3); if MODO == "bornera", tcv = cruda(v(3)); end
            fila = [toc(t0) v(1) v(2) tcv v(4) v(5) SPact]; D(end+1,:) = fila; t_ult = fila(1); %#ok<SAGROW>
            fprintf(fid,"%.3f,%d,%.1f,%.2f,%.2f,%.2f,%g\n",fila);
            if mod(size(D,1),5)==0 && isvalid(fig)
                set(h1,'XData',D(:,1),'YData',D(:,4)); set(h2,'XData',D(:,1),'YData',D(:,5));
                set(h4,'XData',D(:,1),'YData',D(:,7)); set(h3,'XData',D(:,1),'YData',D(:,3)); drawnow limitrate
                fprintf("t=%5.0f s  SP=%4g  PWM=%4.1f  TC=%6.1f  IR=%6.1f\n",fila(1),fila(7),fila(3),fila(4),fila(5));
            end
        end
    elseif startsWith(L,"#")
        fprintf("%s\n",L);
        if contains(L,["SEGURIDAD","ERROR"]), warning("El ESP32 detuvo el proceso."); break; end
    end
end
writeline(s,"S"); fclose(fid); clear s
datos = array2table(D,'VariableNames',{'t_pc_s','t_esp_ms','pwm_pct','T_termocupla','T_ir_obj','T_ir_amb','SP'});

% ---- Resultados por escalon ----
if MODO == "bornera"      % referencia = termocupla en la bornera, ultimos 60 s de cada escalon
    for i = 1:numel(ESCALONES)
        k = datos.t_pc_s > i*T_PASO - 60 & datos.t_pc_s <= i*T_PASO; if nnz(k) < 5, continue; end
        LECT(end+1,:) = [i*T_PASO ESCALONES(i) mean(datos.T_termocupla(k)) NaN NaN NaN median(datos.T_ir_obj(k)) mean(datos.T_ir_amb(k)) NaN]; %#ok<SAGROW>
    end
end
R = array2table(LECT,'VariableNames',{'t_s','SP_C','T_contacto_C','Laser_punto_C','Laser_cinta_C','Laser_centro_C','T_IR_C','T_amb_sensor_C','T_termocupla_camara_C'});
R = R(~isnan(R.T_contacto_C),:);
if ~isempty(R)
    R.Error_IR_C = R.T_IR_C - R.T_contacto_C;
    K = @(T) (T + 273.15).^4;
    R.Emisividad_aprox = (K(R.T_IR_C) - K(R.T_amb_sensor_C)) ./ (K(R.T_contacto_C) - K(R.T_amb_sensor_C));
    R.Error_laser_punto_C = R.Laser_punto_C - R.T_contacto_C;  R.Error_laser_cinta_C = R.Laser_cinta_C - R.T_contacto_C;
    rmse = sqrt(mean(R.Error_IR_C.^2));
    disp(R)
    fprintf("RMSE IR vs contacto: %.2f °C | sesgo medio: %+.2f °C | emisividad media aprox: %.3f\n", rmse, mean(R.Error_IR_C), mean(R.Emisividad_aprox));
    if height(R) >= 2
        p = polyfit(R.T_IR_C, R.T_contacto_C, 1);
        fprintf("Correccion lineal sugerida: T_real = %.4f * T_IR + %.3f\n", p(1), p(2));
    else, p = [NaN NaN]; end
    writetable(R, base + "_resultados.csv");
    save(base + ".mat","datos","D","R","rmse","p","MATERIAL","ESCALONES","T_PASO","MODO");
else
    save(base + ".mat","datos","D","MATERIAL","ESCALONES","T_PASO","MODO");
end
if isvalid(fig)
    savefig(fig, base + ".fig"); try, theme(fig,"light"); catch, end
    exportgraphics(fig, base + ".png", 'Resolution', 200);
end
fprintf("Listo: todo guardado en 05_datos_ensayos\\%s\n", carpeta);
if MODO == "bornera", fprintf("RECUERDA: volver a conectar la termocupla original de la camara.\n"); end
diary off
