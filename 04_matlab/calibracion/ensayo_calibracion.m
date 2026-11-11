%% CALIBRACION DEL SENSOR IR CONTRA TERMOCUPLA TIPO K DE CONTACTO
% La termocupla tipo K va pegada a la lamina (cinta Kapton, ~1 cm del punto del IR)
% y conectada en la bornera de la termocupla de la camara. El control sigue siendo con el IR (PID),
% asi que la termocupla solo registra. La lamina se lleva por varios escalones de temperatura
% y en cada uno se compara IR vs contacto.
% Guarda todo en 05_datos_ensayos\CALIBRACION IR\MATERIAL_calibracion_fecha\
% Antes de correr: monitor serial CERRADO. Correr con el boton Run.
% AL TERMINAR: volver a conectar la termocupla original de la camara.
clear; clc;
CARPETA_DATOS = fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), '05_datos_ensayos');
cd(CARPETA_DATOS);

PUERTO   = "COM14";
MATERIAL = "PS";                 % PS = lamina BLANCA | PET = lamina TRANSPARENTE
if MATERIAL == "PS", ESCALONES = [45 60 75 85]; else, ESCALONES = [40 50 60 65]; end   % siempre por debajo de Tg
T_PASO   = 300;                  % s en cada escalon (se promedian los ultimos 60 s)
ENFRIAR  = 120;                  % s registrando al final con el cuarzo apagado
GANANCIAS = [1.3 0.010 10];      % PID
% El firmware le aplica a la termocupla la calibracion de Cristian (0.83396*cruda + 9.5883),
% que era para la termocupla del calefactor. Aqui se deshace para obtener la lectura cruda del MAX6675.
cruda = @(tc) (tc - 9.5883) / 0.83396;

meses = ["ene","feb","mar","abr","may","jun","jul","ago","sep","oct","nov","dic"]; ahora = datetime('now');
nombre  = MATERIAL + "_calibracion_" + sprintf("%02d%s_%02d%02d", day(ahora), meses(month(ahora)), hour(ahora), minute(ahora));
carpeta = fullfile("CALIBRACION IR", nombre); mkdir(carpeta); base = fullfile(carpeta, nombre);
diary(char(fullfile(carpeta, "consola.txt")));

s = serialport(PUERTO,115200); configureTerminator(s,"LF"); s.Timeout = 5; pause(2); flush(s);
fid = fopen(base + ".csv",'w'); fprintf(fid,"t_pc_s,t_esp_ms,pwm_pct,T_contacto_firmware,T_ir_obj,T_ir_amb,T_contacto,SP\n");

D = zeros(0,8);
fig = figure('Name',nombre); setappdata(fig,'parar',false);
uicontrol(fig,'Style','pushbutton','String','DETENER (guarda todo)','FontWeight','bold', ...
    'BackgroundColor',[1 .6 .6],'Units','normalized','Position',[.80 .94 .19 .05], ...
    'Callback',@(~,~) setappdata(fig,'parar',true));
subplot(2,1,1); h1=plot(nan,nan,'r','LineWidth',1.2); hold on; h2=plot(nan,nan,'b','LineWidth',1.2); h4=stairs(nan,nan,'--','Color',[0 .75 0]);
grid on; ylabel('T lamina (°C)'); legend('Contacto (tipo K)','IR','Referencia','Location','southeast'); title(nombre,'Interpreter','none')
subplot(2,1,2); h3=plot(nan,nan,'k','LineWidth',1.2); grid on; ylabel('IR - contacto (°C)'); xlabel('t (s)'); yline(0,':');

writeline(s, sprintf("K%g,%g,%g", GANANCIAS)); pause(0.3);
t0 = tic; t_ult = 0; paso = 0; SPact = NaN; apagado = false;
T_TOTAL = numel(ESCALONES)*T_PASO;
fprintf("Inicio %s | %s | escalones %s °C | %d s cada uno\n", datetime('now'), MATERIAL, mat2str(ESCALONES), T_PASO);
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
    if tt - t_ult > 15, writeline(s,"S"); warning("No llegan datos hace 15 s. PWM apagado."); break; end
    try, L = strtrim(readline(s)); catch, continue; end
    if startsWith(L,"DATA,")
        v = str2double(split(extractAfter(L,"DATA,"),","))';
        if numel(v)==5
            fila = [toc(t0) v cruda(v(3)) SPact]; D(end+1,:) = fila; t_ult = fila(1); %#ok<SAGROW>
            fprintf(fid,"%.3f,%d,%.1f,%.2f,%.2f,%.2f,%.2f,%g\n",fila);
            if mod(size(D,1),5)==0 && isvalid(fig)
                set(h1,'XData',D(:,1),'YData',D(:,7)); set(h2,'XData',D(:,1),'YData',D(:,5));
                set(h4,'XData',D(:,1),'YData',D(:,8)); set(h3,'XData',D(:,1),'YData',D(:,5)-D(:,7)); drawnow limitrate
                fprintf("t=%5.0f s  SP=%4g  PWM=%4.1f  CONTACTO=%6.1f  IR=%6.1f  dif=%5.1f\n",fila(1),fila(8),fila(3),fila(7),fila(5),fila(5)-fila(7));
            end
        end
    elseif startsWith(L,"#")
        fprintf("%s\n",L);
        if contains(L,["SEGURIDAD","ERROR"]), warning("El ESP32 detuvo el proceso."); break; end
    end
end
writeline(s,"S"); fclose(fid); clear s
datos = array2table(D,'VariableNames',{'t_pc_s','t_esp_ms','pwm_pct','T_contacto_firmware','T_ir_obj','T_ir_amb','T_contacto','SP'});

% ---- Resultados por escalon (promedio de los ultimos 60 s de cada uno) ----
R = table();
for i = 1:numel(ESCALONES)
    k = datos.t_pc_s > i*T_PASO - 60 & datos.t_pc_s <= i*T_PASO;
    if nnz(k) < 5, continue; end
    Tc = mean(datos.T_contacto(k)); Ti = median(datos.T_ir_obj(k)); Ta = mean(datos.T_ir_amb(k));
    eps_est = ((Ti+273.15)^4 - (Ta+273.15)^4) / ((Tc+273.15)^4 - (Ta+273.15)^4);   % emisividad aparente
    R = [R; table(ESCALONES(i), Tc, Ti, Ti-Tc, std(datos.T_contacto(k)), eps_est, ...
        'VariableNames',{'SP_C','T_contacto_C','T_IR_C','Error_IR_C','Std_contacto_C','Emisividad_aprox'})]; %#ok<AGROW>
end
if ~isempty(R)
    rmse = sqrt(mean(R.Error_IR_C.^2));
    p = polyfit(R.T_IR_C, R.T_contacto_C, 1);           % correccion lineal: T_real = p1*T_IR + p2
    disp(R)
    fprintf("RMSE IR vs contacto: %.2f °C | sesgo medio: %+.2f °C | emisividad media aprox: %.3f\n", rmse, mean(R.Error_IR_C), mean(R.Emisividad_aprox));
    fprintf("Correccion lineal sugerida: T_real = %.4f * T_IR + %.3f\n", p(1), p(2));
    writetable(R, base + "_resultados.csv");
    save(base + ".mat","datos","D","R","rmse","p","MATERIAL","ESCALONES","T_PASO");
else
    save(base + ".mat","datos","D","MATERIAL","ESCALONES","T_PASO");
end
if isvalid(fig)
    savefig(fig, base + ".fig"); try, theme(fig,"light"); catch, end
    exportgraphics(fig, base + ".png", 'Resolution', 200);
end
fprintf("Listo: todo guardado en 05_datos_ensayos\\%s\n", carpeta);
fprintf("RECUERDA: volver a conectar la termocupla original de la camara.\n");
diary off
