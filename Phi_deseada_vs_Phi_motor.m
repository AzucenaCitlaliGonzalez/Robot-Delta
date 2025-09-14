% Extraer el vector de tiempo
t = out.tout;  % (21846x1)

% Extraer los datos de phi_d y reordenarlos
Phi_d_raw = out.phi_d_log;     % (3x1x21846)
Phi_d = squeeze(Phi_d_raw)';   % Resultado: (21846x3)
Phi_motor= out.Phi_motor;


% Unir tiempo y ángulos en una sola matriz
data_to_save = [t, Phi_d,Phi_motor];     % (21846x4)
% Definir encabezado
header = 'Tiempo,phid_1,phid_2,phid_3,phim_1,phim_2,phim_3';
% Nombre del archivo
filename = 'Phi_deseada_vs_Phi_motor.txt';
% Abrir archivo para escritura
fid = fopen(filename, 'w');
% Escribir encabezado
fprintf(fid, '%s\n', header);

% Escribir los datos, separados por comas
fclose(fid); % Primero cierra para evitar conflicto con writematrix
writematrix(data_to_save, filename, 'WriteMode', 'append');

disp(['Datos guardados en: ', filename]);

% Calcular error cuadrático medio
rmse = sqrt(mean((Phi_d-Phi_motor).^2,1));
disp(rmse);
