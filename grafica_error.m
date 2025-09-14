clc; clear; close all;

% === Cargar el archivo ===
filename = 'Phi_deseada_vs_Phi_motor.txt';

% Saltamos la primera fila porque es encabezado
data = readmatrix(filename, 'NumHeaderLines', 1);

% Separar vectores
t         = data(:,1);            % tiempo
Phi_d     = data(:,2:4);          % ángulos deseados (phid_1, phid_2, phid_3)
Phi_motor = data(:,5:7);          % ángulos medidos (phim_1, phim_2, phim_3)

% === Calcular error ===
err = Phi_d - Phi_motor;   % (N x 3)

% === RMSE por ventanas ===
window = 1000;                          % tamaño de ventana
N = floor(size(err,1)/window);          % número de ventanas completas
rmse_win = zeros(N,3);                  % almacenar RMSE

for k = 1:N
    idx = (k-1)*window + (1:window);    % índices de la ventana
    seg = err(idx,:);                   % error en esa ventana
    rmse_win(k,:) = sqrt(mean(seg.^2,1));
end

% === Graficar RMSE por ventana ===
figure;
plot(rmse_win, 'LineWidth', 1.5);
xlabel('Ventana (#)');
ylabel('RMSE (grados)');
title('RMSE de cada motor en ventanas de 1000 muestras');
legend('Motor 1','Motor 2','Motor 3');
grid on;

% === Opcional: graficar error en el tiempo ===
figure;
for i=1:3
    subplot(3,1,i);
    plot(t, err(:,i));
    ylabel(sprintf('Error motor %d (°)',i));
    if i==1, title('Error instantáneo (Phi_d - Phi_motor)'); end
end
xlabel('Tiempo (s)');
grid on;
