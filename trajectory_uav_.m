% Definir los vértices del cuadrado como waypoints (x, y, z)
waypoints = [
    0, 0, 4;
    10, 0, 4;
    10, 3, 4;
    0, 3, 4;
    0, 6, 4;
    10, 6, 4;
    10, 9, 4;
    0, 9, 4;
    0, 12, 4;
    10, 12, 4;
    10, 15, 4
    0, 15, 4;
    0, 15, 0.5;% Repetir el primer punto para cerrar el cuadrado
];

% Número de puntos entre waypoints para la interpolación (más puntos, más suave la trayectoria)
num_points_between = 50;

% Interpolar la trayectoria linealmente entre los waypoints
traj_x = interp1(1:size(waypoints, 1), waypoints(:, 1), linspace(1, size(waypoints, 1), num_points_between*size(waypoints, 1)));
traj_y = interp1(1:size(waypoints, 1), waypoints(:, 2), linspace(1, size(waypoints, 1), num_points_between*size(waypoints, 1)));
traj_z = interp1(1:size(waypoints, 1), waypoints(:, 3), linspace(1, size(waypoints, 1), num_points_between*size(waypoints, 1)));

total_time = 100; % por ejemplo, 10 segundos
time = linspace(0, total_time, length(traj_x));
% Graficar la trayectoria cuadrada
figure;
plot3(traj_x, traj_y, traj_z, 'b', 'LineWidth', 2);
hold on;
plot3(waypoints(:, 1), waypoints(:, 2), waypoints(:, 3), 'ro-', 'LineWidth', 2);
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');

%%
xd=double([time',traj_x']);
yd=double([time',traj_y']);
zd=double([time',traj_z']);
