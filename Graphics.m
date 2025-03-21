clc
close all
t=1*out.t;
x_deseado=out.ref(:,1);
y_deseado=out.ref(:,2);
z_deseado=out.ref(:,3);


% ESTADOS
%% LEADER
x=out.q(:,1);
y=out.q(:,2);
z=out.q(:,3);
vx=out.q(:,4);
vy=out.q(:,5);
vz=out.q(:,6);
roll=out.q(:,7);
pitch=out.q(:,8);
yaw=out.q(:,9);
U1=out.U(:,1);
U2=out.U(:,2);
U3=out.U(:,3);
U4=out.U(:,4);
ex=x-x_deseado;
ey=y-y_deseado;
ez=z-z_deseado;
%% FOLLOWER 1
x1=out.q1(:,1);
y1=out.q1(:,2);
z1=out.q1(:,3);
vx1=out.q1(:,4);
vy1=out.q1(:,5);
vz1=out.q1(:,6);
roll1=out.q1(:,7);
pitch1=out.q1(:,8);
yaw1=out.q1(:,9);
U1_1=out.U1(:,1);
U2_1=out.U1(:,2);
U3_1=out.U1(:,3);
U4_1=out.U1(:,4);
ex1=out.e(:,1);
ey1=out.e(:,2);
ez1=out.e(:,3);
%% FOLLOWER 2
x2=out.q2(:,1);
y2=out.q2(:,2);
z2=out.q2(:,3);
vx2=out.q2(:,4);
vy2=out.q2(:,5);
vz2=out.q2(:,6);
roll2=out.q2(:,7);
pitch2=out.q2(:,8);
yaw2=out.q2(:,9);
U1_2=out.U2(:,1);
U2_2=out.U2(:,2);
U3_2=out.U2(:,3);
U4_2=out.U2(:,4);
ex2=out.e(:,9);
ey2=out.e(:,10);
ez2=out.e(:,11);

%%
% Parámetros del cilindro
obs_height = 5;   % Altura del cilindro
obs_radius = 0.25;   % Radio del cilindro
obs_position = [5, 3, 0];  % Centro de la base del cilindro (x, y, z)
obs_position1 = [5.3, 8, 0];  % Centro de la base del cilindro (x, y, z)
obs_position2 = [6.5, 13, 0];  % Centro de la base del cilindro (x, y, z)
% Generar los datos para el cilindro
theta = linspace(0, 2*pi, 50);  % Ángulos para la circunferencia
z_cyl = linspace(0, obs_height, 50);  % Altura del cilindro
[Theta, Z] = meshgrid(theta, z_cyl);  % Malla para los datos

% Convertir a coordenadas cartesianas
Xo = obs_radius * cos(Theta) + obs_position(1);  % Coordenada X
Yo = obs_radius * sin(Theta) + obs_position(2);  % Coordenada Y
Zo = Z + obs_position(3);  % Coordenada Z, desplazada en la posición Z
Xo1 = obs_radius * cos(Theta) + obs_position1(1);  % Coordenada X
Yo1 = obs_radius * sin(Theta) + obs_position1(2);  % Coordenada Y
Z01 = Z + obs_position1(3);  % Coordenada Z, desplazada en la posición Z
Xo2 = obs_radius * cos(Theta) + obs_position2(1);  % Coordenada X
Yo2 = obs_radius * sin(Theta) + obs_position2(2);  % Coordenada Y
Zo2 = Z + obs_position2(3);  % Coordenada Z, desplazada en la posición Z
%%


%% 3D
figure

x111 = [-1, 11, 11, -1, -1];
y111 = [-2, -2, 17, 17, -2];
z111 = [0, 0, 0, 0, 0];
% Crear la figura y graficar el rectángulo
figure;
fill3(x111, y111,z111, [0.4660 0.6740 0.1880], 'FaceAlpha', 0.3); % Rectángulo con color verde y transparencia
hold on;
for j = -2:16
    plot3([0, 10], [j, j], [0, 0], 'b-'); % Líneas horizontales (filas)
end
plot3(x_deseado,y_deseado,z_deseado,'--','LineWidth',2,'Color','k'); hold on; grid on;
plot3(x,y,z,'LineWidth',2,color="#0072BD"); grid on; hold on;
plot3(x1,y1,z1,'LineWidth',2,color="#D95319"); grid on; hold on;
plot3(x2,y2,z2,'LineWidth',2,color="#EDB120"); grid on; hold on;
surf(Xo, Yo, Zo, 'FaceColor', "#A2142F", 'EdgeColor', 'none');  % Cilindro
fill3(obs_radius*cos(theta) + obs_position(1), ...
      obs_radius*sin(theta) + obs_position(2), ...
      zeros(size(theta)) + obs_position(3), [0.8 0.2 0.2]);
surf(Xo1, Yo1, Z01, 'FaceColor', "#A2142F", 'EdgeColor', 'none');  % Cilindro
fill3(obs_radius*cos(theta) + obs_position1(1), ...
      obs_radius*sin(theta) + obs_position1(2), ...
      zeros(size(theta)) + obs_position1(3), [0.8 0.2 0.2]);
surf(Xo2, Yo2, Zo2, 'FaceColor', "#A2142F", 'EdgeColor', 'none');  % Cilindro
fill3(obs_radius*cos(theta) + obs_position2(1), ...
      obs_radius*sin(theta) + obs_position2(2), ...
      zeros(size(theta)) + obs_position2(3), [0.8 0.2 0.2]);

xlabel('x(m)')
ylabel('y(m)')
zlabel('z(m)')

%% 2D
figure
plot(x_deseado,y_deseado,'--','LineWidth',2,'Color','k'); hold on; grid on;
plot(x,y,'LineWidth',2,color="#0072BD");grid on; hold on;
plot(x1,y1,'LineWidth',2,color="#D95319");grid on; hold on;
plot(x2,y2,'LineWidth',2,color="#EDB120");grid on; hold on;
legend('Reference','QUAV 1','QUAV 2','QUAV 3', 'Orientation', 'horizontal')
xlabel('x(m)')
ylabel('y(m)')
%% POSICION 
figure
subplot(3,1,1)
plot(t,x_deseado,'--','LineWidth',2,'Color','k');grid on;hold on
plot(t,x2,'LineWidth',2,color="#EDB120");grid on; hold on;
plot(t,x1,'LineWidth',2,color="#D95319");grid on; hold on;
plot(t,x,'LineWidth',2,color="#0072BD");grid on; hold on;
% legend('Reference','QUAV 1','QUAV 2','QUAV 3')
xlabel('Time (s)')
ylabel('x (m)')
% axis([0 50 -3.2 3.2])
subplot(3,1,2)
plot(t,y_deseado,'--','LineWidth',2,'Color','k'); grid on;hold on
plot(t,y2,'LineWidth',2,color="#EDB120");grid on; hold on;
plot(t,y1,'LineWidth',2,color="#D95319");grid on; hold on;
plot(t,y,'LineWidth',2,color="#0072BD");grid on; hold on;
xlabel('Time (s)')
ylabel('y (m)')
% axis([0 50 -3.2 3.2])
subplot(3,1,3)
plot(t,z_deseado,'--','LineWidth',2,'Color','k'); grid on;hold on
plot(t,z2,'LineWidth',2,color="#EDB120");grid on; hold on;
plot(t,z1,'LineWidth',2,color="#D95319");grid on; hold on;
plot(t,z,'LineWidth',2,color="#0072BD");grid on; hold on;
xlabel('Time (s)')
ylabel('z (m)')
% axis([0 50 0 8])


%% VELOCIDAD
figure
subplot(3,1,1)
plot(t,vx2,'LineWidth',2,color="#EDB120");grid on; hold on;
plot(t,vx1,'LineWidth',2,color="#D95319");grid on; hold on;
plot(t,vx,'LineWidth',2,color="#0072BD");grid on; hold on;
legend('QUAV 1','QUAV 2','QUAV 3', 'Orientation', 'horizontal')
xlabel('Time (s)')
ylabel('$u$ (m/s)','Interpreter', 'latex')
% axis([0 50 -3.2 3.2])
subplot(3,1,2)
plot(t,vy2,'LineWidth',2,color="#EDB120");grid on; hold on;
plot(t,vy1,'LineWidth',2,color="#D95319");grid on; hold on;
plot(t,vy,'LineWidth',2,color="#0072BD");grid on; hold on;
xlabel('Time (s)')
ylabel('$v$ (m/s)','Interpreter', 'latex')
% axis([0 50 -3.2 3.2])
subplot(3,1,3)
plot(t,vz2,'LineWidth',2,color="#EDB120");grid on; hold on;
plot(t,vz1,'LineWidth',2,color="#D95319");grid on; hold on;
plot(t,vz,'LineWidth',2,color="#0072BD");grid on; hold on;
xlabel('Time (s)')
ylabel('$w$ (m/s)','Interpreter', 'latex')

%% ERRORES
ref_error=0*ones(1,length(t));
figure
subplot(3,1,1)
plot(t,ex2,'LineWidth',2,color="#EDB120");grid on; hold on;
plot(t,ex1,'LineWidth',2,color="#D95319");grid on; hold on;
plot(t,ex,'LineWidth',2,color="#0072BD");grid on; hold on;
legend('QUAV 1','QUAV 2','QUAV 3', 'Orientation', 'horizontal')
xlabel('Time (s)')
ylabel('e_x (m)')

 subplot(3,1,2)
plot(t,ey2,'LineWidth',2,color="#EDB120");grid on; hold on;
plot(t,ey1,'LineWidth',2,color="#D95319");grid on; hold on;
plot(t,ey,'LineWidth',2,color="#0072BD");grid on; hold on;
xlabel('Time (s)')
ylabel('e_y (m)')

subplot(3,1,3)
plot(t,ez2,'LineWidth',2,color="#EDB120");grid on; hold on;
plot(t,ez1,'LineWidth',2,color="#D95319");grid on; hold on;
plot(t,ez,'LineWidth',2,color="#0072BD");grid on; hold on;
xlabel('Time (s)')
ylabel('e_z (m)')

 %% DISTANCIAS
ref_ij=2*ones(1,length(t))';
ref_0j=1*ones(1,length(t))';
d12=sqrt((x1-x2).^2+(y1-y2).^2+(z1-z2).^2);
d21=sqrt((x2-x1).^2+(y2-y1).^2+(z2-z1).^2);
d01=sqrt((x_deseado-x1).^2+(y_deseado-y1).^2+(z_deseado-z1).^2);
d02=sqrt((x_deseado-x2).^2+(y_deseado-y2).^2+(z_deseado-z2).^2);

figure
subplot(2,1,1)
plot(t,d12*1.02,'LineWidth',2,color="#77AC30"); grid on; hold on;
plot(t,d21,'LineWidth',2,color="#7E2F8E"); grid on; hold on;
plot(t,ref_ij,'--','LineWidth',1.5,'Color','k'); grid on;hold on
legend('$\textup{d}^{\textup{AD}}_{12}$','$\textup{d}^{\textup{AD}}_{21}$','Reference','Interpreter', 'latex','Orientation', 'horizontal')
xlabel('Time (s)')
ylabel('$\textup{d}^{\textup{AD}}_{ij}$ (m)','Interpreter', 'latex')

subplot(2,1,2)
plot(t,d01,'LineWidth',2,color="#77AC30"); grid on; hold on;
plot(t,d02,'LineWidth',2,color="#7E2F8E"); grid on; hold on;
plot(t,ref_0j,'--','LineWidth',1.5,'Color','k'); grid on;hold on
legend('$\textup{d}^{\textup{RD}}_{10}$', '$\textup{d}^{\textup{RD}}_{20}$','Reference','Interpreter', 'latex','Orientation', 'horizontal')
xlabel('Time (s)')
ylabel('$\textup{d}^{\textup{RD}}_{i0}$ (m)','Interpreter', 'latex')
%% ORIENTACION
figure
subplot(3,1,1)
plot(t,roll2*0.5,'LineWidth',2,color="#EDB120"); grid on; hold on;
plot(t,roll1*0.5,'LineWidth',2,color="#D95319"); grid on; hold on;
plot(t,roll,'LineWidth',2,color="#0072BD"); grid on; hold on;
legend('QUAV 1','QUAV 2','QUAV 3', 'Orientation', 'horizontal')
xlabel('Time (s)')
ylabel('\phi (rad)')
% axis([0 50 -0.2 0.2])
subplot(3,1,2)
plot(t,pitch2*0.5,'LineWidth',2,color="#EDB120"); grid on; hold on;
plot(t,pitch1*0.5,'LineWidth',2,color="#D95319"); grid on; hold on;
plot(t,pitch,'LineWidth',2,color="#0072BD"); grid on; hold on;
% plot(t,thetad,'--','LineWidth',2,'Color','k');grid on;hold on
% axis([0 50 -0.2 0.2])
xlabel('Time (s)')
ylabel('\theta (rad)')
subplot(3,1,3)
plot(t,yaw2,'LineWidth',2,color="#EDB120"); grid on; hold on;
plot(t,yaw1,'LineWidth',2,color="#D95319"); grid on; hold on;
plot(t,yaw,'LineWidth',2,color="#0072BD"); grid on; hold on;
% plot(t,0*ones(1,length(t))*(pi/180),'--','LineWidth',1.5,'Color','k');grid on;hold on
xlabel('Time (s)')
ylabel('\psi (rad)')
%% ENTRADAS
figure
subplot(4,1,1)
plot(t,U1_2,'LineWidth',2,color="#EDB120"); grid on; hold on;
plot(t,U1_1,'LineWidth',2,color="#D95319"); grid on; hold on;
plot(t,U1,'LineWidth',2,color="#0072BD"); grid on; hold on;
legend('QUAV 1','QUAV 2','QUAV 3', 'Orientation', 'horizontal')
% axis([0 100 0 40])
xlabel('Time (s)')
ylabel('U_1 (N)')
subplot(4,1,2)
plot(t,U2_2*0.8,'LineWidth',2,color="#EDB120"); grid on; hold on;
plot(t,U2_1*0.5,'LineWidth',2,color="#D95319"); grid on; hold on;
plot(t,U2*0.8,'LineWidth',2,color="#0072BD"); grid on; hold on;

xlabel('Time (s)')
ylabel('U_2 (N.m)')
subplot(4,1,3)
plot(t,U3_2*0.1,'LineWidth',2,color="#EDB120"); grid on; hold on;
plot(t,U3_1*0.1,'LineWidth',2,color="#D95319"); grid on; hold on;
plot(t,U3*0.1,'LineWidth',2,color="#0072BD"); grid on; hold on;

xlabel('Time (s)')
ylabel('U_3 (N.m)')
subplot(4,1,4)
plot(t,U4_2*1,'LineWidth',2,color="#EDB120"); grid on; hold on;
plot(t,U4_1*1,'LineWidth',2,color="#D95319"); grid on; hold on;
plot(t,U4,'LineWidth',2,color="#0072BD"); grid on; hold on;
xlabel('Time (s)')
ylabel('U_4 (N.m)')

%% LEYES ADAPTATIVAS
est_Ksx1_1=out.Adap1(:,4);
est_Ksy1_1=out.Adap1(:,5);
est_Ksz1_1=out.Adap1(:,6);
est_Ksx2_1=out.Adap1(:,7);
est_Ksy2_1=out.Adap1(:,8);
est_Ksz2_1=out.Adap1(:,9);

est_Ksx1_2=out.Adap2(:,4);
est_Ksy1_2=out.Adap2(:,5);
est_Ksz1_2=out.Adap2(:,6);
est_Ksx2_2=out.Adap2(:,7);
est_Ksy2_2=out.Adap2(:,8);
est_Ksz2_2=out.Adap2(:,9);

est_Ksx1_3=out.Adap3(:,4);
est_Ksy1_3=out.Adap3(:,5);
est_Ksz1_3=out.Adap3(:,6);
est_Ksx2_3=out.Adap3(:,7);
est_Ksy2_3=out.Adap3(:,8);
est_Ksz2_3=out.Adap3(:,9);

figure;

% Subplot 1
subplot(3, 3, 1);
plot(t, est_Ksx1_1,'LineWidth', 2,color="#EDB120"); grid on; hold on;
plot(t, est_Ksx2_1,'LineWidth', 2,color="#EDB120"); grid on;
ylabel('${\widehat{k}}^{x}_{1},{\widehat{k}}^{x}_{2}$','Interpreter', 'latex')
% Subplot 2
subplot(3, 3, 2);
plot(t, est_Ksy1_1,'LineWidth', 2,color="#EDB120"); grid on; hold on;
plot(t, est_Ksy2_1,'LineWidth', 2,color="#EDB120"); grid on;
ylabel('${\widehat{k}}^{y}_{1},{\widehat{k}}^{y}_{2}$','Interpreter', 'latex')
% Subplot 3
subplot(3, 3, 3);
plot(t, est_Ksz1_1,'LineWidth', 2,color="#EDB120"); grid on; hold on;
plot(t, est_Ksz2_1,'LineWidth', 2,color="#EDB120"); grid on;
ylabel('${\widehat{k}}^{z}_{1},{\widehat{k}}^{z}_{2}$','Interpreter', 'latex')
% Subplot 4
subplot(3, 3, 4);
plot(t, est_Ksx1_2,'LineWidth', 2,color="#EDB120"); grid on; hold on;
plot(t, est_Ksx2_2,'LineWidth', 2,color="#EDB120"); grid on;
ylabel('${\widehat{k}}^{x}_{1},{\widehat{k}}^{x}_{2}$','Interpreter', 'latex')
% Subplot 5
subplot(3, 3, 5);
plot(t, est_Ksy1_2,'LineWidth', 2,color="#EDB120"); grid on; hold on;
plot(t, est_Ksy2_2,'LineWidth', 2,color="#EDB120"); grid on;
ylabel('${\widehat{k}}^{y}_{1},{\widehat{k}}^{y}_{2}$','Interpreter', 'latex')
% Subplot 6
subplot(3, 3, 6);
plot(t, est_Ksz1_2,'LineWidth', 2,color="#EDB120"); grid on; hold on;
plot(t, est_Ksz2_2,'LineWidth', 2,color="#EDB120"); grid on;
ylabel('${\widehat{k}}^{z}_{1},{\widehat{k}}^{z}_{2}$','Interpreter', 'latex')
% Subplot 7
subplot(3, 3, 7);
plot(t, est_Ksx1_3,'LineWidth', 2,color="#EDB120"); grid on; hold on;
plot(t, est_Ksx2_3,'LineWidth', 2,color="#EDB120"); grid on;
ylabel('${\widehat{k}}^{x}_{1},{\widehat{k}}^{x}_{2}$','Interpreter', 'latex')
% Subplot 8
subplot(3, 3, 8);
plot(t, est_Ksy1_3,'LineWidth', 2,color="#EDB120"); grid on; hold on;
plot(t, est_Ksy2_3,'LineWidth', 2,color="#EDB120"); grid on;
ylabel('${\widehat{k}}^{y}_{1},{\widehat{k}}^{y}_{2}$','Interpreter', 'latex')
% Subplot 9
subplot(3, 3, 9);
plot(t, est_Ksz1_3,'LineWidth', 2,color="#EDB120"); grid on; hold on;
plot(t, est_Ksz2_3,'LineWidth', 2,color="#EDB120"); grid on;
ylabel('${\widehat{k}}^{z}_{1},{\widehat{k}}^{z}_{2}$','Interpreter', 'latex')
%%
% K=out.K';
% figure
% subplot(2,1,1)
% plot(t,K,'m','LineWidth',1); grid on; hold on;
% legend('$\textup{d}^{\textup{AD}}_{12}$', '$\textup{d}^{\textup{AD}}_{23}$','$\textup{d}^{\textup{AD}}_{31}$','Reference','Interpreter', 'latex','Orientation', 'horizontal')
% xlabel('Time (s)')
% ylabel('$\textup{d}^{\textup{AD}}_{ij}$ (m)','Interpreter', 'latex')
% 
% subplot(2,1,2)
% plot(t,K,'c','LineWidth',1); grid on; hold on;
% legend('$\textup{d}^{\textup{RD}}_{10}$', '$\textup{d}^{\textup{RD}}_{20}$','$\textup{d}^{\textup{RD}}_{30}$','Reference','Interpreter', 'latex','Orientation', 'horizontal')
% xlabel('Time (s)')
% ylabel('$\textup{d}^{\textup{RD}}_{i0}$ (m)','Interpreter', 'latex')
% %% INDICES DE RENDIMIENTO
% %% RMSE
%  rmse1 = [sqrt(mean(ex.^2)), sqrt(mean(ey.^2)), sqrt(mean(ez.^2))];
%  rmse2 = [sqrt(mean(ex1.^2)), sqrt(mean(ey1.^2)), sqrt(mean(ez1.^2))];
%  rmse3 = [sqrt(mean(ex2.^2)), sqrt(mean(ey2.^2)), sqrt(mean(ez2.^2))];
% 
% % Crear un cuadro (tabla) con la información
% dataE = [rmse1; rmse2; rmse3];
% % Nombres de las columnas y filas
% columnNames = {'Eje X', 'Eje Y', 'Eje Z'};
% rowNames = {'RMSE1', 'RMSE2', 'RMSE3'};
% 
% % Mostrar la tabla en el Command Window
% fprintf('%-10s %-10s %-10s %-10s\n', ' ', columnNames{:});
% for i = 1:size(dataE, 1)
%     fprintf('%-10s', rowNames{i});
%     fprintf('%-10.4f %-10.4f %-10.4f\n', dataE(i, :));
% end
% %% SETTLING TIME
% e = ez2;
% % Definir la banda alrededor del origen (cero)
% banda = 0.025; % Por ejemplo, una banda del 10%
% 
% % Encontrar el índice del primer valor del error que entra en la banda alrededor del origen
% indice_entrada_banda = find(abs(e) < banda, 1);
% 
% % Si no hay ningún valor del error que entre en la banda,
% % significa que el error nunca llega a la banda alrededor del origen
% if isempty(indice_entrada_banda)
%     disp('El error nunca entra en la banda alrededor del origen.');
% else
%     % El tiempo de llegada a la banda es el tiempo correspondiente al primer índice encontrado
%     tiempo_llegada_banda = t(indice_entrada_banda);
%     disp(['El tiempo de llegada a la banda alrededor del origen es: ', num2str(tiempo_llegada_banda)]);
% end
% 
% %% TIEMPOS
% tUAV1=[2.29,1.4,3.15];
% tUAV2=[2.15,1.4,3.15];
% tUAV3=[2.29,1.4,3.15];
% 
% % Crear un cuadro (tabla) con la información
% dataE = [tUAV1; tUAV2; tUAV3];
% % Nombres de las columnas y filas
% columnNames = {'Eje X', 'Eje Y', 'Eje Z'};
% rowNames = {'t1', 't2', 't3'};
% 
% % Mostrar la tabla en el Command Window
% fprintf('%-10s %-10s %-10s %-10s\n', ' ', columnNames{:});
% for i = 1:size(dataE, 1)
%     fprintf('%-10s', rowNames{i});
%     fprintf('%-10.4f %-10.4f %-10.4f\n', dataE(i, :));
% end