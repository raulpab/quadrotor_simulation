function [dx]=Dynamic_UAV2(x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19)
%% Parametros del Quadrotor UAV
    mq=1.4;
    g=9.8;
    Jx=0.0219;
    Jy=0.0219;
    Jz=0.0366;
%   dis=[x17,x18,x19];

%% Estados
xyz=[x1;x2;x3];
xyz_dot=[x4;x5;x6];
rpy=[x7;x8;x9];
rpy_dot=[x10;x11;x12];

x = x1; x_dot = x4;
y = x2; y_dot = x5;
z = x3; z_dot = x6;

phi = x7;   phi_dot = x10;
theta = x8; theta_dot = x11;
psi = x9;   psi_dot = x12;

%% Entradas
% u=[U1,U2,U3,U4]
Uq1=[x13;x14;x15;x16];
ux=(cos(x7)*sin(x8)*cos(x9)+sin(x7)*sin(x9));
uy=(cos(x7)*sin(x8)*sin(x9)-sin(x7)*cos(x9));

%% Aceleraciones
x_dotdot=-(cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi))*(Uq1(1)/mq) +x17;
y_dotdot=-(cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi))*(Uq1(1)/mq) +x18;
z_dotdot=-(cos(phi)*cos(theta))*Uq1(1)/mq+g;
phi_dotdot=((Jy-Jz)/Jx)*theta_dot*psi_dot+Uq1(2)/Jx;
theta_dotdot=((Jz-Jx)/Jy)*phi_dot*psi_dot+Uq1(3)/Jy;
psi_dotdot=((Jx-Jy)/Jz)*theta_dot*phi_dot+Uq1(4)/Jz;

%%
xyz_dotdot=[x_dotdot;y_dotdot;z_dotdot];
rpy_dotdot=[phi_dotdot;theta_dotdot;psi_dotdot];

[dx]=[xyz_dot;
       xyz_dotdot;
       rpy_dot;
       rpy_dotdot];
    

end
