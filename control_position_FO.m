function [U]=control_position_FO(x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19,x20,x21,x22,x23,x24,x25,x26,x27,x28,x29)
%% Parametros Quadrotor
    mq1=1.4;
    g=9.8;
    Jx=0.0219;
    Jy=0.0219;
    Jz=0.0366;
    m=mq1;  
%% Estados
x = x1; x_dot = x4;
y = x2; y_dot = x5;
z = x3; z_dot = x6;

phi = x7;   phi_dot = x10;
theta = x8; theta_dot = x11;
psi = x9;   psi_dot = x12;

%% Posiciones deseadas
    xd=x13; %x
    xd_dot=x14;
    xd_dotdot=0;
    yd=x15;  %y
    yd_dot=x16;
    yd_dotdot=0;
    zd=x17;  %z
    zd_dot=x18;
    zd_dotdot=0;
    psid=x19;   %yaw
    psid_dot=x20;
    psid_dotdot=0;
    
%% POSITION
   %% ERRORES POSICION
    ex=(x-xd);
    ey=(y-yd);
    ez=(z-zd);
    
    ex_dot=(x_dot-xd_dot);
    ey_dot=(y_dot-yd_dot);
    ez_dot=(z_dot-zd_dot);
    
    
    %% parameters
    cx=2.03;
    cy=2.03;
    cz=2.03;
    c2=2;
    c3=0.8;
    k1x_est=x24;
    k2x_est=x25;
    k1y_est=x26;
    k2y_est=x27;
    k1z_est=x28;
    k2z_est=x29;
    %% design
    Ix_fo=(0.05*(abs(ex)^0.3)*tanh(ex));
    Iy_fo=(0.05*(abs(ey)^0.3)*tanh(ey));
    Iz_fo=(0.05*(abs(ez)^0.3)*tanh(ez));

    sx=ex_dot + cx*ex + x21;
    sy=ey_dot + cy*ey + x22;
    sz=ez_dot + cz*ez + x23;
    
    ux=(xd_dotdot  - cx*ex_dot - k2x_est*(abs(sx)^0.2)*tanh(sx) - k1x_est*sx  -Ix_fo);
    uy=(yd_dotdot  - cy*ey_dot - k2y_est*(abs(sy)^0.2)*tanh(sy) - k1y_est*sy  -Iy_fo);
    uz=(zd_dotdot  - cz*ez_dot - k2z_est*(abs(sz)^0.2)*tanh(sz) - k1z_est*sz  -Iz_fo);
    u1=m*sqrt(ux.^2+uy.^2+(uz-g).^2);
    
    
U=[ux,uy,uz,u1,ex,ey,ez,sx,sy,sz];
end