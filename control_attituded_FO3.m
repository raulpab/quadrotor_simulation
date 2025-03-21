function [U]=control_attituded_FO3(x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19,x20,x21,x22,x23,x24,x25,x26,x27,x28,x29,x30,x31)
%% Parametros Quadrotor 1
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
    phid=x21;   %phi
    phid_dot=0;
    phid_dotdot=0;
    thetad=x22;   %theta
    thetad_dot=0;
    thetad_dotdot=0;
     
 
%% ACTITUD

%% functions
    fphi = theta_dot*psi_dot*(Jy - Jz)/Jx;
    gphi = 1/Jx;
    ftheta = phi_dot*psi_dot*(Jz - Jx)/Jy;
    gtheta = 1/Jy;
    fpsi = phi_dot*theta_dot*(Jx - Jy)/Jz;
    gpsi = 1/Jz;
%% ERRORES ACTITUD
    ephi = (phi - phid);
    ephi_dot = (phi_dot - phid_dot);
    etheta = (theta - thetad);
    etheta_dot = (theta_dot - thetad_dot);
    epsi = (psi - psid);
    epsi_dot = (psi_dot - psid_dot);
    
    %% parameters
    cphi=12;
    ctheta=12;
    cpsi=12;
    c2=0.7;
    c3=10;
    k1x_est=x26;
    k2x_est=x27;
    k1y_est=x28;
    k2y_est=x29;
    k1z_est=x30;
    k2z_est=x31;
    %% Design
    Ix_fo=(0.0002*(abs(ephi)^1.2)*tanh(ephi));
    Iy_fo=(0.0002*(abs(etheta)^1.2)*tanh(etheta));
    Iz_fo=(0.0002*(abs(epsi)^1.2)*tanh(epsi));

    sphi=ephi_dot + cphi*ephi + x23;
    stheta=etheta_dot + ctheta*etheta + x24;
    spsi=epsi_dot + cpsi*epsi + x25;
    
    u2=(1/gphi)*(phid_dotdot - fphi - cphi*ephi_dot  - k2x_est*(abs(sphi)^0.2)*tanh(sphi) - k1x_est*sphi - Ix_fo);
    u3=(1/gtheta)*(thetad_dotdot - ftheta - ctheta*etheta_dot  - k2y_est*(abs(stheta)^0.2)*tanh(stheta) - k1y_est*stheta - Iy_fo);
    u4=(1/gpsi)*(psid_dotdot - fpsi - cpsi*epsi_dot  - k2z_est*(abs(spsi)^0.2)*tanh(spsi) - k1z_est*spsi - Iz_fo);

    
U=[u2,u3,u4,ephi,etheta,epsi,sphi,stheta,spsi];