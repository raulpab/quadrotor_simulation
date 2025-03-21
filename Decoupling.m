function [U]=Decoupling(x1,x2,x3,x4)

ux=x1;
uy=x2;
uz=x3;
ut=x4;
m=1.4;
g=9.8;
psid=0;
    u1=m*sqrt(ux.^2+uy.^2+(uz-g).^2);
    % Phi_d y Theta_d
    phid=asin((-m*(ux*sin(psid)-uy*cos(psid)))/(u1));
    thetad=atan((ux*cos(psid)+uy*sin(psid))/(uz-g));

    
        
U=[u1,phid,thetad];