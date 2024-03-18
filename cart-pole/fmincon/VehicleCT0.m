function [dxdt, y] = VehicleCT0(x, u)
% %% parameters
 mCart = 1;  % cart mass
 mPend = 1;  % pendulum mass
 g = 9.81;   % gravity of earth
 L = 0.5;    % pendulum length
 Kd = 10;    % cart damping
 %% Obtain x, u and y
 z_dot = x(2);
 theta = x(3);
 theta_dot = x(4);
 F = u;
 y = x;

 %% Cartpole Model
 dxdt = x;
 dxdt(1) = z_dot;
 dxdt(2) = (F - (Kd*z_dot) - (mPend*L*(theta_dot^2)*sin(theta)) + (mPend*g*sin(theta)*cos(theta))) / (mCart + (mPend*sin(theta)^2));
 dxdt(3) = theta_dot;
 dxdt(4) = ( ( (F-(Kd*z_dot)-(mPend*L*(theta_dot^2)*sin(theta))) * cos(theta) ) +((mCart + mPend)*g*sin(theta))) / ((L*(mPend+mCart)) - (mPend*L*cos(theta)^2));

 
 
 

 %% Vehicle Model
 %{
%% parameter definition
mv = 1530;  %vehicle mass (kg)
Iz = 1536.7;  %yaw inertia
hg = 0.52;   %C.G. height
lf = 1.11;     %m
lr = 2.78-1.11;
L = 2.78;  %wheelbase
ls = 1.55/2;   %vehicle track
g = 9.8;    % gravaty constant
Vx = 25;
lambda=0.0000001;
mu=1;
ay=0;
Vy = x(1);
r = x(2);
%% normal forces considering both longitudial and lateral transfer, positive direction of ay is leftward and ax is forward
Fz0 = mv*g/4;
Fz_FL = Fz0+mv*hg*(-ay*lr)/(2*L*ls);
Fz_FR = Fz0+mv*hg*(ay*lr)/(2*L*ls);
Fz_RL = Fz0+mv*hg*(-ay*lf)/(2*L*ls);
Fz_RR = Fz0+mv*hg*(ay*lf)/(2*L*ls);
%% tire slip angle
% alpha_FL = -(atan((Vy+lf*r)/Vx)-delta);  % slip angle
% alpha_FR = -(atan((Vy+lf*r)/Vx)-delta);  % slip angle
% alpha_RL = atan(-(Vy-lr*r)/Vx);  % slip angle
% alpha_RR = atan(-(Vy-lr*r)/Vx);  % slip angle
alpha_FL = delta_f-atan((Vy+lf*r)./(Vx-r*ls));  % modified slip angle
alpha_FR = delta_f-atan((Vy+lf*r)./(Vx+r*ls));  % modified slip angle
alpha_RL = -atan((Vy-lr*r)./(Vx-r*ls));  % modified slip angle
alpha_RR = -atan((Vy-lr*r)./(Vx+r*ls));  % modified slip angle

x_fl=[lambda,alpha_FL,Vx,Fz_FL,1/mu];
x_fr=[lambda,alpha_FR,Vx,Fz_FR,1/mu];
x_rl=[lambda,alpha_RL,Vx,Fz_RL,1/mu];
x_rr=[lambda,alpha_RR,Vx,Fz_RR,1/mu];

F_fl=LuGre_2D_ss(x_fl);
F_fr=LuGre_2D_ss(x_fr);
F_rl=LuGre_2D_ss(x_rl);
F_rr=LuGre_2D_ss(x_rr);

Fyfl=F_fl(2);
Fyfr=F_fr(2);
Fyrl=F_rl(2);
Fyrr=F_rr(2);

F=u;
y=x;
%% vehicle 2D motion model, Compute dxdt
dxdt = x;
% z_dot
dxdt(1) = ((Fyfl+Fyfr)*cos(delta_f)+Fyrl+Fyrr+F)/mv-Vx*r;
% z_dot_dot
dxdt(2) = (lf*(Fyfl+Fyfr)*cos(delta_f)+ls*(Fyfl-Fyfr)*sin(delta_f)-lr*(Fyrl+Fyrr))/Iz;
%}


















