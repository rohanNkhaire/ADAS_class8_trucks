function x_next = VehicleDT0(xk, uk)
%% Discrete-time nonlinear dynamic model of a pendulum on a cart at time k
    x=xk;
    u=uk(1);
    mCart = 1;  % cart mass
    mPend = 1;  % pendulum mass
    g = 9.81;   % gravity of earth
    L = 0.5;    % pendulum length
    Kd = 10;    % cart damping
    dt = 0.1;
    f = @(x,u) [x(2);...
        (u - (Kd*x(2)) - (mPend*L*(x(4)^2)*sin(x(3))) + (mPend*g*sin(x(3))*cos(x(3)))) / (mCart + (mPend*sin(x(3))^2));
        x(4);...
        ((u - (Kd*x(2)) - (mPend*L*(x(4)^2)*sin(x(3))))*cos(x(3))+((mCart + mPend)*g*sin(x(3)))) / ((L*(mPend+mCart)) - (mPend*L*cos(x(3))^2));];
    
   % Runge-Kutta 4 integration
   k1 = f(x,         u);
   k2 = f(x+dt/2*k1, u);
   k3 = f(x+dt/2*k2, u);
   k4 = f(x+dt*k3,   u);
   x_next = x + dt/6*(k1+2*k2+2*k3+k4);
%{
% Repeat application of Euler method sampled at Ts/M.
    M = 10;
    Ts = 0.1;
    delta = Ts/M;
    xk1 = xk;
    for ct=1:M
        xk1 = xk1 + delta*VehicleCT0(xk1,uk);
    end
%}
end