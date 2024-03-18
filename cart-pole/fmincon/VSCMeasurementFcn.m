function y = VSCMeasurementFcn(xk)
%% Discrete-time nonlinear dynamic model of a pendulum on a cart at time k
y = [xk(1);xk(3)];