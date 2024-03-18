%% Swing-up Control of a Pendulum Using Nonlinear Model Predictive Control
clear
close all
clc
%% Product Requirement
% This example requires Optimization Toolbox(TM) to solve a nonlinear
% programming problem at each control interval.
if ~mpcchecktoolboxinstalled('optim')
    disp('Optimization Toolbox is required to run this example.')
    return
end
tic
%%
% Set the sample time:
Ts = 0.1;
%%
% Set the prediction horizon:
N = 10;
x = [0;0;-pi;0];
uopt = zeros(N,1);
xref = [4;0;0.01;0];
Q = diag([10,1,20,0]);

% R matrix penalizes MV rate of change.  A small value is used here because
% aggressive control is desired.
R = 0.1;

%%
options = optimoptions('fmincon','Algorithm','sqp','Display','none');
Duration = 20;
% Apply the MV constraints because the force has lower and upper bounds.
LB = -100*ones(N,1);
UB = 100*ones(N,1);
% Start simulation
fprintf('Simulation started.  It might take a while...\n')
xHistory = x;
uHistory = uopt(1);
Time=[];
Exit=[];
J_vec=[];
for ct = 1:(20/Ts)
    tic
    % Nonlinear MPC computation with full state feedback (no state estimator)
    COSTFUN = @(u) VSCObjectiveFCN(u,x,Ts,N,xref,uopt(1),Q,R);
    CONSFUN = @(u) VSCConstraintFCN(u,x,Ts,N,xref);
    [uopt, fval, exit] = fmincon(COSTFUN,uopt,[],[],[],[],LB,UB,CONSFUN,options);    
    % Implement first optimal control move and update plant states
    x = VehicleDT0(x, uopt(1));
    % Save plant states for display.
    xHistory = [xHistory x]; 
    Exit=[Exit; exit];
    J_vec=[J_vec; fval];
    uHistory = [uHistory uopt(1)];
    Time=[Time toc];
end
fprintf('Simulation finished!\n')
toc


%%
% Plot the closed-loop response.

%{
%load("Data_files/opt_control_c3.mat")
Opt_ctrl=uHistory;
x_curr=[0;0;-pi;0];
xHistory=x_curr;
for ct=1:200
    x_curr = VehicleDT0(x_curr, Opt_ctrl(ct+1));
    xHistory=[xHistory x_curr];
end
%}
figure(1)
subplot(3,2,1);hold on
plot(0:Ts:Duration,xHistory(1,:),"->",'Color','r','MarkerSize',1.5);
yline(xref(1),"--", 'Color','b');
xlabel('time');
ylabel('z');
title('cart position');
legend('z','Zref');
subplot(3,2,2);hold on
plot(0:Ts:Duration,xHistory(2,:),"->",'Color','r','MarkerSize',1.5);
yline(xref(2),"--", 'Color','b');
xlabel('time');
ylabel('zdot');
title('cart velocity');
legend('z_dot','Z_dot ref');
subplot(3,2,3);hold on
plot(0:Ts:Duration,xHistory(3,:),"->",'Color','r','MarkerSize',1.5);
yline(xref(3),"--" ,'Color','b');
xlabel('time');
ylabel('theta');
title('pendulum angle');
legend('theta','Theta ref');
subplot(3,2,4);hold on
plot(0:Ts:Duration,xHistory(4,:),"->",'Color','r','MarkerSize',1.5);
yline(xref(4),"--", 'Color','b');
xlabel('time');
ylabel('thetadot');
title('pendulum velocity');
legend('Theta dot','Theta dot ref');
subplot(3,2,5);hold on
plot(0:Ts:Duration,uHistory(1,:),"->",'Color','r','MarkerSize',1.5);
xlabel('time');
ylabel('u');
title('Input Force/N');
legend('Input');
subplot(3,2,6);hold on
plot(J_vec,'-ok')
title('Closed loop stage cost')
xlabel('Iterations')
ylabel('Stage cost')
hold off
hold off

figure(2)
plot(Time);
figure(3)
Theta=xHistory(3,:)';
hold on
threshold=Exit==-2;
%plot(0:Ts:Duration,xHistory(3,:)); hold on
plot(Theta(~threshold),'bo','DisplayName','Data')
hold on
plot(Theta(threshold),'r*','DisplayName','Outlier')
hold off
xlabel('time');
ylabel('theta');
title('pendulum angle');