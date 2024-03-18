function uopt = VSCFmincon(x,u0)
%% Extrinsic function used by Nonlinear MPC Block in the pendulum model
% Range of force: from -100 (to left) to 100 (to right)
    Ts=0.1;
    N=10;
    xref=[4,0,0.01,0]';
    LB = -100*ones(N,1);
    UB = 100*ones(N,1);
    Q = diag([10,1,20,0]);
    R = 0.1;
    lastmv = u0(1);
    COSTFUN = @(u) VSCObjectiveFCN(u, x, Ts, N, xref,lastmv,Q,R);
    CONSFUN = @(u) VSCConstraintFCN(u, x, Ts, N,xref);
    options = optimoptions('fmincon','Algorithm','sqp','Display','none');
    uopt = fmincon(COSTFUN,u0,[],[],[],[],LB,UB,CONSFUN,options);
end