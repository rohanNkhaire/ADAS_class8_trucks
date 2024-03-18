function [uopt,fval] = Cartpole_mpc_function(x_curr, uopt)
    %%
    % Set the sample time:
    Ts = 0.1;
    %%
    % Set the prediction horizon:
    N = 10;
    xref = [4;0;0;0];
    options = optimoptions('fmincon','Algorithm','sqp','Display','none');
    LB = -100*ones(N,1);
    UB = 100*ones(N,1);
    % Start simulation
    fprintf('Simulation started.  It might take a while...\n')
    COSTFUN = @(u) VSCObjectiveFCN(u,x_curr,Ts,N,xref,uopt(1));
    CONSFUN = @(u) VSCConstraintFCN(u,x_curr,Ts,N,xref);
    tic
    uopt = fmincon(COSTFUN,uopt,[],[],[],[],LB,UB,CONSFUN,options);  
    toc
    fval = 0;
    fprintf('Simulation finished!\n')
end

