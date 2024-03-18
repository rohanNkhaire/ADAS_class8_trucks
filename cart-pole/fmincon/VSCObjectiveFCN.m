function J = VSCObjectiveFCN(u,x,Ts,N,xref,u0,Q,R)
%% Nonlinear MPC design parameters

%% Cost Calculation
% Set initial plant states, controller output and cost.
xk = x;
uk = u(1);
J = 0;

Qf =[ 37.6212   20.2678  -51.4087  -11.7150
    20.2678   51.0094 -181.6907  -36.4531
    -51.4087 -181.6907  664.5378  132.0926
    -11.7150  -36.4531  132.0926   26.3550];

% Loop through each prediction step.
for ct=1:N
    % Obtain plant state at next prediction step.
    xk1 = VehicleDT0(xk, uk);
    % accumulate state tracking cost from x(k+1) to x(k+N).
    %J = J + (xref-xk1)'*Q*(xref-xk1);
    %J = J + (xref-xk)'*Q*(xref-xk);
    % accumulate MV rate of change cost from u(k) to u(k+N-1).
    
    if ct~=N
        J = J + (xref-xk1)'*Q*(xref-xk1);
    end
    J = J + (uk)'*R*(uk);
    %{
    if ct==1
        %J = J + (uk-u0)'*R*(uk-u0);
    else
        %J = J + (uk-u(ct-1))'*R*(uk-u(ct-1));
    end
    %}
    % Update xk and uk for the next prediction step.
    xk = xk1;
    if ct<N
        uk = u(ct+1);
    end
end
%J = J + (xref-xk1)'*Q*(xref-xk1); %Adding terminal cost

