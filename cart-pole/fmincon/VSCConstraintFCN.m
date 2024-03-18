function [c, ceq] = VSCConstraintFCN(u,x,Ts,N,xref)
    
    %% Inequality constraints calculation
    xk = x;
    uk = u(1);
    c1=[];
    c2=[];
    c3=[];
    c4=[];
    xfinal=0;
    X=xk;
    for ct=1:N
        % Obtain plant state at next prediction step.
        xk1 = VehicleDT0(xk, uk);
        xk = xk1;
        X=[X, xk];
        if(ct==1)
            c1= xk1(1)-xref(1);
            c1=[c1; xk1(3)-xref(3)];
        end
        c2= xk1(1)-xref(1);
        c2=[c2; xk1(3)-xref(3)];
        if ct<N
            uk = u(ct+1);
        end
    end
    xfinal=X(:,end); %%terminal state
    for ct=2:N-1
        c3=[c3; X(1,ct)-xfinal(1)];
        c3=[c3; X(3,ct)-xfinal(3)];
    end
    for ct=2:N-1
        c4=[c4; X(1,ct)-X(1,ct+1)];
        c4=[c4; X(3,ct)-X(3,ct+1)];
    end
    c5=c4;
    c3=[c3; xfinal(1)-xref(1)];
    c3=[c3; xfinal(3)-xref(3)];
    c4=[c4; xfinal(1)-xref(1)];
    c4=[c4; xfinal(3)-xref(3)];    
    %c = [c; (xfinal-xref)'*P*(xfinal-xref) - alpha];
    c=c1;
    %c=[];
    %% No equality constraints
    %ceq = xfinal-xref;    %% Equality contraints
    ceq=[];
end
