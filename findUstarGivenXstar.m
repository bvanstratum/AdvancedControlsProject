function [A, B, Ustar] = fundUstarGivenXstar(params,F,dX,X,Xstar)
% syms r th dr dth  k m r0 br bt g ur uth 
%the dynamics given in the problem
% f1 = r*dth^2-k/m*(r-r0)+cos(th)-br*dr+ur
% f2 = -g/r*sin(th)-2*dr*dth/r-bt*dth+uth
syms Uth1 Uth2
U = [Uth1 Uth2 0].'

% r_zero     =  r == Xstar(1);
% theta_zero = th == Xstar(2); %theta zero pt
% Xdot = [dr dth f1 f2].'
% x    = [r th dr dth]
% u_sym = [ur uth].'
% set the dx vector to zero and solve for x
eomsAtoperatingPt = subs(F,dX,zeros(length(dX),1))
XdotEqZero = F(4:6)-U == zeros(size(dX(4:6)));
XdotEqZero = subs(XdotEqZero,X,Xstar');
XplusUstar = solve([XdotEqZero],[Uth1, Uth2])

%numerical Xstar
XplusUstar = double(...
            struct2array(...
                subs(XplusUstar,[       m        r0        k        g], ...
                                [params.m params.r0 params.k params.g]) ...
                        ) ...
                    )'

Xstar = XplusUstar(1:4)
Ustar = XplusUstar(5:end)
% take jacobians
A = jacobian(Xdot,x)
B = jacobian(Xdot,u_sym)
disp('the symbolic matrix')
A = subs(A,[r th dr dth],Xstar')
disp('with the values plugged in')
A = double(subs(A,[k m br g r0 bt],...
    [params.k params.m params.br params.g params.r0 params.bt]))
B = double(B)
% Xstar = [Zstar.r Zstar.th Zstar.dr Zstar.dth]';
% Ustar = [Zstar.ur Zstar.uth]';
if debug
%     this should be zero
    genODEFUN(0,Xstar,Ustar,params)
end
end