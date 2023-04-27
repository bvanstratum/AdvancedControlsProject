function dX = GeneralODEfun(M,C,G,X,t,L1,mb,dXfun,U)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
M = M(t,X);
C = C(t,X);
G = G(t,X);
Th1  = X(1);
Th2  = X(2);
S    = X(3);
dTh2 = X(5);
dS   = X(6);
Cextra = [2*L1*dS*dTh2*mb*cos(Th1 - Th2)
          2*S*dS*dTh2*mb
          0];
Bdyn = [1   0
        0   1
        0   0];
dq = X(4:6);

ddq = M\(-C*dq.^2-Cextra-G+Bdyn*U);


% ddq_check = dXfun(t,X);
% [ddq1 ddq];
dX =[dq;ddq];
end

